/*
 * ad_cavernpipe.c - CavernPipe Audio Decoder
 *
 * Protocol (per CavernPipe Bitstream Structure.txt):
 * - Handshake: 8 bytes [bitdepth, mandatoryFrames, channels(UInt16 LE), updateRate(Int32 LE)]
 * - Send: [4-byte LE length][compressed audio data]
 * - Render: Send 0 (4-byte LE) to trigger rendering of cached data
 * - Receive: [4-byte LE length][rendered PCM data]
 */

#include <string.h>
#include <windows.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "config.h"
#include "common/common.h"
#include "common/codecs.h"
#include "common/msg.h"
#include "audio/aframe.h"
#include "audio/format.h"
#include "audio/chmap.h"
#include "audio/fmt-conversion.h"
#include "audio/chmap_avchannel.h"
#include "demux/packet.h"
#include "filters/f_decoder_wrapper.h"
#include "filters/filter_internal.h"

#define CAVERN_PIPE_NAME L"\\\\.\\pipe\\CavernPipe"
#define MAX_PTS_QUEUE 4096

struct queued_packet {
    uint8_t *buf;
    size_t size;
    struct queued_packet *next;
};

struct cavernpipe_ctx {
    struct mp_log *log;
    HANDLE hPipe;
    bool connected;
    int channels;
    int output_channels;  // From Save.dat or default 6
    struct mp_aframe *fmt;
    struct mp_aframe_pool *pool;
    struct mp_decoder public;

    // Protocol constants from handshake
    int mandatory_frames;   // From handshake: byte 1
    int update_rate;        // From handshake: bytes 4-7 (Int32 LE)

    // Track bytes sent for mandatory frame requirement
    uint64_t bytes_sent;

    double pts_queue[MAX_PTS_QUEUE];
    int pts_head, pts_tail;
    double current_pts;

    struct queued_packet *q_head;
    struct queued_packet *q_tail;

    uint64_t dbg_packets_in;
    uint64_t dbg_pcm_frames_out;
};

static void close_pipe(struct cavernpipe_ctx *ctx) {
    if (ctx->hPipe && ctx->hPipe != INVALID_HANDLE_VALUE) {
        MP_INFO(ctx, "[PIPE] Closing handle.\n");
        CloseHandle(ctx->hPipe);
    }
    ctx->hPipe = INVALID_HANDLE_VALUE;
    ctx->connected = false;

    struct queued_packet *curr = ctx->q_head;
    while (curr) {
        struct queued_packet *next = curr->next;
        free(curr->buf);
        free(curr);
        curr = next;
    }
    ctx->q_head = ctx->q_tail = NULL;
}

static int connect_cavernpipe(struct cavernpipe_ctx *ctx) {
    if (ctx->connected) return 0;

    // Wait for pipe
    if (!WaitNamedPipeW(CAVERN_PIPE_NAME, 10000)) {
        MP_ERR(ctx, "[CONNECT] Pipe not available.\n");
        return -1;
    }

    ctx->hPipe = CreateFileW(CAVERN_PIPE_NAME, GENERIC_READ | GENERIC_WRITE,
                             0, NULL, OPEN_EXISTING, 0, NULL);

    if (ctx->hPipe == INVALID_HANDLE_VALUE) {
        MP_ERR(ctx, "[CONNECT] Failed: %lu\n", GetLastError());
        return -1;
    }

    MP_INFO(ctx, "[PIPE] Connected. Sending Handshake...\n");

    // Per CavernPipe Bitstream Structure.txt:
    // Byte 0: BitDepth (32 = float32)
    // Byte 1: Mandatory frames (24 for E-AC-3/JOC)
    // Bytes 2-3: Output channel count (UInt16 LE) - get from Save.dat or default 6
    // Bytes 4-7: Update rate (Int32 LE = 64 for E-AC-3)
    uint8_t handshake[8] = {
        32,                         // Byte 0: BitDepth = 32 (float32)
        24,                         // Byte 1: Mandatory frames = 24
        (uint8_t)(ctx->output_channels & 0xFF),        // Bytes 2-3: Channels (UInt16 LE)
        (uint8_t)((ctx->output_channels >> 8) & 0xFF),
        64, 0, 0, 0                // Bytes 4-7: UpdateRate = 64 (Int32 LE)
    };

    DWORD written = 0;
    if (!WriteFile(ctx->hPipe, handshake, 8, &written, NULL) || written != 8) {
        MP_ERR(ctx, "[HANDSHAKE] Write failed: %lu\n", GetLastError());
        close_pipe(ctx);
        return -1;
    }
    FlushFileBuffers(ctx->hPipe);

    // Use blocking mode - CavernPipeServer uses blocking I/O
    // The protocol (length prefix + render command) handles flow control
    DWORD mode = PIPE_READMODE_BYTE | PIPE_WAIT;
    if (!SetNamedPipeHandleState(ctx->hPipe, &mode, NULL, NULL)) {
        MP_WARN(ctx, "[PIPE] Failed to set blocking mode: %lu\n", GetLastError());
    }

    ctx->connected = true;
    ctx->mandatory_frames = 24;
    ctx->update_rate = 64;
    ctx->bytes_sent = 0;
    MP_INFO(ctx, "[HANDSHAKE] Done. Channels=%d, MandatoryFrames=24, UpdateRate=64.\n",
            ctx->output_channels);
    return 0;
}

// Read exactly n bytes from pipe (blocking)
static BOOL read_exact(HANDLE hPipe, void *buf, DWORD n) {
    DWORD total_read = 0;
    while (total_read < n) {
        DWORD read = 0;
        if (!ReadFile(hPipe, (uint8_t*)buf + total_read, n - total_read, &read, NULL)) {
            return FALSE;
        }
        if (read == 0) {
            // Pipe broken or empty
            return FALSE;
        }
        total_read += read;
    }
    return TRUE;
}

// Write exactly n bytes to pipe
static BOOL write_exact(HANDLE hPipe, const void *buf, DWORD n) {
    DWORD total_written = 0;
    while (total_written < n) {
        DWORD written = 0;
        if (!WriteFile(hPipe, (const uint8_t*)buf + total_written, n - total_written, &written, NULL)) {
            return FALSE;
        }
        if (written == 0) {
            return FALSE;
        }
        total_written += written;
    }
    return TRUE;
}

// Read 4-byte little-endian length
static uint32_t read_length(HANDLE hPipe) {
    uint32_t len = 0;
    if (read_exact(hPipe, &len, 4)) {
        // Already little-endian on x86/Windows
        return len;
    }
    return 0;
}

static void cavernpipe_process(struct mp_filter *da) {
    struct cavernpipe_ctx *ctx = da->priv;

    if (!ctx->connected && connect_cavernpipe(ctx) < 0) {
        mp_pin_out_request_data(da->ppins[0]);
        return;
    }

    // --- 1. DRAIN PCM FROM SERVER ---
    // Per protocol: receive [4-byte LE length][PCM data]
    DWORD avail = 0;
    if (PeekNamedPipe(ctx->hPipe, NULL, 0, NULL, &avail, NULL) && avail >= 4) {
        // Read 4-byte length prefix (little-endian)
        uint32_t pcm_size = read_length(ctx->hPipe);

        if (pcm_size > 0 && pcm_size <= avail - 4) {
            int samples = pcm_size / (ctx->output_channels * sizeof(float));

            struct mp_aframe *out = mp_aframe_new_ref(ctx->fmt);
            if (out && mp_aframe_pool_allocate(ctx->pool, out, samples) >= 0) {
                uint8_t **pdata = mp_aframe_get_data_rw(out);

                if (read_exact(ctx->hPipe, pdata[0], pcm_size)) {
                    mp_aframe_set_size(out, samples);

                    // Handle PTS
                    if (ctx->pts_head != ctx->pts_tail) {
                        ctx->current_pts = ctx->pts_queue[ctx->pts_head];
                        ctx->pts_head = (ctx->pts_head + 1) % MAX_PTS_QUEUE;
                    }
                    if (ctx->current_pts != MP_NOPTS_VALUE) {
                        mp_aframe_set_pts(out, ctx->current_pts);
                        ctx->current_pts += (double)samples / 48000.0;
                    }

                    mp_pin_in_write(da->ppins[1], MAKE_FRAME(MP_FRAME_AUDIO, out));
                    ctx->dbg_pcm_frames_out++;

                    if (ctx->dbg_pcm_frames_out % 50 == 0) {
                        MP_INFO(ctx, "[PCM-OUT] %llu frames (%u bytes, %d samples)\n",
                                ctx->dbg_pcm_frames_out, pcm_size, samples);
                    }
                } else {
                    talloc_free(out);
                }
            }
        }
    }

    // --- 2. FLUSH BITSTREAM QUEUE ---
    // Per protocol: send [4-byte LE length][compressed data]
    while (ctx->q_head) {
        // Send: [4-byte LE length][data]
        uint32_t len = (uint32_t)ctx->q_head->size;
        if (!write_exact(ctx->hPipe, &len, 4)) {
            MP_WARN(ctx, "[SEND] Failed to write length prefix\n");
            break;
        }

        if (!write_exact(ctx->hPipe, ctx->q_head->buf, (DWORD)ctx->q_head->size)) {
            MP_WARN(ctx, "[SEND] Failed to write data\n");
            break;
        }

        // Track raw bytes sent for debugging
        ctx->bytes_sent += ctx->q_head->size;

        // Packet sent successfully
        struct queued_packet *tmp = ctx->q_head;
        ctx->q_head = ctx->q_head->next;
        if (!ctx->q_head) ctx->q_tail = NULL;
        free(tmp->buf);
        free(tmp);

        MP_DBG(ctx, "[SEND] Sent %u bytes (total: %llu)\n", len, ctx->bytes_sent);
    }

    // --- 3. SEND RENDER COMMAND ---
    // Per protocol: send 0 to trigger rendering of cached data
    // This tells the server to process what it has and send back PCM
    if (ctx->q_head == NULL && ctx->connected && ctx->bytes_sent > 0) {
        uint32_t zero = 0;
        if (write_exact(ctx->hPipe, &zero, 4)) {
            MP_DBG(ctx, "[RENDER] Sent render command (0)\n");
            ctx->bytes_sent = 0;  // Reset after sending render command
        }
    }

    // --- 4. INGEST FROM MPV ---
    if (mp_pin_out_has_data(da->ppins[0])) {
        struct mp_frame inframe = mp_pin_out_read(da->ppins[0]);
        if (inframe.type == MP_FRAME_PACKET) {
            struct demux_packet *pkt = inframe.data;

            // Queue the compressed data (will be sent with length prefix in step 2)
            uint8_t *send_buf = malloc(pkt->len);
            if (send_buf) {
                memcpy(send_buf, pkt->buffer, pkt->len);

                if (pkt->pts != MP_NOPTS_VALUE) {
                    ctx->pts_queue[ctx->pts_tail] = pkt->pts;
                    ctx->pts_tail = (ctx->pts_tail + 1) % MAX_PTS_QUEUE;
                }

                struct queued_packet *qp = malloc(sizeof(struct queued_packet));
                qp->buf = send_buf;
                qp->size = pkt->len;
                qp->next = NULL;

                if (ctx->q_tail) ctx->q_tail->next = qp;
                else ctx->q_head = qp;
                ctx->q_tail = qp;

                ctx->dbg_packets_in++;
                if (ctx->dbg_packets_in % 100 == 0) {
                    MP_INFO(ctx, "[FLOW] Atmos Packets In: %llu\n", ctx->dbg_packets_in);
                }
            }
            talloc_free(pkt);
        } else if (inframe.type == MP_FRAME_EOF) {
            MP_INFO(ctx, "[EOF] Sending flush.\n");
            // Send render command with no new data to flush remaining PCM
            uint32_t zero = 0;
            write_exact(ctx->hPipe, &zero, 4);
            FlushFileBuffers(ctx->hPipe);
            mp_pin_in_write(da->ppins[1], inframe);
        } else {
            mp_pin_in_write(da->ppins[1], inframe);
        }
        mp_filter_internal_mark_progress(da);
    }

    mp_pin_out_request_data(da->ppins[0]);
}

static void cavernpipe_reset(struct mp_filter *da) {
    struct cavernpipe_ctx *ctx = da->priv;
    MP_INFO(ctx, "[RESET] Reconnecting...\n");
    close_pipe(da->priv);
    ctx->pts_head = ctx->pts_tail = 0;
    ctx->current_pts = MP_NOPTS_VALUE;
    ctx->dbg_packets_in = 0;
    ctx->dbg_pcm_frames_out = 0;
}

static void cavernpipe_destroy(struct mp_filter *da) {
    close_pipe(da->priv);
}

static const struct mp_filter_info cavernpipe_filter = {
    .name = "ad_cavernpipe",
    .priv_size = sizeof(struct cavernpipe_ctx),
    .process = cavernpipe_process,
    .reset = cavernpipe_reset,
    .destroy = cavernpipe_destroy,
};

static struct mp_decoder *cavernpipe_create(struct mp_filter *parent,
                                            struct mp_codec_params *codec,
                                            const char *decoder)
{
    struct mp_filter *da = mp_filter_create(parent, &cavernpipe_filter);
    if (!da) return NULL;

    mp_filter_add_pin(da, MP_PIN_IN, "in");
    mp_filter_add_pin(da, MP_PIN_OUT, "out");

    struct cavernpipe_ctx *ctx = da->priv;
    ctx->log = mp_log_new(da, parent->log, NULL);
    ctx->pool = mp_aframe_pool_create(ctx);
    ctx->public.f = da;
    ctx->hPipe = INVALID_HANDLE_VALUE;

    // Get channel count from Save.dat or default to 6
    // Per doc: first line of %appdata%\Cavern\Save.dat is channel count
    ctx->output_channels = 6;  // Default
    // TODO: Could read from Save.dat here

    ctx->channels = ctx->output_channels;

    ctx->fmt = mp_aframe_create();
    mp_aframe_set_format(ctx->fmt, AF_FORMAT_FLOAT);
    mp_aframe_set_rate(ctx->fmt, 48000);

    struct mp_chmap chmap;
    mp_chmap_from_channels(&chmap, ctx->output_channels);
    mp_aframe_set_chmap(ctx->fmt, &chmap);

    MP_INFO(ctx, "[STATE] CavernPipe Filter Initialized (%d-channel).\n", ctx->output_channels);
    return &ctx->public;
}

static void add_decoders(struct mp_decoder_list *list) {
    mp_add_decoder(list, "eac3", "cavernpipe", "CavernPipe Atmos");
    mp_add_decoder(list, "truehd", "cavernpipe", "CavernPipe Atmos");
}

const struct mp_decoder_fns ad_cavernpipe = {
    .create = cavernpipe_create,
    .add_decoders = add_decoders,
};
