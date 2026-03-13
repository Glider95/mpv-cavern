/*
 * ad_cavernpipe.c - Raw Stream & Non-Blocking Queue Version
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
    struct mp_aframe *fmt;
    struct mp_aframe_pool *pool;
    struct mp_decoder public;

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
    ctx->hPipe = NULL;
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

    ctx->hPipe = CreateFileW(CAVERN_PIPE_NAME, GENERIC_READ | GENERIC_WRITE, 
                             0, NULL, OPEN_EXISTING, 0, NULL);
    
    if (ctx->hPipe == INVALID_HANDLE_VALUE) return -1;

    MP_INFO(ctx, "[PIPE] Connected. Sending Handshake (32-bit Float, 6ch)...\n");
    
    // Handshake based on previous successful connections
    uint8_t handshake[8] = { 32, 24, 6, 0, 64, 0, 0, 0 };
    DWORD written = 0;
    if (!WriteFile(ctx->hPipe, handshake, 8, &written, NULL) || written != 8) {
        MP_ERR(ctx, "[PIPE] Handshake write failed. Error: %lu\n", GetLastError());
        close_pipe(ctx);
        return -1;
    }
    FlushFileBuffers(ctx->hPipe);

    // CRITICAL: Non-blocking mode prevents mpv from hanging when the pipe backs up
    DWORD mode = PIPE_READMODE_BYTE | PIPE_NOWAIT;
    if (!SetNamedPipeHandleState(ctx->hPipe, &mode, NULL, NULL)) {
        MP_ERR(ctx, "[PIPE] Failed to set non-blocking mode.\n");
    }

    ctx->connected = true;
    MP_INFO(ctx, "[PIPE] Handshake confirmed. Ready for raw streaming.\n");
    return 0;
}

static void cavernpipe_process(struct mp_filter *da) {
    struct cavernpipe_ctx *ctx = da->priv;

    if (!ctx->connected && connect_cavernpipe(ctx) < 0) {
        mp_pin_out_request_data(da->ppins[0]);
        return;
    }

    // --- 1. DRAIN RAW PCM FROM SERVER ---
    DWORD avail = 0;
    // Peek safely tells us how many bytes are ready
    if (PeekNamedPipe(ctx->hPipe, NULL, 0, NULL, &avail, NULL) && avail >= (ctx->channels * sizeof(float))) {
        // Calculate how many complete samples we can read (stripping partial floats)
        uint32_t bytes_to_read = avail - (avail % (ctx->channels * sizeof(float)));
        int samples = bytes_to_read / (ctx->channels * sizeof(float));
        
        struct mp_aframe *out = mp_aframe_new_ref(ctx->fmt);
        if (out && mp_aframe_pool_allocate(ctx->pool, out, samples) >= 0) {
            uint8_t **pdata = mp_aframe_get_data_rw(out);
            DWORD read = 0;
            
            // Read the raw float stream directly into mpv's audio buffer
            if (ReadFile(ctx->hPipe, pdata[0], bytes_to_read, &read, NULL) && read > 0) {
                // Adjust frame size in case of a rare partial read
                samples = read / (ctx->channels * sizeof(float));
                mp_aframe_set_size(out, samples);

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
                    MP_INFO(ctx, "[FLOW] PCM Frames Out: %llu (Read %lu bytes)\n", 
                            ctx->dbg_pcm_frames_out, read);
                }
            } else {
                talloc_free(out);
            }
        }
    }

    // --- 2. FLUSH RAW BITSTREAM QUEUE ---
    while (ctx->q_head) {
        DWORD written = 0;
        BOOL success = WriteFile(ctx->hPipe, ctx->q_head->buf, (DWORD)ctx->q_head->size, &written, NULL);

        if (success && written == ctx->q_head->size) {
            // Entire packet written successfully
            struct queued_packet *tmp = ctx->q_head;
            ctx->q_head = ctx->q_head->next;
            if (!ctx->q_head) ctx->q_tail = NULL;
            free(tmp->buf);
            free(tmp);
        } else if (success && written > 0) {
            // Partial write: Pipe filled up mid-packet.
            // Shift the unwritten bytes to the front of the buffer and try again next loop.
            ctx->q_head->size -= written;
            memmove(ctx->q_head->buf, ctx->q_head->buf + written, ctx->q_head->size);
            break; 
        } else {
            // Write failed (pipe is totally full right now)
            break; 
        }
    }

    // --- 3. INGEST FROM MPV ---
    if (mp_pin_out_has_data(da->ppins[0])) {
        struct mp_frame inframe = mp_pin_out_read(da->ppins[0]);
        if (inframe.type == MP_FRAME_PACKET) {
            struct demux_packet *pkt = inframe.data;
            
            // NO 4-BYTE PREFIX. Sending pure payload.
            size_t full_size = pkt->len;
            uint8_t *send_buf = malloc(full_size);
            
            if (send_buf) {
                memcpy(send_buf, pkt->buffer, pkt->len);

                if (pkt->pts != MP_NOPTS_VALUE) {
                    ctx->pts_queue[ctx->pts_tail] = pkt->pts;
                    ctx->pts_tail = (ctx->pts_tail + 1) % MAX_PTS_QUEUE;
                }

                struct queued_packet *qp = malloc(sizeof(struct queued_packet));
                qp->buf = send_buf; 
                qp->size = full_size; 
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
            MP_INFO(ctx, "[DEBUG] EOF Reached. Sending 0-byte pipeline flush.\n");
            uint8_t zero = 0;
            DWORD written;
            WriteFile(ctx->hPipe, &zero, 1, &written, NULL);
            mp_pin_in_write(da->ppins[1], inframe);
        } else {
            MP_INFO(ctx, "[DEBUG] Passing non-packet frame (Type %d)\n", inframe.type);
            mp_pin_in_write(da->ppins[1], inframe);
        }
        mp_filter_internal_mark_progress(da);
    }
    mp_pin_out_request_data(da->ppins[0]);
}

static void cavernpipe_reset(struct mp_filter *da) {
    struct cavernpipe_ctx *ctx = da->priv;
    if (ctx) MP_INFO(ctx, "[STATE] Filter Reset.\n");
    close_pipe(da->priv);
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
    ctx->channels = 6;
    ctx->dbg_packets_in = 0;
    ctx->dbg_pcm_frames_out = 0;
    
    ctx->fmt = mp_aframe_create();
    mp_aframe_set_format(ctx->fmt, AF_FORMAT_FLOAT); 
    mp_aframe_set_rate(ctx->fmt, 48000);
    
    struct mp_chmap chmap;
    mp_chmap_from_channels(&chmap, 6); 
    mp_aframe_set_chmap(ctx->fmt, &chmap);

    MP_INFO(ctx, "[STATE] CavernPipe Filter Initialized (6-channel 5.1).\n");

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