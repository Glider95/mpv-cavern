/*
 * ad_cavernpipe.c - Diagnostic Version with Verbose Logging
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
}

static int connect_cavernpipe(struct cavernpipe_ctx *ctx) {
    if (ctx->connected) return 0;

    if (!WaitNamedPipeW(CAVERN_PIPE_NAME, 0)) {
        return -1; // Server not ready
    }

    ctx->hPipe = CreateFileW(CAVERN_PIPE_NAME, GENERIC_READ | GENERIC_WRITE, 
                             0, NULL, OPEN_EXISTING, 0, NULL);
    
    if (ctx->hPipe == INVALID_HANDLE_VALUE) {
        MP_ERR(ctx, "[CONN] CreateFileW failed. Error: %lu\n", GetLastError());
        return -1;
    }

    // Handshake: [BitDepth][MandatoryFrames][Channels(U16 LE)][UpdateRate(I32 LE)]
    uint8_t handshake[8] = { 32, 24, 6, 0, 64, 0, 0, 0 };
    DWORD written = 0;
    
    MP_INFO(ctx, "[CONN] Sending Handshake: 32-bit, 24 Mandatory, 6ch, 64 UpdateRate\n");
    if (!WriteFile(ctx->hPipe, handshake, 8, &written, NULL) || written != 8) {
        MP_ERR(ctx, "[CONN] Handshake write failed. Error: %lu\n", GetLastError());
        close_pipe(ctx);
        return -1;
    }
    FlushFileBuffers(ctx->hPipe);

    DWORD mode = PIPE_READMODE_BYTE | PIPE_NOWAIT;
    if (!SetNamedPipeHandleState(ctx->hPipe, &mode, NULL, NULL)) {
        MP_WARN(ctx, "[CONN] Could not set non-blocking mode.\n");
    }

    ctx->connected = true;
    MP_INFO(ctx, "[CONN] Handshake successful. Connection established.\n");
    return 0;
}

static BOOL read_exact(struct cavernpipe_ctx *ctx, void *buf, DWORD n) {
    DWORD total_read = 0;
    int retries = 0;
    while (total_read < n) {
        DWORD read = 0;
        if (!ReadFile(ctx->hPipe, (uint8_t*)buf + total_read, n - total_read, &read, NULL)) {
            DWORD err = GetLastError();
            if (err == ERROR_NO_DATA || err == ERROR_IO_PENDING) {
                if (retries++ > 100) return FALSE;
                Sleep(1);
                continue;
            }
            MP_ERR(ctx, "[READ] ReadFile failed. Error: %lu\n", err);
            return FALSE;
        }
        if (read == 0) return FALSE;
        total_read += read;
    }
    return TRUE;
}

static void cavernpipe_process(struct mp_filter *da) {
    struct cavernpipe_ctx *ctx = da->priv;

    if (!ctx->connected && connect_cavernpipe(ctx) < 0) {
        mp_pin_out_request_data(da->ppins[0]);
        return;
    }

    // --- 1. DRAIN PCM FROM SERVER ---
    DWORD avail = 0;
    if (PeekNamedPipe(ctx->hPipe, NULL, 0, NULL, &avail, NULL)) {
        if (avail > 0) {
            MP_VERBOSE(ctx, "[PIPE] Bytes available for reading: %lu\n", avail);
        }
    }

    while (avail >= 4) {
        uint32_t pcm_len = 0;
        DWORD peek_read = 0;
        
        if (PeekNamedPipe(ctx->hPipe, &pcm_len, 4, &peek_read, NULL, NULL) && peek_read == 4) {
            if (pcm_len == 0) {
                MP_VERBOSE(ctx, "[PCM] Received 0-byte marker, skipping.\n");
                read_exact(ctx, &pcm_len, 4); 
                goto next_check;
            }
            
            if (avail >= pcm_len + 4) {
                MP_INFO(ctx, "[PCM] Full payload ready: %u bytes. Processing...\n", pcm_len);
                read_exact(ctx, &pcm_len, 4); 
                
                int samples = pcm_len / (ctx->channels * sizeof(float));
                struct mp_aframe *out = mp_aframe_new_ref(ctx->fmt);
                
                if (out && mp_aframe_pool_allocate(ctx->pool, out, samples) >= 0) {
                    uint8_t **pdata = mp_aframe_get_data_rw(out);
                    if (read_exact(ctx, pdata[0], pcm_len)) {
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
                        MP_VERBOSE(ctx, "[FLOW] Dispatched PCM frame #%llu\n", ctx->dbg_pcm_frames_out);
                    } else {
                        MP_ERR(ctx, "[PCM] Failed to read expected payload bytes.\n");
                        talloc_free(out);
                    }
                } else {
                    MP_ERR(ctx, "[PCM] mpv aframe allocation failed.\n");
                }
            } else {
                MP_VERBOSE(ctx, "[PCM] Incomplete payload: %lu/%u bytes available.\n", avail - 4, pcm_len);
                break; 
            }
        } else {
            break;
        }

    next_check:
        if (!PeekNamedPipe(ctx->hPipe, NULL, 0, NULL, &avail, NULL)) break;
    }

    // --- 2. INGEST FROM MPV ---
    if (mp_pin_out_has_data(da->ppins[0])) {
        struct mp_frame inframe = mp_pin_out_read(da->ppins[0]);
        if (inframe.type == MP_FRAME_PACKET) {
            struct demux_packet *pkt = inframe.data;
            uint32_t p_len = (uint32_t)pkt->len;
            DWORD written = 0;

            MP_VERBOSE(ctx, "[BITSTREAM] Sending packet: %u bytes\n", p_len);
            if (WriteFile(ctx->hPipe, &p_len, 4, &written, NULL) &&
                WriteFile(ctx->hPipe, pkt->buffer, p_len, &written, NULL)) {
                
                if (pkt->pts != MP_NOPTS_VALUE) {
                    ctx->pts_queue[ctx->pts_tail] = pkt->pts;
                    ctx->pts_tail = (ctx->pts_tail + 1) % MAX_PTS_QUEUE;
                }
                ctx->dbg_packets_in++;
                
                // Real-time render trigger strategy
                // We send a 0 trigger only after the initial mandatory frame buffer is filled
                if (ctx->dbg_packets_in > 24 && (ctx->dbg_packets_in % 4 == 0)) {
                    uint32_t trigger = 0;
                    WriteFile(ctx->hPipe, &trigger, 4, &written, NULL);
                    MP_VERBOSE(ctx, "[BITSTREAM] Sent periodic render trigger (0).\n");
                }
            } else {
                MP_ERR(ctx, "[BITSTREAM] Write failed. Error: %lu\n", GetLastError());
            }
            talloc_free(pkt);
        } else if (inframe.type == MP_FRAME_EOF) {
            MP_INFO(ctx, "[STATE] EOF reached. Sending final flush.\n");
            uint32_t flush = 0;
            DWORD written = 0;
            WriteFile(ctx->hPipe, &flush, 4, &written, NULL);
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
    MP_INFO(ctx, "[STATE] Filter reset.\n");
    close_pipe(ctx);
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
    ctx->hPipe = INVALID_HANDLE_VALUE;
    
    ctx->fmt = mp_aframe_create();
    mp_aframe_set_format(ctx->fmt, AF_FORMAT_FLOAT); 
    mp_aframe_set_rate(ctx->fmt, 48000);
    
    struct mp_chmap chmap;
    mp_chmap_from_channels(&chmap, 6); 
    mp_aframe_set_chmap(ctx->fmt, &chmap);

    MP_INFO(ctx, "[STATE] Decoder created.\n");
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
