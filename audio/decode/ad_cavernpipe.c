/*
 * ad_cavernpipe.c - Diagnostic Version with Protocol Fixes
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
    int samplerate;
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

    // Soft-check to prevent mpv startup deadlock if server is busy
    if (!WaitNamedPipeW(CAVERN_PIPE_NAME, 0)) return -1;

    ctx->hPipe = CreateFileW(CAVERN_PIPE_NAME, GENERIC_READ | GENERIC_WRITE, 
                             0, NULL, OPEN_EXISTING, 0, NULL);
    
    if (ctx->hPipe == INVALID_HANDLE_VALUE) return -1;

    // PROTOCOL: Server expects [Int32 SampleRate][Int32 Channels]
    int32_t handshake[2] = { (int32_t)ctx->samplerate, (int32_t)ctx->channels };
    DWORD written = 0;
    if (!WriteFile(ctx->hPipe, handshake, 8, &written, NULL) || written != 8) {
        close_pipe(ctx);
        return -1;
    }
    FlushFileBuffers(ctx->hPipe);

    // Set to non-blocking mode for processing
    DWORD mode = PIPE_READMODE_BYTE | PIPE_NOWAIT;
    SetNamedPipeHandleState(ctx->hPipe, &mode, NULL, NULL);

    ctx->connected = true;
    MP_INFO(ctx, "[PIPE] Connected & Handshaked (%dHz/%dch).\n", ctx->samplerate, ctx->channels);
    return 0;
}

static void cavernpipe_process(struct mp_filter *da) {
    struct cavernpipe_ctx *ctx = da->priv;

    if (!ctx->connected && connect_cavernpipe(ctx) < 0) {
        mp_pin_out_request_data(da->ppins[0]);
        return;
    }

    // --- 1. DRAIN ALL PCM FROM SERVER (While loop prevents pipe clogging) ---
    DWORD avail = 0;
    while (PeekNamedPipe(ctx->hPipe, NULL, 0, NULL, &avail, NULL) && avail >= 4) {
        uint32_t pcm_size = 0;
        DWORD peek_read = 0;
        
        if (PeekNamedPipe(ctx->hPipe, &pcm_size, 4, &peek_read, NULL, NULL) && peek_read == 4) {
            
            if (pcm_size == 0) {
                // Empty header, consume and continue
                read_exact(ctx->hPipe, &pcm_size, 4);
                continue;
            } 
            else if (avail >= pcm_size + 4) {
                // Header + Full payload is ready
                read_exact(ctx->hPipe, &pcm_size, 4);
                int samples = pcm_size / (ctx->output_channels * sizeof(float));
                struct mp_aframe *out = mp_aframe_new_ref(ctx->fmt);
                
                if (out && mp_aframe_pool_allocate(ctx->pool, out, samples) >= 0) {
                    uint8_t **pdata = mp_aframe_get_data_rw(out);

                    if (read_exact(ctx->hPipe, pdata[0], pcm_size)) {
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
                            MP_INFO(ctx, "[PCM-OUT] %llu frames (%u bytes)\n", ctx->dbg_pcm_frames_out, pcm_size);
                        }
                    } else {
                        talloc_free(out);
                    }
                } else {
                    // Allocation failed, but we must drain the pipe to prevent desync
                    uint8_t *trash = malloc(pcm_size);
                    if (trash) {
                        read_exact(ctx->hPipe, trash, pcm_size);
                        free(trash);
                    }
                }
            } else {
                // Not enough data for the full PCM payload yet, wait for next cycle
                break;
            }
        } else {
            break;
        }
    }

    // --- 2. INGEST & SEND BITSTREAM (No zero-trigger!) ---
    if (mp_pin_out_has_data(da->ppins[0])) {
        struct mp_frame inframe = mp_pin_out_read(da->ppins[0]);
        if (inframe.type == MP_FRAME_PACKET) {
            struct demux_packet *pkt = inframe.data;

            uint32_t len = (uint32_t)pkt->len;
            if (write_exact(ctx->hPipe, &len, 4) && write_exact(ctx->hPipe, pkt->buffer, pkt->len)) {
                
                // NOTE: We DO NOT send '0' here anymore. 
                // We just feed the stream naturally and let Cavern return PCM when ready.

                if (pkt->pts != MP_NOPTS_VALUE) {
                    ctx->pts_queue[ctx->pts_tail] = pkt->pts;
                    ctx->pts_tail = (ctx->pts_tail + 1) % MAX_PTS_QUEUE;
                }

                ctx->dbg_packets_in++;
                if (ctx->dbg_packets_in % 100 == 0) {
                    MP_INFO(ctx, "[FLOW] Atmos Packets In: %llu\n", ctx->dbg_packets_in);
                }
            } else {
                MP_WARN(ctx, "[PIPE] Write failure during packet send.\n");
            }
            talloc_free(pkt);
        } else if (inframe.type == MP_FRAME_EOF) {
            MP_INFO(ctx, "[EOF] Sending final flush command.\n");
            // Send flush command (0) ONLY on EOF
            uint32_t render_trigger = 0;
            write_exact(ctx->hPipe, &render_trigger, 4);
            mp_pin_in_write(da->ppins[1], inframe);
        } else {
            mp_pin_in_write(da->ppins[1], inframe);
        }
        mp_filter_internal_mark_progress(da);
    }

    mp_pin_out_request_data(da->ppins[0]);
}

static void cavernpipe_reset(struct mp_filter *da) {
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
    ctx->hPipe = INVALID_HANDLE_VALUE;
    
    ctx->channels = 6;
    ctx->samplerate = 48000;
    
    ctx->fmt = mp_aframe_create();
    mp_aframe_set_format(ctx->fmt, AF_FORMAT_FLOAT); 
    mp_aframe_set_rate(ctx->fmt, ctx->samplerate);
    
    struct mp_chmap chmap;
    mp_chmap_from_channels(&chmap, ctx->channels); 
    mp_aframe_set_chmap(ctx->fmt, &chmap);

    MP_INFO(ctx, "[STATE] CavernPipe Filter Initialized.\n");
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
