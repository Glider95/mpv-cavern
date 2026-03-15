/*
 * ad_cavernpipe.c — CavernPipe audio decoder for mpv
 *
 * Sends compressed E-AC-3 / AC-3 bitstream to CavernPipeServer via a
 * Windows named pipe and receives rendered float32 PCM back.
 *
 * The protocol is strictly synchronous:
 *   1. Client connects and sends an 8-byte handshake.
 *   2. Client sends [u32 length][compressed bytes].
 *   3. Server replies with [u32 length][float32 PCM bytes].
 *   4. Repeat 2-3.
 *
 * We reuse mpv's lavc_process() helper so pin/EOF flow is handled
 * identically to ad_lavc.
 */

#include <string.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef _WIN32
#include <windows.h>
#else
#error "CavernPipe decoder currently supports Windows only."
#endif

#include "config.h"
#include "common/common.h"
#include "common/codecs.h"
#include "common/msg.h"
#include "audio/aframe.h"
#include "audio/format.h"
#include "audio/chmap.h"
#include "demux/packet.h"
#include "demux/stheader.h"
#include "filters/f_decoder_wrapper.h"
#include "filters/filter_internal.h"

#include <libavutil/error.h>

/* ------------------------------------------------------------------ */
/*  Constants                                                          */
/* ------------------------------------------------------------------ */

#define CAVERN_PIPE_NAME    L"\\\\.\\pipe\\CavernPipe"
#define DEFAULT_CHANNELS    6
#define DEFAULT_SAMPLE_RATE 48000

/*
 * We accumulate compressed data before sending it to the server so that
 * CavernPipe has enough material to decode and satisfy the mandatory-
 * frames requirement in a single round-trip.
 *
 * For E-AC-3: one frame = 1536 samples = 2048..6144 bytes (typical).
 * mandatory_frames=24 with update_rate=64 means the server needs
 * 24*64 = 1536 decoded samples = 1 full E-AC-3 frame.  But the E-AC-3
 * decoder reads ahead to the next frame header, so we need >=2 frames.
 *
 * Sending ~128 kB (several frames) per round-trip is safe and matches
 * the reference CavernPipeClient behaviour of sending up to 1 MB.
 */
#define SEND_BUF_TARGET     (32 * 1024)
#define SEND_BUF_MAX        (1 * 1024 * 1024)

/* How many float samples (per-channel) of decoded PCM we can buffer. */
#define PCM_BUF_SAMPLES     (256 * 1024)

/* ------------------------------------------------------------------ */
/*  Private state                                                      */
/* ------------------------------------------------------------------ */

struct priv {
    struct mp_codec_params *codec;

    /* Pipe handle */
    HANDLE pipe;
    bool   connected;

    /* Codec parameters */
    int channels;
    int sample_rate;
    int update_rate;
    int mandatory_frames;

    /* Accumulation buffer for compressed data to send */
    uint8_t *send_buf;
    size_t   send_fill;      /* bytes currently in send_buf */
    size_t   send_alloc;     /* allocated size */

    /* Buffer for received PCM (interleaved float32) */
    float   *pcm_buf;
    int      pcm_samples;    /* total samples (per-channel) buffered */
    int      pcm_read;       /* samples (per-channel) already consumed */

    /* Output format template */
    struct mp_aframe      *fmt;
    struct mp_aframe_pool *pool;

    /* PTS tracking */
    double next_pts;

    /* lavc_process helper state */
    struct lavc_state state;

    struct mp_decoder public;
};

/* ------------------------------------------------------------------ */
/*  Pipe helpers                                                       */
/* ------------------------------------------------------------------ */

static bool write_all(HANDLE h, const void *data, size_t len)
{
    const uint8_t *bp = data;
    while (len > 0) {
        DWORD n = 0;
        if (!WriteFile(h, bp, (DWORD)len, &n, NULL) || n == 0)
            return false;
        bp  += n;
        len -= n;
    }
    return true;
}

static bool read_all(HANDLE h, void *data, size_t len)
{
    uint8_t *bp = data;
    while (len > 0) {
        DWORD n = 0;
        if (!ReadFile(h, bp, (DWORD)len, &n, NULL) || n == 0)
            return false;
        bp  += n;
        len -= n;
    }
    return true;
}

/* ------------------------------------------------------------------ */
/*  Channel count from Cavern Driver settings                          */
/* ------------------------------------------------------------------ */

static int read_cavern_channel_count(void)
{
    char appdata[MAX_PATH];
    DWORD len = GetEnvironmentVariableA("APPDATA", appdata, sizeof(appdata));
    if (!len || len >= sizeof(appdata))
        return DEFAULT_CHANNELS;

    char path[MAX_PATH];
    if (snprintf(path, sizeof(path), "%s\\Cavern\\Save.dat", appdata) >= (int)sizeof(path))
        return DEFAULT_CHANNELS;

    FILE *f = fopen(path, "rb");
    if (!f)
        return DEFAULT_CHANNELS;

    char line[64] = {0};
    int ch = DEFAULT_CHANNELS;
    if (fgets(line, sizeof(line), f)) {
        int v = atoi(line);
        if (v > 0 && v <= 64)
            ch = v;
    }
    fclose(f);
    return ch;
}

/* ------------------------------------------------------------------ */
/*  Connection management                                              */
/* ------------------------------------------------------------------ */

static void close_pipe(struct priv *p)
{
    if (p->pipe != INVALID_HANDLE_VALUE) {
        /* Flush the pipe so the server sees the disconnect cleanly. */
        FlushFileBuffers(p->pipe);
        /* Disconnect our end; the server will get a broken-pipe error
         * on the next ReadInt32() and re-create the named pipe. */
        CloseHandle(p->pipe);
    }
    p->pipe = INVALID_HANDLE_VALUE;
    p->connected = false;
    p->send_fill = 0;
    p->pcm_samples = 0;
    p->pcm_read = 0;
}

static bool connect_pipe(struct mp_filter *f)
{
    struct priv *p = f->priv;

    if (p->connected)
        return true;

    if (!WaitNamedPipeW(CAVERN_PIPE_NAME, 2000)) {
        MP_VERBOSE(f, "CavernPipe: WaitNamedPipe failed (%lu)\n",
                   GetLastError());
        return false;
    }

    p->pipe = CreateFileW(CAVERN_PIPE_NAME,
                          GENERIC_READ | GENERIC_WRITE,
                          0, NULL, OPEN_EXISTING, 0, NULL);
    if (p->pipe == INVALID_HANDLE_VALUE) {
        MP_WARN(f, "CavernPipe: CreateFile failed (%lu)\n", GetLastError());
        return false;
    }

    /* Switch to byte-mode (in case the server created it as message mode) */
    DWORD mode = PIPE_READMODE_BYTE;
    SetNamedPipeHandleState(p->pipe, &mode, NULL, NULL);

    /*
     * Handshake — 8 bytes, must match CavernPipeProtocol.cs:
     *   byte 0   : BitDepth  (32 = float32)
     *   byte 1   : mandatory frames
     *   byte 2-3 : output channel count  (uint16 LE)
     *   byte 4-7 : update rate           (int32  LE)
     */
    uint8_t hs[8];
    hs[0] = 32;                                         /* Float32 */
    hs[1] = (uint8_t)p->mandatory_frames;
    hs[2] = (uint8_t)(p->channels & 0xFF);
    hs[3] = (uint8_t)((p->channels >> 8) & 0xFF);
    hs[4] = (uint8_t)(p->update_rate & 0xFF);
    hs[5] = (uint8_t)((p->update_rate >> 8) & 0xFF);
    hs[6] = (uint8_t)((p->update_rate >> 16) & 0xFF);
    hs[7] = (uint8_t)((p->update_rate >> 24) & 0xFF);

    if (!write_all(p->pipe, hs, sizeof(hs))) {
        MP_ERR(f, "CavernPipe: handshake write failed (%lu)\n",
               GetLastError());
        close_pipe(p);
        return false;
    }

    p->connected = true;
    MP_INFO(f, "CavernPipe: connected  ch=%d  rate=%d  update=%d  mandatory=%d\n",
            p->channels, p->sample_rate, p->update_rate, p->mandatory_frames);
    return true;
}

/* ------------------------------------------------------------------ */
/*  Send accumulated compressed data -> receive PCM                    */
/* ------------------------------------------------------------------ */

static bool send_and_receive(struct mp_filter *f)
{
    struct priv *p = f->priv;

    if (!p->connected || p->send_fill == 0)
        return false;

    /* --- send: [u32 length][compressed data] --- */
    uint32_t out_len = (uint32_t)p->send_fill;
    MP_DBG(f, "CavernPipe: sending %u bytes...\n", out_len);
    if (!write_all(p->pipe, &out_len, 4)) {
        MP_WARN(f, "CavernPipe: write length failed (%lu)\n", GetLastError());
        close_pipe(p);
        return false;
    }
    if (!write_all(p->pipe, p->send_buf, p->send_fill)) {
        MP_WARN(f, "CavernPipe: write payload failed (%lu)\n", GetLastError());
        close_pipe(p);
        return false;
    }
    FlushFileBuffers(p->pipe);

    MP_DBG(f, "CavernPipe: sent %u bytes, waiting for reply...\n", out_len);
    p->send_fill = 0;

    /* --- receive: [u32 length][float32 PCM] --- */
    uint32_t in_len = 0;
    if (!read_all(p->pipe, &in_len, 4)) {
        MP_WARN(f, "CavernPipe: read reply length failed (%lu)\n",
                GetLastError());
        close_pipe(p);
        return false;
    }

    MP_DBG(f, "CavernPipe: reply length = %u bytes\n", in_len);

    if (in_len == 0) {
        MP_DBG(f, "CavernPipe: server returned empty reply\n");
        return true;
    }

    int frame_bytes = sizeof(float) * p->channels;
    if (in_len % frame_bytes != 0) {
        MP_ERR(f, "CavernPipe: reply size %u not aligned to frame (%d ch)\n",
               in_len, p->channels);
        close_pipe(p);
        return false;
    }

    int total_floats  = in_len / sizeof(float);
    int total_samples = total_floats / p->channels;  /* per-channel */

    /* Compact PCM buffer: move unconsumed data to the front */
    int remain = p->pcm_samples - p->pcm_read;
    if (p->pcm_read > 0 && remain > 0) {
        memmove(p->pcm_buf,
                p->pcm_buf + p->pcm_read * p->channels,
                remain * p->channels * sizeof(float));
    }
    p->pcm_samples = (remain > 0) ? remain : 0;
    p->pcm_read = 0;

    /* Check if we have room */
    if (p->pcm_samples + total_samples > PCM_BUF_SAMPLES) {
        MP_WARN(f, "CavernPipe: PCM overflow (%d + %d > %d), dropping old data\n",
                p->pcm_samples, total_samples, PCM_BUF_SAMPLES);
        p->pcm_samples = 0;
    }

    /* Read PCM into our buffer */
    MP_DBG(f, "CavernPipe: reading %u bytes of PCM...\n", in_len);
    float *dst = p->pcm_buf + p->pcm_samples * p->channels;
    if (!read_all(p->pipe, dst, in_len)) {
        MP_WARN(f, "CavernPipe: read PCM payload failed (%lu)\n",
                GetLastError());
        close_pipe(p);
        return false;
    }
    p->pcm_samples += total_samples;

    MP_DBG(f, "CavernPipe: received %d samples (%u bytes), "
               "buffered %d total\n",
            total_samples, in_len, p->pcm_samples);
    return true;
}

/* ------------------------------------------------------------------ */
/*  Append compressed data to send buffer                              */
/* ------------------------------------------------------------------ */

static void append_compressed(struct priv *p, const uint8_t *data, size_t len)
{
    size_t needed = p->send_fill + len;
    if (needed > p->send_alloc) {
        size_t new_alloc = needed + SEND_BUF_TARGET;
        if (new_alloc > SEND_BUF_MAX)
            new_alloc = SEND_BUF_MAX;
        if (new_alloc < needed)
            new_alloc = needed;
        uint8_t *tmp = realloc(p->send_buf, new_alloc);
        if (!tmp)
            return;
        p->send_buf   = tmp;
        p->send_alloc = new_alloc;
    }
    memcpy(p->send_buf + p->send_fill, data, len);
    p->send_fill += len;
}

/* ------------------------------------------------------------------ */
/*  lavc_process callbacks                                             */
/* ------------------------------------------------------------------ */

/*
 * cp_send():  Called by lavc_process() when it has a compressed packet.
 *             We accumulate data and do a synchronous round-trip when
 *             we have enough.
 *
 * Returns:  0               packet consumed
 *           AVERROR(EAGAIN)  not used here
 */
static int cp_send(struct mp_filter *f, struct demux_packet *pkt)
{
    struct priv *p = f->priv;

    if (!pkt) {
        /* NULL pkt = flush/drain.  Send whatever we have accumulated. */
        if (p->send_fill > 0 && p->connected)
            send_and_receive(f);
        return 0;
    }

    if (!p->connected && !connect_pipe(f))
        return 0;   /* silently drop — will be retried on next packet */

    /* Track PTS from the first packet we see */
    if (p->next_pts == MP_NOPTS_VALUE && pkt->pts != MP_NOPTS_VALUE)
        p->next_pts = pkt->pts;

    append_compressed(p, pkt->buffer, pkt->len);

    MP_DBG(f, "CavernPipe: cp_send pkt=%zu bytes, accumulated=%zu / %zu\n",
           (size_t)pkt->len, p->send_fill, (size_t)SEND_BUF_TARGET);

    /*
     * Send when we have accumulated enough compressed data.
     * The reference CavernPipeClient sends up to 1 MB at a time.
     * We use SEND_BUF_TARGET (~128 kB) to balance latency vs. feeding
     * the decoder enough frames so it can satisfy mandatory-frames
     * (the E-AC-3 decoder also reads ahead past the current frame).
     */
    if (p->send_fill >= SEND_BUF_TARGET)
        send_and_receive(f);

    return 0;
}

/*
 * cp_receive(): Called by lavc_process() to pull a decoded frame.
 *
 * Returns:  0               frame written to *out
 *           AVERROR(EAGAIN)  no data ready, need more input
 *           AVERROR_EOF      end of stream
 */
static int cp_receive(struct mp_filter *f, struct mp_frame *out)
{
    struct priv *p = f->priv;

    int avail = p->pcm_samples - p->pcm_read;

    /*
     * Do NOT flush the send buffer here — the server needs enough
     * compressed data to decode (mandatoryFrames * updateRate samples)
     * before it will reply.  Flushing a partial buffer causes a deadlock.
     *
     * lavc_process() will keep feeding packets to cp_send() until
     * SEND_BUF_TARGET is reached; the drain (EOF) case is handled
     * by the NULL-packet path in cp_send().
     */

    if (avail <= 0)
        return AVERROR(EAGAIN);

    /* Build an mp_aframe with the available PCM */
    struct mp_aframe *af = mp_aframe_new_ref(p->fmt);
    if (!af)
        return AVERROR(EAGAIN);

    if (mp_aframe_pool_allocate(p->pool, af, avail) < 0) {
        talloc_free(af);
        return AVERROR(EAGAIN);
    }

    uint8_t **planes = mp_aframe_get_data_rw(af);
    if (!planes) {
        talloc_free(af);
        return AVERROR(EAGAIN);
    }

    memcpy(planes[0],
           p->pcm_buf + p->pcm_read * p->channels,
           avail * p->channels * sizeof(float));
    p->pcm_read += avail;

    if (p->next_pts != MP_NOPTS_VALUE) {
        mp_aframe_set_pts(af, p->next_pts);
        p->next_pts += (double)avail / p->sample_rate;
    }

    mp_aframe_sanitize_float(af);

    *out = MAKE_FRAME(MP_FRAME_AUDIO, af);
    return 0;
}

/* ------------------------------------------------------------------ */
/*  Filter callbacks                                                   */
/* ------------------------------------------------------------------ */

static void cavernpipe_process(struct mp_filter *f)
{
    struct priv *p = f->priv;
    lavc_process(f, &p->state, cp_send, cp_receive);
}

static void cavernpipe_reset(struct mp_filter *f)
{
    struct priv *p = f->priv;
    /* On seek: disconnect so CavernPipe clears its internal cache. */
    close_pipe(p);
    p->next_pts = MP_NOPTS_VALUE;
    p->state = (struct lavc_state){0};
}

static void cavernpipe_destroy(struct mp_filter *f)
{
    struct priv *p = f->priv;
    close_pipe(p);
    free(p->send_buf);
    free(p->pcm_buf);
}

static const struct mp_filter_info cavernpipe_filter = {
    .name       = "ad_cavernpipe",
    .priv_size  = sizeof(struct priv),
    .process    = cavernpipe_process,
    .reset      = cavernpipe_reset,
    .destroy    = cavernpipe_destroy,
};

/* ------------------------------------------------------------------ */
/*  Decoder creation                                                   */
/* ------------------------------------------------------------------ */

static struct mp_decoder *cavernpipe_create(struct mp_filter *parent,
                                            struct mp_codec_params *codec,
                                            const char *decoder)
{
    const char *cname = codec->codec ? codec->codec : "";

    if (strcmp(cname, "truehd") == 0) {
        MP_ERR(parent,
               "CavernPipe: TrueHD is not supported in this build.\n");
        return NULL;
    }

    struct mp_filter *f = mp_filter_create(parent, &cavernpipe_filter);
    if (!f)
        return NULL;

    mp_filter_add_pin(f, MP_PIN_IN,  "in");
    mp_filter_add_pin(f, MP_PIN_OUT, "out");

    struct priv *p  = f->priv;
    p->codec        = codec;
    p->pipe         = INVALID_HANDLE_VALUE;
    p->sample_rate  = codec->samplerate > 0 ? codec->samplerate
                                             : DEFAULT_SAMPLE_RATE;
    p->channels     = read_cavern_channel_count();
    p->next_pts     = MP_NOPTS_VALUE;
    p->public.f     = f;

    /*
     * Codec-specific handshake parameters.
     *
     * E-AC-3 (JOC/Atmos):
     *   update_rate=64, mandatory_frames=24 -> 1536 decoded samples
     *   before first reply (= exactly 1 E-AC-3 frame).
     *
     * AC-3:
     *   update_rate=256, mandatory_frames=6 -> 1536 decoded samples.
     */
    if (strcmp(cname, "eac3") == 0) {
        p->update_rate      = 64;
        p->mandatory_frames = 24;
    } else {
        p->update_rate      = 256;
        p->mandatory_frames = 6;
    }

    /* Allocate buffers */
    p->send_alloc = SEND_BUF_TARGET * 2;
    p->send_buf   = malloc(p->send_alloc);
    p->pcm_buf    = malloc(PCM_BUF_SAMPLES * p->channels * sizeof(float));
    if (!p->send_buf || !p->pcm_buf) {
        free(p->send_buf);
        free(p->pcm_buf);
        talloc_free(f);
        return NULL;
    }

    /* Output format template: interleaved float32 */
    p->fmt  = mp_aframe_create();
    p->pool = mp_aframe_pool_create(p);

    mp_aframe_set_format(p->fmt, AF_FORMAT_FLOAT);
    mp_aframe_set_rate(p->fmt, p->sample_rate);

    struct mp_chmap chmap;
    mp_chmap_from_channels(&chmap, p->channels);
    mp_aframe_set_chmap(p->fmt, &chmap);

    MP_INFO(f, "CavernPipe: ready  codec=%s  ch=%d  rate=%d  "
               "update=%d  mandatory=%d\n",
            cname, p->channels, p->sample_rate,
            p->update_rate, p->mandatory_frames);

    return &p->public;
}

/* ------------------------------------------------------------------ */
/*  Registration                                                       */
/* ------------------------------------------------------------------ */

static void add_decoders(struct mp_decoder_list *list)
{
    mp_add_decoder(list, "eac3", "cavernpipe", "CavernPipe Atmos");
    mp_add_decoder(list, "ac3",  "cavernpipe", "CavernPipe Atmos");
}

const struct mp_decoder_fns ad_cavernpipe = {
    .create       = cavernpipe_create,
    .add_decoders = add_decoders,
};
