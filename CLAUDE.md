# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is **MPV v0.41.0** with an integrated CavernPipe audio decoder for Dolby Atmos playback. The modification allows mpv to send compressed audio (E-AC-3/TrueHD/AC-3) to CavernPipe for spatial audio rendering, enabling real-time Atmos decoding without pre-conversion.

### Signal Flow
```
Media File (E-AC-3/TrueHD/AC-3)
    ↓
MPV (with ad_cavernpipe decoder)
    ↓ [compressed frame via named pipe]
CavernPipeServer (Atmos rendering: 12ch → 8ch)
    ↓ [PCM 8ch]
Snapserver → Network → Snapclients → ESP32 Speakers
```

## Build Commands

```bash
# Setup build directory
meson setup build

# Compile
meson compile -C build

# Install
meson install -C build
```

For Windows builds, see `DOCS/compile-windows.md`.

## Key Modified Files

| File | Purpose |
|------|---------|
| `audio/decode/ad_cavernpipe.c` | Custom CavernPipe audio decoder |
| `filters/f_decoder_wrapper.c` | Decoder selection logic |
| `filters/f_decoder_wrapper.h` | Decoder wrapper header |
| `meson.build` | Build system (includes ad_cavernpipe.c) |

## CavernPipe Protocol

### Handshake (8 bytes)
| Byte | Content |
|------|---------|
| 0 | BitDepth (32 for float32) |
| 1 | Mandatory frames (24 for E-AC-3/JOC) |
| 2-3 | Output channel count (UInt16 LE, e.g., 8 for 7.1) |
| 4-7 | UpdateRate (64 for E-AC-3, 1024 for streaming) |

### Streaming
- Send: 4-byte length (LE) + compressed audio data
- Receive: 4-byte length (LE) + rendered PCM float32

### Connection
- Windows: Named pipe `\\.\pipe\CavernPipe`
- Unix: Unix socket

## Known Issues

**Deadlock**: If the client doesn't send enough data for `mandatory_frames * update_rate * channels` samples, CavernPipe won't reply, causing the client to hang. Fix: send sufficient data in initial frames, or reconnect on seek.

**Error**: `read_cavern: failed to read response length` indicates the handshake failed or insufficient data was sent.

## Usage

```bash
# Play with custom decoder
mpv.exe --ad=cavernpipe "dolby-atmos.mkv"

# Or auto-detect (if configured)
mpv.exe "dolby-atmos.mkv"
```

## References

- Cavern website: https://cavern.sbence.hu
- MPV manual: https://mpv.io/manual/master/
