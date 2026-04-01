# mpv-cavern

**MPV + CavernPipe** — Real-time Dolby Atmos playback through spatial audio rendering.

This is a modified build of [mpv](https://mpv.io/) (v0.41.0) with an integrated **CavernPipe audio decoder** that sends compressed audio (E-AC-3 / AC-3) to [CavernPipeServer](https://cavern.sbence.hu/) for Dolby Atmos spatial rendering, and receives rendered PCM back for playback.

TrueHD streams are decoded locally via FFmpeg's built-in decoder.

## Signal Flow

```
Media File (E-AC-3 / AC-3 with Dolby Atmos)
    ↓
mpv (ad_cavernpipe decoder)
    ↓ [compressed frames via Windows named pipe]
CavernPipeServer (Atmos spatial rendering)
    ↓ [PCM float32]
Audio Output (WASAPI / Snapserver / etc.)
```

For TrueHD:
```
Media File (TrueHD)
    ↓
mpv (ad_cavernpipe / FFmpeg TrueHD decoder)
    ↓ [PCM]
Audio Output
```

## Supported Codecs

| Codec | Path | Atmos Support |
|-------|------|---------------|
| E-AC-3 (Dolby Digital Plus) | CavernPipeServer | ✅ Full JOC/Atmos spatial rendering |
| AC-3 (Dolby Digital) | CavernPipeServer | ✅ Spatial rendering |
| TrueHD | FFmpeg (local decode) | ❌ Channel bed only (7.1) |

## Quick Start

### Prerequisites

- Windows 10/11
- [.NET 8.0 Runtime](https://dotnet.microsoft.com/download/dotnet/8.0) (for CavernPipeServer)

### 1. Start CavernPipeServer

```cmd
CavernPipeServer\CavernPipeServer.exe
```

The server creates a Windows named pipe (`\\.\pipe\CavernPipe`) and waits for connections.

### 2. Play a file with Atmos

```cmd
build\mpv.exe --ad=cavernpipe "your-atmos-file.mkv"
```

Or with verbose logging:
```cmd
build\mpv.exe --ad=cavernpipe --no-video -v "your-atmos-file.mkv"
```

### 3. Play TrueHD (no server needed)

```cmd
build\mpv.exe --ad=cavernpipe "your-truehd-file.mkv"
```

## Channel Configuration

The output channel count is read from `%APPDATA%\Cavern\Save.dat` (first line). This is set by the Cavern Driver configuration tool. Default is 6 channels (5.1).

To change manually:
```cmd
echo 8 > "%APPDATA%\Cavern\Save.dat"
```

Supported: up to 64 channels. For >8 channels, the output uses an "unknown" channel layout to bypass mpv's remixing.

## Building from Source

Requires MSYS2 with clang64 toolchain and FFmpeg development libraries.

```bash
# In MSYS2 clang64 shell
meson setup build
meson compile -C build
```

### Modified Files

| File | Purpose |
|------|---------|
| `audio/decode/ad_cavernpipe.c` | CavernPipe audio decoder (E-AC-3/AC-3 via pipe, TrueHD via FFmpeg) |
| `filters/f_decoder_wrapper.c` | Decoder selection logic (cavernpipe integration) |
| `filters/f_decoder_wrapper.h` | Decoder wrapper header |
| `meson.build` | Build system (includes ad_cavernpipe.c) |

## CavernPipe Protocol

The named pipe protocol is synchronous:

1. **Handshake** (8 bytes): `[BitDepth=32][MandatoryFrames][Channels U16 LE][UpdateRate I32 LE]`
2. **Send**: `[U32 length LE][compressed audio bytes]`
3. **Receive**: `[U32 length LE][float32 PCM bytes]`
4. Repeat 2-3.

## Known Limitations

- **E-AC-3 with AHT** (Adaptive Hybrid Transform): Not supported by Cavern's decoder — these files will fail.
- **TrueHD Atmos objects**: FFmpeg decodes the channel bed only (up to 7.1); Atmos spatial metadata is not preserved. Full TrueHD Atmos requires a native MLP decoder in Cavern (not yet implemented).
- **WASAPI shared mode**: Limited to the device's configured channel count (typically 2 or 8).
- **One client at a time**: CavernPipeServer accepts a single pipe connection.

## Credits & References

- **mpv** — Free, open-source media player: [mpv.io](https://mpv.io/) | [GitHub](https://github.com/mpv-player/mpv)
- **Cavern** — Open-source spatial audio engine by [VoidX (sbence)](https://cavern.sbence.hu/): [GitHub](https://github.com/VoidXH/Cavern)
- **CavernPipeServer** — Part of the Cavern project, provides real-time Dolby Atmos rendering via named pipe

## License

mpv is licensed under GPLv2+ (see [LICENSE.GPL](LICENSE.GPL)) and LGPLv2.1+ (see [LICENSE.LGPL](LICENSE.LGPL)).

The CavernPipe decoder (`ad_cavernpipe.c`) follows the same license as mpv.

CavernPipeServer and Cavern libraries are provided under their own license — see the [Cavern repository](https://github.com/VoidXH/Cavern) for details.
