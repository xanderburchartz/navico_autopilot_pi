# Navico Autopilot plugin for OpenCPN

Controls a Navico autopilot computer (B&G and Simrad NAC-1, NAC-2, NAC-3,
AC12, AC42) directly over NMEA 2000:

- Standby, Auto (heading hold), Wind, Nav, No Drift;
- course changes of -10, -1, +1 and +10 degrees (in Wind mode they move the
  wind angle);
- shows the pilot mode, the heading and the target heading or wind angle.

Status: **alpha**. Tested so far:

| Platform | OpenCPN | Tested against |
|---|---|---|
| macOS (arm64; x86_64 under Rosetta) | 5.14.2 | B&G NAC-3 on a boat, and a simulated gateway |
| Linux flatpak, Raspberry Pi 5 (aarch64) | 5.14.0 | B&G NAC-3 on a boat |
| Windows (32-bit build) | 5.14.2 | simulated gateway |
| Android (arm64, armhf) | | builds, not tested |

On the NAC-3, Standby, Auto, Wind, No Drift and the course changes were
followed. Nav was sent but the pilot stayed in Auto (no active route at the
time); not yet confirmed. AC12/AC42 and NAC-1/2 have not been tried. Reports
are welcome.

## How it works

The plugin uses only OpenCPN's own NMEA 2000 API (`GetActiveDrivers`,
`RegisterTXPGNs`, `WriteCommDriverN2K` and N2K listeners). It does not need
any CAN hardware of its own: any NMEA 2000 connection through which OpenCPN
can transmit works, for example a Yacht Devices or Actisense gateway on the
network, an Actisense NGT-1, or SocketCAN on Linux.

**Requires OpenCPN 5.14 or later** for network gateways (Yacht Devices RAW,
Actisense RAW ASCII). In OpenCPN 5.12 the network N2K driver numbers its
fast-packet frames wrongly, so every second command is lost: a receiver
following the standard drops it. Fixed in 5.14.

### Safety

- The plugin sends **nothing on its own**: no heartbeat, no requests, no
  repeats. Each button press sends one PGN 130850.
- STBY can be pressed whenever a pilot is selected, even without recent data.
- If more than one autopilot is found, the plugin does not pick one; choose it
  in the settings.
- Every command is written to the OpenCPN log with its bytes.

## Protocol

| Sent | PGN 130850, priority 2, destination 255 |
|---|---|
| Standby | `41 9F <pilot> FF FF 0A 06 00 FF FF FF` |
| Auto | `41 9F <pilot> FF FF 0A 09 00 FF FF FF` |
| Wind | `41 9F <pilot> FF FF 0A 0F 00 FF FF FF` |
| Nav | `41 9F <pilot> FF FF 0A 0A 00 FF FF FF` |
| No Drift | `41 9F <pilot> FF FF 0A 0C 00 FF FF FF` |
| -1 / +1 | `41 9F <pilot> FF FF 0A 1A 00 02/03 AF 00 FF` |
| -10 / +10 | `41 9F <pilot> FF FF 0A 1A 00 02/03 D1 06 FF` |

`41 9F` is manufacturer 1857 (Simrad), industry marine. `<pilot>` is the bus
address of the autopilot. Byte 7 must be `00`; a NAC-3 ignores the command
when it is `FF` (canboat calls this byte "spare"). Direction 2 is port,
3 starboard; the angle is in 0.0001 rad.

Received: 65305 (pilot mode, report `0A`; status, report `02`), 127237
(heading to steer), 65341 (wind angle to hold), 60928 (address claims, to
follow the pilot when its address changes). The actual heading comes from
OpenCPN itself.

**Which device is the pilot.** A NAC-3 claims several addresses; only the
part with function 150 (autopilot) follows 130850. The plugin takes the source
of 65305 "Mode" reports from an AC or NAC (model byte `00` or `64`). A B&G
keypad also sends 65305, but as requests (report `03` and `0B`); those are
ignored. Settings: automatic (only if exactly one pilot is found) or a fixed
address.

Sources: canboat (`docs/canboat.json`), recordings of a B&G Triton2 pilot
keypad and a NAC-3 on a real boat, and the notes in the `autopilot` branch of
TwoCan (no code taken from it).

## Building

Like other catalog plugins built from the shipdriver_pi templates, see
[INSTALL.md](INSTALL.md). On macOS:

```
git submodule update --init opencpn-libs
mkdir build-osx && cd build-osx
cmake -DCMAKE_BUILD_TYPE=Release "-DOCPN_TARGET_TUPLE=darwin-wx32;10;universal" ..
make tarball
```

Without a wxWidgets 3.2 installation the template tries to build wxWidgets
3.2.4 into `cache/`. With Xcode 26 that fails (its bundled libpng includes
`<fp.h>`, which no longer exists); building wxWidgets 3.2.12 with
`./configure ... --prefix=<repo>/cache` and a one-line `fp.h` that includes
`<math.h>` on the include path works.

Unit tests (no wxWidgets needed):

```
c++ -std=c++17 -Isrc test/test_simnet.cpp src/simnet.cpp src/pilot_state.cpp -o test_simnet && ./test_simnet
```

## Continuous integration

`.github/workflows/build.yml` builds every catalog target on GitHub Actions:
flatpak (x86_64, aarch64), Debian bookworm/trixie/forky (x86_64, arm64),
macOS universal, Android (arm64, armhf) and Windows (Win32, x64). Each run
keeps the tarballs as workflow artifacts. Upload to Cloudsmith and the catalog
metadata push only happen when the `CLOUDSMITH_API_KEY` secret is set.

## Testing without a boat

`test/` describes testing against a simulated Yacht Devices gateway with a
NAC-3 (a small Node.js program). Never try a new version on a boat first.

## How this plugin was made

The code was written with an AI coding assistant (Claude, by Anthropic), under
the direction of the maintainer, who owns the boat it was tested on; commits
say so in a `Co-Authored-By` line. The protocol bytes do not come from the
assistant but from canboat and from recordings of a real bus (B&G Triton2
pilot keypad and NAC-3), and every command was checked against a simulated
gateway before it was tried on the boat, with the maintainer at the helm.

## License

GPL-3.0-or-later, see [COPYING](COPYING). The build system comes from the
shipdriver_pi templates (GPL-3.0-or-later).
