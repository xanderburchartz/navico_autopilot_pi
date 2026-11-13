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

## Installing a test build

Until the plugin is in the OpenCPN catalog, test builds are on the
[Releases](../../releases) page. Download the tarball for your system (the
release notes say which), then in OpenCPN choose Options, Plugins,
**Import plugin...** and pick the file without unpacking it. Enable the
plugin and choose the NMEA 2000 connection in its preferences.

Please report results, good or bad, with the "Test report" issue form.
Reports for Nav with an active route, AC12/AC42, NAC-1/2 and Android are
especially useful.

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

- The plugin sends **no heartbeat**. Each button press sends one PGN 130850;
  confirming NAV sends two (see below). The only repeated command is
  non-follow-up: while a course button is held in standby, the direction is
  repeated every 0.5 s, as the NAC-3 stops by itself about 2.5 s after the
  last one; on release, when the panel closes or OpenCPN loses focus, it sends
  stop three times, 0.1 s apart.
- Requests: once per connection (and when you open the settings) one ISO
  request for address claims to all devices (one frame back from each). On
  serial and SocketCAN connections also a request for product information to
  each autopilot and rudder sensor that answers, not to the whole bus. Not on
  network gateways: OpenCPN's network driver keeps 126996 to itself, so the
  answer would not reach the plugin.
- STBY can be pressed whenever a pilot is selected, even without recent data.
- Autopilot and rudder sensor are chosen automatically (the most likely one if
  there are several) and can be overruled in the settings.
- Every command and request is written to the OpenCPN log with its bytes.

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
| Confirm (NAV) | `41 9F <pilot> FF FF 0A 1C 00 FF FF FF`, then Nav |
| Tack / gybe (WIND) | `41 9F <pilot> FF FF 0A 11 00 00 FF FF FF` |
| NFU port / stbd / stop | `41 9F <pilot> FF FF 02 0D 00 04/05/FF 00 00 00` |

`41 9F` is manufacturer 1857 (Simrad), industry marine. `<pilot>` is the bus
address of the autopilot. Byte 7 must be `00`; a NAC-3 ignores the command
when it is `FF` (canboat calls this byte "spare"). Direction 2 is port,
3 starboard; the angle is in 0.0001 rad.

**NAV needs a confirmation.** After Nav the pilot reports its mode with bit 7
set (`88 00` from standby, `90 00` from auto) and waits 10 s. A Triton2 keypad
confirms with event `0x1C` directly followed by Nav; the pilot then reports
`40 00` (Nav). Without valid navigation data on the bus (129283/129284 all "no
value") the pilot echoes Nav but stays where it is and raises "Navigation data
missing". The plugin shows its own confirmation window for this (measured on a
NAC-3, 13 Nov 2026).

Received: 65305 (pilot mode, report `0A`; status, report `02`), 130851 (a
literal echo of each command the pilot executed, 25-55 ms later), 130850 with
byte 4 = `01` and 130856 (pilot alarms and their text), 127237 (heading to
steer), 65341 (set wind angle), 127245 (rudder angle, bytes 4-5, positive =
starboard), 130306 (apparent wind), 129284 (bearing and distance to the
waypoint), 128259 (speed through the water, if chosen instead of SOG),
60928 and 126996 (NAME and model ID, to recognise devices). The
actual heading and speed come from OpenCPN itself.

**Pilot settings.** The NAC-3 keeps its settings itself and answers reads
over the Simnet key value channel, PGN 130845: `41 9F <device> 00 FF FF
<group> <param lo hi> <op> <value, 4 bytes LE>` with op 0 read, 1 set, 2
reply. It answers reads from a Yacht Devices gateway too (measured 13 Nov
2026). The rudder calibration (09/01, 09/02) lives on the rudder feedback
node, not on the pilot. A set uses instance `FF` and pads a 2-byte value
with `FF`, as a Triton2 does; the pilot replies with the value it took.
The plugin reads the known settings when the settings window opens, one
request at a time (the next after the answer or 1.5 s, an unanswered one is
asked once more), and changes a setting only after a confirmation. Boat
type, drive voltage and rudder calibration are read only.

**Steering profiles.** Byte 3 of 130845 is an instance. A NAC-3 set to
Sail keeps group 1A (rudder gain 1A/0D, autotrim 0E, counter rudder 0F,
turn rate 10) per steering profile, instances 0 to 5; instance FF answers
with the active profile and from 6 on the answer is FF. The names come in
130846 under 18/8008 per instance: ENGINE, REACH, RUN, BEAT, WORK4, WORK5
(measured 13 Nov 2026). The "Low" profile in group 19 that other sources
name does not answer on this NAC-3. After a change the NAC-3 sends the
values of all instances, so the plugin keeps replies per instance.

**Which device is the pilot.** A NAC-3 claims several addresses; only the
part with function 150 (autopilot) follows 130850. The plugin takes the source
of 65305 "Mode" reports from an AC or NAC (model byte `00` or `64`). A B&G
keypad also sends 65305, but as requests (report `03` and `0B`); those are
ignored. With more than one candidate the plugin prefers one with manufacturer
Navico in its NAME (an emulated pilot claims another make), then a NAC.

**Rudder sensor.** Several devices may send 127245: a NAC-3 sends both a real
and a "virtual" rudder feedback with the same make and function, and a pilot
also sends 127245 without a position. The plugin prefers a rudder feedback
unit (function 155) of the pilot's make whose model ID does not say
"virtual"; without model IDs (network gateways, see above) the lowest address.
The choice can be overruled in the settings and is stored by NAME.

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
metadata push only happen when the `CLOUDSMITH_API_KEY` secret is set. A tag
(for example `v0.1.0-alpha`) also creates a GitHub release with all tarballs;
tags with a hyphen become a pre-release.

## Testing without a boat

`test/` describes testing against a simulated Yacht Devices gateway with a
NAC-3 (a small Node.js program).

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
