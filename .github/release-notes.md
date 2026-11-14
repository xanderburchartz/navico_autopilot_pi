Test build of the Navico Autopilot plugin. Requires **OpenCPN 5.14 or later**.

## New in 0.1.1

Fixed:
- OpenCPN could crash on quit when a window of the plugin (settings, tack,
  NAV confirmation) was open or had been open. The windows are now closed
  before the plugin is unloaded.
- The catalog metadata (.xml) was invalid XML because of the "&" in "B&G".
  This release has metadata files for each tarball.
- Translations come from the plugin's own catalog only. Before, a common word
  such as "Cancel" or "True" could take OpenCPN's translation, and the
  plugin's Dutch catalog could change texts elsewhere in OpenCPN. All texts
  of the settings window are now in the translation template
  (`po/navico_autopilot_pi.pot`).

Panel:
- Display in the style of a Navico pilot: mode, heading, set heading or
  wind angle, rudder angle, boat speed (STW or SOG, your choice) and the
  pilot's alarms.
- NAV asks for a confirmation and then confirms on the pilot the way a
  Triton2 keypad does (without it the pilot does not take Nav).
- Tack or gybe in Wind mode; non-follow-up steering with the course buttons
  in standby (the rudder moves while the button is held).
- Name of the waypoint the pilot steers to: from the plotter if it sends one
  (129285), else from OpenCPN's active route, with a short note on arrival
  and at the end of the route.
- Follows OpenCPN's day, dusk and night colours.
- Pilot alarms also appear as OpenCPN notifications (the bell on the chart).
  An alarm that comes and goes keeps one notification; it is taken back 30 s
  after the pilot stopped reporting it. Not on Android (see below).

Pilot settings (gear icon top right of the panel):
- Reads the settings of the pilot computer and lets you change them, each
  change after a confirmation: wind mode, tack time and angle, manual speed,
  and per steering profile ("Low speed" and "High speed", as on a Triton2)
  turn rate, rudder gain, counter rudder, autotrim, init rudder, rudder
  limit, off heading limit, track response, approach angle and course change
  confirm angle.
- Installation settings (boat length and speeds, rudder deadband, minimum
  and maximum power, minimum rudder) stay hidden until you press
  "Installation" three times within 3 s. Boat type, drive voltage, drive
  engage and rudder calibration are only shown, not changed. The control
  method is not in the plugin yet.
- Before the first change, the plugin stores a snapshot of all settings.
  "Snapshots..." lists them and can put an earlier snapshot back; it shows
  what will change and asks first.
- Measured on a B&G NAC-3. Other pilots may answer differently; please
  report what you see.

Other:
- Built against plugin API 1.20 (OpenCPN 5.12 and later load it; network
  gateways need 5.14, see the README). The Android builds stay on API 1.18.
- No product information requests over network gateways (OpenCPN keeps the
  answers to itself there).

## Which file

| Your OpenCPN | File ending in |
|---|---|
| Windows (the normal 32-bit OpenCPN) | `msvc-wx32-10-x86.tar.gz` |
| Windows, 64-bit OpenCPN built with wxWidgets 3.3 | `msvc-64-10-x86_64.tar.gz` (not tested) |
| macOS (Apple silicon and Intel) | `darwin-wx32-10-arm64-x86_64.tar.gz` |
| Linux flatpak, PC | `flatpak-x86_64-25.08-x86_64.tar.gz` |
| Linux flatpak, Raspberry Pi 64-bit | `flatpak-aarch64-25.08-aarch64.tar.gz` |
| Debian 12 / 13 / 14 package | `debian-<arch>-12/13/14-<arch>.tar.gz` |
| Android | `android-arm64-16-arm64.tar.gz` or `android-armhf-16-armhf.tar.gz` (not tested) |

## Install

1. Download the file for your system. Do not unpack it.
2. In OpenCPN: Options, Plugins, **Import plugin...**, and pick the file.
3. Enable the plugin, open its preferences and choose the NMEA 2000
   connection. The connection must be able to transmit.

On Windows, importing the file for the other bitness gives "incompatible
plugin" and also removes a version that was already installed.

On Android, OpenCPN only shows Import plugin when `CatalogExpert=1` is set
in `opencpn.conf`, which the Android app does not let you change yet. The
Android builds will be installable from the plugin catalog later.

The plugin sends one command per button press (NFU repeats while the button
is held), asks devices for their name once per connection, and reads the
pilot settings only when the settings window opens. A setting changes only
after you confirm it.

## Reporting

Please open an issue with the "Test report" form, also when everything
worked. The plugin writes each command with its bytes to `opencpn.log`
(lines starting with `navico_autopilot_pi:`); those lines help most.
