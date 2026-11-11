Test build of the Navico Autopilot plugin. Requires **OpenCPN 5.14 or later**.

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

Try it against a simulator or in the harbour first, with the pilot's own
keypad within reach. The plugin sends one command per button press and
nothing else.

## Reporting

Please open an issue with the "Test report" form, also when everything
worked. The plugin writes each command with its bytes to `opencpn.log`
(lines starting with `navico_autopilot_pi:`); those lines help most.
