# Testing

## Unit tests

`test_simnet.cpp` checks the bytes of every command and the parsing of the
received messages against frames recorded on a real bus (B&G Triton2 keypad,
NAC-3, commands through a Yacht Devices gateway that the NAC-3 followed).

```
c++ -std=c++17 -Wall -Isrc test/test_simnet.cpp src/simnet.cpp src/pilot_state.cpp src/pilot_settings.cpp src/route_info.cpp -o build/test_simnet
build/test_simnet
c++ -std=c++17 -Wall -Isrc test/test_wrap0183.cpp src/wrap0183.cpp src/simnet.cpp -o build/test_wrap0183
build/test_wrap0183
c++ -std=c++17 -Wall -Isrc test/test_sk_bridge.cpp src/sk_bridge.cpp src/simnet.cpp src/pilot_state.cpp src/pilot_settings.cpp -o build/test_sk_bridge
build/test_sk_bridge
```

`test_wrap0183.cpp` checks the $PCDIN and $MXPGN sentences (checksums, the
MiniPlex example frames, fast packets split and put back together).
`test_sk_bridge.cpp` feeds what a Signal K server reports through the same
`PilotState` as the bus messages.

## Against a simulated gateway

Any program that speaks the Yacht Devices RAW protocol over TCP and acts like
a NAC-3 will do. The one used during development (`yd-gateway-sim.js`, not
part of this repository) sends 65305, 127237 and 65341 for a pilot at
address 4 and logs every 130850 it receives, for example
`commando 0x9 -> heading`.

1. Start the simulator on port 1457.
2. Start OpenCPN with a separate home and configuration directory, so your
   own settings are not touched. On macOS:

   ```
   HOME=$PWD/build/ocpn-home /Applications/OpenCPN.app/Contents/MacOS/OpenCPN -c $PWD/build/ocpn-home/config
   ```

   The plugin then goes into
   `build/ocpn-home/Library/Application Support/OpenCPN/Contents/PlugIns`.
3. Add a network connection: NMEA 2000, TCP, 127.0.0.1, port 1457,
   input and output.
4. Enable the plugin and choose the connection in its settings. Press STBY,
   AUTO, +1, +10, -1, -10, WIND, STBY and compare with the simulator log.

## NMEA 0183 with wrapped NMEA 2000

`test/sim/wrap0183_sim.js` (optional, needs Node.js; not part of the build)
is a simulated gateway with a NAC-3 at address 4 that speaks `$PCDIN` or
`$MXPGN` over TCP and logs every command:

```
node test/sim/wrap0183_sim.js mxpgn 10112
```

In OpenCPN add a network connection: NMEA 0183, TCP, 127.0.0.1, port 10112,
input and output. In the plugin settings choose that connection with
"$MXPGN (MiniPlex)" (or start the simulator with `pcdin` and choose
"$PCDIN (SeaSmart)").

## Signal K

Any Signal K server 2.x that reads the bus works; for tests a local one with
a simulated gateway as its NMEA 2000 source (provider type
`ydwg02-canboatjs`, TCP) and the server plugins `signalk-simnet-pilot-state`
and/or `signalk-autopilot` (type simrad). In the plugin settings enter the
server as `address:port`, choose "Signal K server" as the connection, and,
if the server has security, press "Request access" and approve the request
in the server's admin page.

`test/sk_client_check.cpp` runs the plugin's HTTP client on its own:

```
c++ -std=c++17 -Isrc $(wx-config --cxxflags base) test/sk_client_check.cpp src/signalk_client.cpp $(wx-config --libs base) -o build/sk_client_check
build/sk_client_check 127.0.0.1 3000 GET /signalk/v1/api/vessels/self/steering
```
