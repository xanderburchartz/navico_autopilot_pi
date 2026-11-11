# Testing

## Unit tests

`test_simnet.cpp` checks the bytes of every command and the parsing of the
received messages against frames recorded on a real bus (B&G Triton2 keypad,
NAC-3, commands through a Yacht Devices gateway that the NAC-3 followed).

```
c++ -std=c++17 -Wall -Isrc test/test_simnet.cpp src/simnet.cpp src/pilot_state.cpp -o build/test_simnet
build/test_simnet
```

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
