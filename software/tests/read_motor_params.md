# Passive motor parameter snapshot

`read_motor_params` reads the configured motors through the existing USB-CAN serial interface. It sends only the vendor read-parameter request (`0x33`) and passive status request (`0xCC`). It does not enable, disable, zero, control, switch modes, write parameters, or save parameters. It never constructs `OpenEAIArm` or `MotorManager`.

The application uses the motor IDs, exact motor type strings, serial device, and baud rate from the selected YAML configuration. A master ID of `0` is accepted for factory/default setups; slave IDs must be unique and in `1..0x7FE`. A status response is accepted only when its CAN ID matches the configured master ID or the successfully read `MST_ID`, and its payload low nibble identifies the configured slave. The parameter response must echo the full slave ID, read opcode, and requested RID.

## Build

From the `software` directory, build the standalone target with the project CMake configuration:

```sh
cmake -S . -B build
cmake --build build --target read_motor_params
```

The target uses `yaml-cpp`, `Threads`, `config_loader.cpp`, `dm.cpp`, and `serial_port.cpp`. It does not link the arm controller, dynamics, KDL, or URDF libraries. The top-level CMake project finds the arm's KDL, URDF, and Python dependencies during configuration, so a direct standalone build can be useful on a host without those packages:

```sh
g++ -std=c++17 -O2 -pthread -Iinclude \
  tests/read_motor_params.cpp src/config_loader.cpp src/dm.cpp src/serial_port.cpp \
  -lyaml-cpp -o read_motor_params
```

The hardware host still needs the configured serial device and permissions; this workspace's Windows environment is not the target runtime.

## Run

Run from the `software` directory so the default config path resolves:

```sh
./build/read_motor_params configs/default_right.yml
```

Options:

```text
--config FILE       Select a configuration file (also accepts it positionally).
--output DIR        Create the snapshot in a new, non-existing directory.
--timeout-ms N      Per-request receive timeout; default 100 ms.
--retries N         Attempts per register/status request; default 2.
--legacy-registers  Also query legacy RIDs 50..53; omitted by default.
--list-registers    Print the offline RID table and exit without opening the serial port.
```

For example:

```sh
./build/read_motor_params --config configs/default_right.yml --timeout-ms 100 --retries 2
```

By default, output goes to a unique `software/motor_param_snapshots/snapshot_YYYYMMDDTHHMMSS` directory. An explicit `--output` path must not already exist. The tool copies the selected config before opening the serial port. Do not run another controller, vendor tool, or serial reader against the same CAN adapter during collection; responses could otherwise be consumed by the wrong process.

## Output

- `config.yml` is the exact input config copy.
- `manifest.txt` records the collection time, configured port and baud rate, request table, timeout/retry settings, and the no-write behavior.
- `parameters.csv` contains one row for each attempted RID and motor. It retains the response bytes and raw little-endian 32-bit word; values are decoded only where the current parameter typing is known. Timeouts are reported as timeouts and do not establish that a register is unsupported.
- `status.csv` contains the passive status frame, host receive monotonic timestamp, raw position/velocity/torque fields, error nibble, both trailing status bytes, and decoded values only when that motor's `PMAX`, `VMAX`, and `TMAX` were successfully read. The timestamp is when the host matched the response, not a motor-side timestamp. No SDK fallback limits are substituted.

The application queries one configured motor at a time. The protocol has no transaction sequence number; parameter replies are matched by slave ID and RID, while status replies use the master CAN ID and status payload's low slave-ID nibble. A status reply cannot be confirmed for slave IDs above `0x0F`; those motors' parameter replies can still be collected by full slave ID, while status remains a timeout. Exit code `0` means all motors returned at least one parameter, valid `PMAX`/`VMAX`/`TMAX` readbacks, and a matching status reply. Exit code `2` means a partial snapshot was written with one or more of those required observations missing. Exit code `1` reports a config, output, or setup error. Timeouts and missing rows remain in the CSV output for offline review.

The default RID table is `0..37`, `54..56`, `59..65`, `70..73`, and `80..83`. Legacy RIDs `50..53` are opt-in because firmware support and meaning can vary. In the current manual, RID 37 is `Boot_ver`; RIDs 54–56 are `x_off`, `dir`, and `m_off`; RIDs 59–65 are `IBase`, `VBus`, `TMOS`, `T_CEL`, and phase-current offsets; RIDs 70–73 are the ISEN/OSEN application and boot versions; RIDs 80–81 are `p_m` and `xout`; and RIDs 82–83 are `Xin_raw_d` and `Xout_raw_d`. The current manual types RIDs 54–56 and 59–65 as floats, while RIDs 82–83 are unsigned integers. Older SDK tables map RID 54 to `m_off`, so firmware revision matters. Raw bytes remain the archival source when firmware interpretation is uncertain. Run `--list-registers` to see the names and interpretation notes without opening the serial port.

Status frames have no query transaction ID. A matching asynchronous or buffered status frame can be observed during the receive window, so this snapshot must not be used to measure command latency. If a motor was already enabled before collection, the tool leaves it in that state and does not disable it. The status query itself does not request a state transition.

## Vendor references

- [Damiao motor SDK, USB-to-CAN examples](https://github.com/dmBots/motor-sdk/tree/main/Python%E4%BE%8B%E7%A8%8B/u2can): documents the `0x33` read-parameter flow and lists RID names/types used by the SDK.
- [Damiao motor debugging tool](https://github.com/dmBots/motor-debugging-tool): official debugging software releases and documentation; use it to inspect which values the installed firmware exposes when the controller is available.

Vendor SDK comments and manuals can differ across firmware generations. The snapshot therefore stores raw response bytes instead of treating every RID label as universal.
