# Agent instructions

## Git workflow

- Do not create branches. Make commits directly on `main`.
- Commit and push to `main`; do not open pull requests unless asked.

## Device safety

- Never switch a device: no `--off` or `--toggle`, and no other action that powers a socket off. `--on` is allowed only on KINGLAMP (`B3:00:00:00:73:C0`) when the user asks for it.
- Exception: the test lamp (a less bright lamp, address kept out of this file) may be switched on and off with `--on` and `--off` when the user asks for an on/off test. Use it for that test only.
- Read-only actions (`--status`, `--meter`, `--name`, `--*-get`) may run against any listed device.
- Debugging (`-v`) and every write (`--set-*`, `--timer-set`, `--set-schedule`, `--time-set`, `--on`) run against KINGLAMP only, except `--on` and `--off` on the test lamp as above.
- Do not put real device MAC addresses in tracked files. Use the placeholder `AA:BB:CC:DD:EE:FF`.

## BLE sniffer test (nRF sniffer dongle)

- Tool: `nrfutil ble-sniffer` 0.22.0, dongle firmware 5.0.0. Check the firmware with `sniff`: the log in `~/.nrfutil/logs/nrfutil-ble-sniffer.log` should say `Firmware version 5.0.0`.
- Port: the dongle exposes two serial ports. Use the second one, `/dev/ttyACM1`. `/dev/ttyACM0` times out on writes.
- Start the sniffer in the background before the test and kill it by PID afterwards. Never kill by name: `pkill -f` matches the shell itself.
  - `nrfutil ble-sniffer sniff --port /dev/ttyACM1 --scan-follow-rsp --scan-follow-aux --output-pcap-file <pcap> --follow "<ADDR> public" &`
- Run the matching `voltcraft.pl` commands with `--log <file>` at the same time, then compare: the sniffer's timestamps run about 0.9 s later than the log's.
- Inspect the capture with `tshark -r <pcap> -Y btatt`.
- Captures and logs go in the scratchpad, never in the repository.
- If the dongle stops answering: put it into DFU mode (see the hidden button on the board), flash the bundled firmware with `nrfutil device program --traits nordicDfu --firmware <zip>`, then replug it.
