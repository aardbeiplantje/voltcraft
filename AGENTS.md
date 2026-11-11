# Agent instructions

## Git workflow

- Do not create branches. Make commits directly on `main`.
- Commit and push to `main`; do not open pull requests unless asked.

## Device safety

- Two lamps may run any command, including writes and `--off`: KINGLAMP (`B3:00:00:00:73:C0`) and the test lamp (a less bright lamp, address kept out of this file).
- KINGLAMP must never be powered on: no `--on` on KINGLAMP, even when asked. It is too bright.
- Read-only actions (`--status`, `--meter`, `--name`, `--*-get`) may run against any listed device.
- Debugging (`-v`) and writes (`--set-*`, `--timer-set`, `--set-schedule`, `--time-set`) run against the two lamps only.
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
