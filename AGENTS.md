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
