# Agent instructions

## Git workflow

- Do not create branches. Make commits directly on `main`.
- Commit and push to `main`; do not open pull requests unless asked.

## Device safety

- Never switch a device: no `--off` or `--toggle`, and no other action that powers a socket off. `--on` is allowed only on KINGLAMP (`B3:00:00:00:73:C0`) when the user asks for it.
- Read-only actions (`--status`, `--meter`, `--name`, `--*-get`) may run against any listed device.
- Debugging (`-v`) and every write (`--set-*`, `--timer-set`, `--set-schedule`, `--time-set`, `--on`) run against KINGLAMP only.
- Do not put real device MAC addresses in tracked files. Use the placeholder `AA:BB:CC:DD:EE:FF`.
