# Agent instructions

## Git workflow

- Do not create branches. Make commits directly on `main`.
- Commit and push to `main`; do not open pull requests unless asked.

## Device safety

- Never switch a device: no `--on`, `--off` or `--toggle`, and no other action that powers a socket on or off. Use read-only actions only (`--status`, `--meter`, `--name`, `--*-get`).
- Do not put real device MAC addresses in tracked files. Use the placeholder `AA:BB:CC:DD:EE:FF`.
