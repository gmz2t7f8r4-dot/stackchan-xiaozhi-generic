# Upstream and provenance

This repository is a generic, privacy-cleaned derivative assembled from the
following open-source projects:

- [Stackchan-HtSz](https://github.com/mo-hantang/Stackchan-HtSz), based on
  commit `0aa7d9c95f4c429ce9836d2530879acd757f7728`.
- [xiaozhi-esp32](https://github.com/78/xiaozhi-esp32), the firmware and board
  framework used by the project.
- [xiaozhi-esp32-server](https://github.com/xinnan-tech/xiaozhi-esp32-server),
  the basis for the optional server integration patches.
- [stackchan-mcp](https://github.com/kisaragi-mochi/stackchan-mcp), referenced
  for StackChan MCP integration and behavior.

The repository also contains an SCServo-compatible driver derived from
Feetech's `SCServo_Linux` / `SCServo_lib`. Those driver files are licensed
under GPL-3.0; see `main/boards/m5stack-core-s3/SCServo_lib_LICENSE.txt` and
`NOTICE.md`.

Project-specific names, credentials, private endpoints, Wi-Fi settings and
personal artwork are intentionally not included. Public additions in this
repository include the generic camera/MCP flow, BLE dance-page integration,
navigation stability fixes, configuration examples and documentation.
