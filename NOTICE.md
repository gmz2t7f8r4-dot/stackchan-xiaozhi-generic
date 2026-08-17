# Attribution and third-party notices

This repository is a derivative work and does not claim original authorship of
the complete codebase.

Primary upstream projects:

- `xiaozhi-esp32` — Shenzhen Xinzhi Future Technology Co., Ltd. and project
  contributors. Distributed under the upstream MIT License.
- `Stackchan-HtSz` — community modifications for the M5Stack CoreS3 StackChan.
- `stackchan-mcp` by kisaragi-mochi and contributors — MIT-licensed protocol,
  hardware and integration reference.
- `M5Stack/StackChan` — the official open-source StackChan firmware and mobile
  app. Its BLE service layout and dance payload format were used as a
  compatibility reference under its published license.
- `xiaozhi-esp32-server` by xinnan-tech and contributors — MIT-licensed server
  code used as the basis for the optional integration patches in `server/`.
- Espressif ESP-IDF and ESP Component Registry packages — distributed under
  their respective licenses.

The repository preserves upstream copyright headers. New project-specific
changes are offered under the MIT License in the root `LICENSE` file unless a
file states otherwise.

The files `SCS.cc`, `SCS.h`, `SCSCL.cc`, `SCSCL.h`, `SCSerial.cc`,
`SCSerial.h` and `INST.h` under `main/boards/m5stack-core-s3/` derive from
Feetech SCServo_lib and are licensed under GPL-3.0. The corresponding license
is included as `SCServo_lib_LICENSE.txt`. Because the current CoreS3 build
statically links this driver, a distributed complete firmware image is
effectively GPL-3.0 and its corresponding source must remain available.

Generated firmware may contain linked third-party components. Redistributors
are responsible for reviewing the license metadata of the exact dependency
versions resolved by ESP-IDF Component Manager.

Private character artwork, personal prompts, photos, credentials, server
addresses and device identifiers are intentionally excluded.
