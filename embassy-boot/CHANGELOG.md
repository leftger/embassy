# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

<!-- next-header -->
## Unreleased - ReleaseDate

- Fix blocking `verify_and_mark_updated` to use a 64-byte hash chunk (matching async), so verification works on flashes with `READ_SIZE > 2`.
- Fix partition size assert so a DFU partition smaller than ACTIVE cannot pass via `u32` wraparound in release builds.
- Fix `hash` to avoid reading past the DFU partition on the final partial chunk.
- Align blocking `FirmwareState` buffer/`READ_SIZE` handling with the async updater.
- Erase the entire DFU partition on the first `write_firmware` of an update so a shorter image cannot leave a stale tail.

## 0.7.0 - 2026-03-10

- Fixed documentation and assertion of STATE partition size requirements
- Added documentation for package features
- Made `read_state` on `BootLoader` public
- Update embassy-embedded-hal to 0.6.0
- Update embassy-sync to 0.8.0

## 0.6.1 - 2025-08-26

- First release with changelog.
