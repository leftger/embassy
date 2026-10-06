# embassy-stm32-venc

Embassy driver for the **STM32N6 Video Encoder (VENC)** — the Hantro H.264 and
JPEG encoder IP.

ST ships the encoder as a software stack rather than a prebuilt library, so this
crate pairs the [`stm32-bindings`](https://github.com/embassy-rs/stm32-bindings)
FFI with a Rust implementation of the Encoder Wrapper Layer (EWL) — the
OS/platform seam the stack normally fills with FreeRTOS or ThreadX code.

```text
embassy-stm32-venc
├── ffi        bindgen declarations for h264encapi.h / jpegencapi.h / ewl.h
├── platform   the 20 EWL entry points, over embassy-time + a static arena
├── coherency  Cortex-M55 D-cache maintenance for ASIC-visible buffers
├── h264       safe wrapper: Config / Encoder / Frame
└── jpeg       safe wrapper: Config / Encoder
```

## How it fits together

| Layer | Where |
|---|---|
| Hantro encoder C stack (BSD-3-Clause) | prebuilt `libvenc.a`, linked by `stm32-bindings` |
| Raw API bindings + archive | `stm32-bindings` (`venc` / `venc-m55` features) |
| EWL platform seam | this crate (`platform.rs`) — no RTOS, no ST HAL |
| Safe API | this crate (`h264.rs`, `jpeg.rs`) |

The EWL seam is small: `libvenc.a` leaves exactly twenty `EWL*` symbols
unresolved, and everything else resolves inside the archive. They are
implemented here with:

* **registers** — `embassy_stm32::pac::VENC.swreg(n)`
* **timing** — `embassy_time::Instant` (so a time driver must be enabled)
* **memory** — a 8-byte-aligned first-fit arena supplied by the caller
* **synchronisation** — a polling `EWLWaitHwRdy` that idles in `wfi`

## Usage

```rust,ignore
use embassy_stm32::peripherals::VENC;
use embassy_stm32_venc::{Venc, h264, jpeg};

// ST's default pool size; place it in ASIC-visible RAM.
static mut POOL: [u8; 0x190000] = [0; 0x190000];

let venc = Venc::new(p.VENC, unsafe { &mut POOL });

// H.264
let mut enc = venc.h264(h264::Config::new(800, 480, 30))?;
let mut out = [0u8; 800 * 480];

let n = enc.stream_start(&mut out)?;
let n = enc.encode(&h264::Frame::yuv420_planar(&y, &u, &v), h264::CodingType::Intra, 0, &mut out)?;
let n = enc.stream_end(&mut out)?;

// JPEG (same ASIC, same pool)
let mut jenc = venc.jpeg(jpeg::Config::new(800, 480))?;
let n = jenc.encode(&h264::Frame::yuv420_planar(&y, &u, &v), &mut out)?;
```

## Prerequisites

* **Time driver.** `EWLWaitHwRdy` uses `embassy_time` for its timeout, so a time
  driver must be configured (e.g. `embassy-stm32`'s `time-driver-any`).
* **Pool.** A large enough, long-lived buffer must be passed to `Venc::new`. ST's
  default is `0x190000` (1.6 MiB); the requirement depends on resolution and
  configuration. It must be reachable by the VENC's bus masters.
* **Caches.** The safe encoder methods clean the input and output buffers around
  each frame; buffers placed in cacheable memory rely on that. Buffers in
  non-cacheable memory need no action.
* **Blocking.** Encoding is synchronous: `EWLWaitHwRdy` blocks the calling
  context until the ASIC finishes. Run it on a dedicated executor or a blocking
  thread.
* **Isolation (RIF).** When TrustZone/RIF isolation is active, the VENC bus
  master must be granted access to the memories holding the pool and picture
  buffers (see `embassy-stm32`'s `rif` module).

## Building the bindings

This crate depends on the `venc` module of `stm32-bindings`, which is generated:

```sh
# in the stm32-bindings repo
STM32CUBEN6_DIR=/path/to/STM32CubeN6 ./d build-venc   # vendor headers + build libvenc.a
./d gen                                               # generate build/stm32-bindings
```

`build-venc` compiles `Middlewares/Third_Party/VideoEncoder` (H.264 + JPEG) for
Cortex-M55 with `arm-none-eabi-gcc`. `H264TestId.c` is replaced by a small port
shim so the library does not pull in `stdio`; consequently
`H264EncTestCropping` and `H264EncTestInputLineBuf` are no-ops.

## Limitations

* Single instance (an EWL/ASIC constraint — ST supports one encoder at a time).
* No interrupt-driven completion yet; `EWLWaitHwRdy` polls and idles with `wfi`.
* Not validated on hardware: the crate compiles and links, but the produced
  bitstreams have not been decode-tested.

## License

The Hantro encoder sources redistributed with `stm32-bindings` are BSD-3-Clause
(Verisilicon/Google); see `stm32-bindings-gen/venc/LICENSE.md`. This crate itself
is MIT OR Apache-2.0.
