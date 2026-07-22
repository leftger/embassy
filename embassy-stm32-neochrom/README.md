# embassy-stm32-neochrom

Embassy integration for ST NeoChrom (GPU2D) using the [NemaGFX](https://github.com/STMicroelectronics/x-cube-image-processing/tree/main/Middleware/NemaGFX) middleware via [`stm32-bindings`](https://github.com/embassy-rs/stm32-bindings).

## Status

Early bring-up. Provides a minimal driver over the generated NemaGFX FFI and the platform HAL from `nema-gfx-hal`.

## Local development

Generate `stm32-bindings` first:

```bash
cd ../stm32-bindings
cargo run --release --bin stm32-bindings-gen -- --module nema_gfx
```

Then build this crate (example chip feature):

```bash
cd ../embassy/embassy-stm32-neochrom
cargo check --features stm32n657x0,stub-gpu2d
```

## Features

| Feature | Description |
|---------|-------------|
| `stm32n657x0` | STM32N657 + Cortex-M55 NemaGFX library |
| `stub-gpu2d` | Link GPU2D HAL stub instead of STM32Cube (default) |
| `neochrom-m55` | NemaGFX bindings + M55 prebuilt library only |

Disable `default-features` and `stub-gpu2d` when wiring a real STM32Cube GPU2D HAL on hardware.
