# ERDI laser rangefinder driver for STM32 (HAL, circular DMA)

A small **STM32 HAL driver for ERDI 905 nm laser rangefinder modules**. It receives the module's UART stream with **circular DMA**, so there is no per-byte interrupt even at **460800 baud**, and parses frames with a dependency-free C99 core. Supports **LRF22VB, LRF50VB, LRF50H, LRF100H, LRF200H, LRF300H, LRF600H, LRF1500H** (`5C` stream) and **LR1000E2 ... LR2000E4, LRF1200A1, LRF3000A1, SPD1200N0/N2/N4** (`55 AA` polled). Use it as a **laser distance sensor driver for STM32F4 / G4 / L4 / H7** in drones, robots, AGVs, level and position measurement.

- `include/erdi_lrf_core.h`: parser + model table, no HAL, host-testable
- `include/erdi_lrf_hal.h`, `src/erdi_lrf_hal.c`: HAL wrapper (`erdi_hal_start`, `erdi_hal_poll`, `erdi_hal_request`, `erdi_hal_fresh`)
- `examples/nucleo_f411re`: complete PlatformIO example (USART6 + DMA2, output on the ST-LINK VCP)

## Use

```c
#include "erdi_lrf_hal.h"

static erdi_hal_t lrf;
/* 1. configure the UART as 8N1 at erdi_hal_baud("LRF22VB") and its RX DMA as circular, byte-wide */
erdi_hal_start(&lrf, &huart6, "LRF22VB");

for (;;) {
  erdi_measurement_t m;
  if (erdi_hal_poll(&lrf, &m) && !m.out_of_range) {
    use_distance(m.metres);          /* metres, unit handled per model */
  }
}
```

Polled models (family A): call `erdi_hal_request(&lrf)` at your rate, then `erdi_hal_poll()`.

Other STM32 families: build with `-DERDI_HAL_INCLUDE='"stm32g4xx_hal.h"'`. Poll at least every `ERDI_HAL_DMA_BUF / (baud / 10)` seconds (128 bytes at 460800 baud is about 2.8 ms of data: raise `ERDI_HAL_DMA_BUF` or poll from a timer if your loop is slower).

## Wiring

| Module | STM32 |
|---|---|
| TX | UART RX (PA12 / USART6 on the example) |
| RX | UART TX (PA11) |
| GND | GND |
| VCC | own supply per the model manual |

Check the module's UART logic level against your MCU (3.3 V) before connecting; supply voltage is not UART voltage. Model pages: <https://erdilrf.com/models/?utm_source=github&utm_medium=readme&utm_campaign=erdi-lrf-stm32-hal>.

## Frame formats

| Family | Frame | Check |
|---|---|---|
| B (stream, 460800) | `5C lo hi chk` (LRF1500H `5C lo mid hi chk`) | `~(sum of distance bytes)`; H series centimetres, VB series millimetres |
| A (polled, 115200) | `55 AA 88 FF FF FF FF 84` request; `55 AA 88 STA FF DH DL SUM` reply | reply sum includes the header; value x 10 metres |

Protocol source and manual quotations: [ErdiTech/erdi-lrf-drivers](https://github.com/ErdiTech/erdi-lrf-drivers).

## Build and test

```bash
gcc -std=c99 -Wall -Wextra -o t test/test_core.c -lm && ./t       # parser tests on the host
cd examples/nucleo_f411re && pio run                                # firmware build (PlatformIO)
```

The firmware example builds in CI; it has not been run on a bench with a live module by the maintainers yet, so check baud accuracy on your clock tree (the example runs from the 16 MHz HSI with 8x oversampling).

## Related projects

- [erdi-lrf-esp32-micropython](https://github.com/ErdiTech/erdi-lrf-esp32-micropython): ESP32 / RP2040 MicroPython driver
- [erdi-lrf-home-assistant](https://github.com/ErdiTech/erdi-lrf-home-assistant): ESPHome component
- [laser-rangefinder-module-database](https://github.com/ErdiTech/laser-rangefinder-module-database): dataset of laser rangefinder modules

## Support / 打赏

If this project saved you time, you can buy the author a coffee in crypto. Thank you!
如果这个项目帮到了你，欢迎用加密货币打赏支持，谢谢！

| Network | Asset | Address |
|---|---|---|
| TRON (TRC20) | USDT | `TNGdTwE6opH6rmUyoG1JUR7FSyEb4k273R` |
| Ethereum / BNB Chain / Polygon | USDT | `0x59c43144a862b351fa1086ae842d1327c9d91613` |
| Ethereum / Arbitrum / Base | ETH | `0x59c43144a862b351fa1086ae842d1327c9d91613` |
| Bitcoin | BTC | `1RS6tWXHregLfEsiPzVUCUBuhgDg5uTSV` |
| Solana | USDT | `HZguEXfTtLChNdJTxAYCbcGscaqikz8JYrEpUhEyeVd9` |

Please send only the listed asset on the listed network — other tokens or networks may be lost.
请按表中对应的网络转对应的币种，转错币种或网络可能无法找回。

Built by tommyapi.com

## License

MIT — see [LICENSE](LICENSE).
