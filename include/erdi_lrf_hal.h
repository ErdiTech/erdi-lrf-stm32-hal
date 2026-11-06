/*
 * erdi_lrf_hal.h - STM32 HAL wrapper for ERDI laser rangefinder modules.
 *
 * Receives the module's UART stream with circular DMA (no per-byte interrupt, safe at 460800 baud),
 * feeds the bytes to the dependency-free parser in erdi_lrf_core.h and gives you the latest distance.
 *
 * Works on any STM32 family whose HAL provides HAL_UART_Receive_DMA() and __HAL_DMA_GET_COUNTER().
 * Define ERDI_HAL_INCLUDE to your family header, e.g. -DERDI_HAL_INCLUDE='"stm32g4xx_hal.h"'.
 *
 * Wiring: module TX -> MCU UART RX, module RX -> MCU UART TX, common GND, module VCC from its own supply.
 * Configure the UART as 8N1 at erdi_hal_baud(model): 460800 for LRF22VB/LRF50VB/H series, 115200 for the rest.
 * The DMA channel/stream for UART RX must be set to DMA_CIRCULAR, byte width, memory increment.
 */
#ifndef ERDI_LRF_HAL_H
#define ERDI_LRF_HAL_H

#ifndef ERDI_HAL_INCLUDE
#define ERDI_HAL_INCLUDE "stm32f4xx_hal.h"
#endif
#include ERDI_HAL_INCLUDE

#include "erdi_lrf_core.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifndef ERDI_HAL_DMA_BUF
#define ERDI_HAL_DMA_BUF 128u /* bytes; must cover the longest gap between two erdi_hal_poll() calls */
#endif

typedef struct {
  UART_HandleTypeDef *huart;
  uint8_t dma_buf[ERDI_HAL_DMA_BUF];
  uint16_t tail;               /* next byte to consume */
  erdi_parser_t parser;
  erdi_measurement_t last;     /* newest frame */
  uint32_t last_tick_ms;       /* HAL_GetTick() when it arrived */
  bool have_frame;
} erdi_hal_t;

/* Baud rate to configure on the UART for this model (0 if the model is unknown). */
uint32_t erdi_hal_baud(const char *model);

/* Start circular-DMA reception on an already initialised UART/DMA pair. */
HAL_StatusTypeDef erdi_hal_start(erdi_hal_t *h, UART_HandleTypeDef *huart, const char *model);

/* Consume bytes received since the last call. Returns true when at least one new valid frame arrived;
 * *out (optional) then holds the newest measurement. Call from the main loop at least every
 * ERDI_HAL_DMA_BUF / (baud / 10) seconds. */
bool erdi_hal_poll(erdi_hal_t *h, erdi_measurement_t *out);

/* Family A models only: send the single-shot request (55 AA 88 FF FF FF FF 84). */
HAL_StatusTypeDef erdi_hal_request(erdi_hal_t *h);

/* True if the newest frame is younger than max_age_ms. */
bool erdi_hal_fresh(const erdi_hal_t *h, uint32_t max_age_ms);

#ifdef __cplusplus
}
#endif
#endif /* ERDI_LRF_HAL_H */
