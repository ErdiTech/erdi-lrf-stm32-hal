/*
 * Nucleo-F411RE example: ERDI LRF22VB on USART6 (PA11 = TX, PA12 = RX), 460800 baud, circular DMA.
 * Distance is printed on USART2 (ST-LINK virtual COM port, 115200 baud). Change MODEL for your module.
 * Clock: default HSI 16 MHz (no PLL), which keeps the example short.
 */
#include <stdio.h>
#include <string.h>

#include "erdi_lrf_hal.h"
#include "stm32f4xx_hal.h"

#define MODEL "LRF22VB"

static UART_HandleTypeDef huart2;
static UART_HandleTypeDef huart6;
static DMA_HandleTypeDef hdma_usart6_rx;
static erdi_hal_t lrf;

void SysTick_Handler(void) { HAL_IncTick(); }

static void gpio_uart_init(void) {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef g = {0};
  g.Mode = GPIO_MODE_AF_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Pin = GPIO_PIN_2 | GPIO_PIN_3;  /* USART2 TX/RX (ST-LINK VCP) */
  g.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &g);
  g.Pin = GPIO_PIN_11 | GPIO_PIN_12;  /* USART6 TX/RX (rangefinder) */
  g.Alternate = GPIO_AF8_USART6;
  HAL_GPIO_Init(GPIOA, &g);
}

static void uart_init(void) {
  __HAL_RCC_USART2_CLK_ENABLE();
  __HAL_RCC_USART6_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&huart2);

  huart6.Instance = USART6;
  huart6.Init.BaudRate = erdi_hal_baud(MODEL);
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_8;  /* better baud accuracy at 460800 from a 16 MHz clock */

  hdma_usart6_rx.Instance = DMA2_Stream1;
  hdma_usart6_rx.Init.Channel = DMA_CHANNEL_5;
  hdma_usart6_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
  hdma_usart6_rx.Init.PeriphInc = DMA_PINC_DISABLE;
  hdma_usart6_rx.Init.MemInc = DMA_MINC_ENABLE;
  hdma_usart6_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_usart6_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_usart6_rx.Init.Mode = DMA_CIRCULAR;
  hdma_usart6_rx.Init.Priority = DMA_PRIORITY_HIGH;
  hdma_usart6_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
  HAL_DMA_Init(&hdma_usart6_rx);
  __HAL_LINKDMA(&huart6, hdmarx, hdma_usart6_rx);
  HAL_UART_Init(&huart6);
}

int main(void) {
  HAL_Init();
  gpio_uart_init();
  uart_init();
  if (erdi_hal_start(&lrf, &huart6, MODEL) != HAL_OK) {
    for (;;) {}
  }
  erdi_measurement_t m;
  uint32_t last_print = 0;
  for (;;) {
    erdi_hal_poll(&lrf, &m);
    if (HAL_GetTick() - last_print >= 200 && erdi_hal_fresh(&lrf, 500)) {
      last_print = HAL_GetTick();
      char line[48];
      int n;
      if (lrf.last.out_of_range) {
        n = snprintf(line, sizeof(line), "out of range\r\n");
      } else {
        int mm = (int)(lrf.last.metres * 1000.0f + 0.5f);
        n = snprintf(line, sizeof(line), "%d.%03d m\r\n", mm / 1000, mm % 1000);
      }
      HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)n, 50);
    }
  }
}
