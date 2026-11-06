#include "erdi_lrf_hal.h"

uint32_t erdi_hal_baud(const char *model) {
  const erdi_model_t *m = erdi_find_model(model);
  return m ? m->baud : 0u;
}

HAL_StatusTypeDef erdi_hal_start(erdi_hal_t *h, UART_HandleTypeDef *huart, const char *model) {
  const erdi_model_t *m = erdi_find_model(model);
  if (h == NULL || huart == NULL || m == NULL) return HAL_ERROR;
  memset(h, 0, sizeof(*h));
  h->huart = huart;
  erdi_parser_init(&h->parser, m);
  return HAL_UART_Receive_DMA(huart, h->dma_buf, (uint16_t)ERDI_HAL_DMA_BUF);
}

bool erdi_hal_poll(erdi_hal_t *h, erdi_measurement_t *out) {
  /* DMA writes at head = BUF - remaining; we consume from tail up to head, wrapping once. */
  uint16_t head = (uint16_t)(ERDI_HAL_DMA_BUF - __HAL_DMA_GET_COUNTER(h->huart->hdmarx));
  bool got = false;
  while (h->tail != head) {
    erdi_measurement_t m;
    if (erdi_parser_feed(&h->parser, h->dma_buf[h->tail], &m)) {
      h->last = m;
      h->have_frame = true;
      h->last_tick_ms = HAL_GetTick();
      got = true;
    }
    h->tail = (uint16_t)((h->tail + 1u) % ERDI_HAL_DMA_BUF);
  }
  if (got && out != NULL) *out = h->last;
  return got;
}

HAL_StatusTypeDef erdi_hal_request(erdi_hal_t *h) {
  if (h->parser.model->family != ERDI_FAMILY_A) return HAL_ERROR;
  uint8_t req[8];
  erdi_single_shot_request(req);
  return HAL_UART_Transmit(h->huart, req, sizeof(req), 20);
}

bool erdi_hal_fresh(const erdi_hal_t *h, uint32_t max_age_ms) {
  return h->have_frame && (HAL_GetTick() - h->last_tick_ms) <= max_age_ms;
}
