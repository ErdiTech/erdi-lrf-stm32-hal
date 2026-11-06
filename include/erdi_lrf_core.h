/*
 * erdi_lrf_core.h - dependency-free C99 frame parser for ERDI laser rangefinder modules.
 *
 * Family B ("5C" stream, 460800 baud):
 *     5C lo hi chk            (4-byte; all B models except LRF1500H)
 *     5C lo mid hi chk        (5-byte; LRF1500H)
 *     chk = ~(sum of distance bytes), distance little-endian.
 *     H series count centimetres, VB series count millimetres.
 * Family A ("55 AA" polled, 115200 baud):
 *     request 55 AA 88 FF FF FF FF 84
 *     reply   55 AA 88 STA FF DH DL SUM   (SUM covers the header; value x 10 metres)
 *
 * Feed bytes one at a time; no allocation, no I/O, safe in an ISR or a main loop.
 */
#ifndef ERDI_LRF_CORE_H
#define ERDI_LRF_CORE_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { ERDI_FAMILY_A = 0, ERDI_FAMILY_B = 1 } erdi_family_t;

typedef struct {
  const char *name;
  erdi_family_t family;
  uint32_t baud;
  float unit_m;        /* metres per raw count */
  uint8_t frame_len;   /* bytes per frame */
  uint32_t oor_raw;    /* raw value meaning "out of range" (0 = none) */
} erdi_model_t;

static const erdi_model_t ERDI_MODELS[] = {
  {"LR1000E2", ERDI_FAMILY_A, 115200, 0.1f, 8, 0},
  {"LR1000E4", ERDI_FAMILY_A, 115200, 0.1f, 8, 0},
  {"LR1200E2", ERDI_FAMILY_A, 115200, 0.1f, 8, 0},
  {"LR1200E4", ERDI_FAMILY_A, 115200, 0.1f, 8, 0},
  {"LR1500E2", ERDI_FAMILY_A, 115200, 0.1f, 8, 0},
  {"LR1500E4", ERDI_FAMILY_A, 115200, 0.1f, 8, 0},
  {"LR2000E2", ERDI_FAMILY_A, 115200, 0.1f, 8, 0},
  {"LR2000E4", ERDI_FAMILY_A, 115200, 0.1f, 8, 0},
  {"LRF1200A1", ERDI_FAMILY_A, 115200, 0.1f, 8, 0},
  {"LRF3000A1", ERDI_FAMILY_A, 115200, 0.1f, 8, 0},
  {"SPD1200N0", ERDI_FAMILY_A, 115200, 0.1f, 8, 0},
  {"SPD1200N2", ERDI_FAMILY_A, 115200, 0.1f, 8, 0},
  {"SPD1200N4", ERDI_FAMILY_A, 115200, 0.1f, 8, 0},
  {"LRF50H", ERDI_FAMILY_B, 460800, 0.01f, 4, 65535u},
  {"LRF100H", ERDI_FAMILY_B, 460800, 0.01f, 4, 65535u},
  {"LRF200H", ERDI_FAMILY_B, 460800, 0.01f, 4, 65535u},
  {"LRF300H", ERDI_FAMILY_B, 460800, 0.01f, 4, 65535u},
  {"LRF600H", ERDI_FAMILY_B, 460800, 0.01f, 4, 65535u},
  {"LRF1500H", ERDI_FAMILY_B, 460800, 0.01f, 5, 16777215u},
  {"LRF22VB", ERDI_FAMILY_B, 460800, 0.001f, 4, 20000u},
  {"LRF50VB", ERDI_FAMILY_B, 460800, 0.001f, 4, 50000u},
};
#define ERDI_MODEL_COUNT ((int)(sizeof(ERDI_MODELS) / sizeof(ERDI_MODELS[0])))

/* Look up a model by name (case-sensitive, upper case). NULL if unknown. */
static inline const erdi_model_t *erdi_find_model(const char *name) {
  for (int i = 0; i < ERDI_MODEL_COUNT; i++) {
    if (strcmp(ERDI_MODELS[i].name, name) == 0) return &ERDI_MODELS[i];
  }
  return NULL;
}

typedef struct {
  const erdi_model_t *model;
  uint8_t buf[8];
  uint8_t n;
  uint32_t good;      /* valid frames */
  uint32_t bad;       /* bytes dropped while resyncing */
} erdi_parser_t;

typedef struct {
  float metres;       /* valid only when !out_of_range */
  uint32_t raw;
  bool out_of_range;  /* B: out-of-range marker; A: STA != 1 (no valid echo) */
} erdi_measurement_t;

static inline void erdi_parser_init(erdi_parser_t *p, const erdi_model_t *m) {
  memset(p, 0, sizeof(*p));
  p->model = m;
}

static inline uint8_t erdi_checksum_b(const uint8_t *d, uint8_t n) {
  uint8_t s = 0;
  for (uint8_t i = 0; i < n; i++) s = (uint8_t)(s + d[i]);
  return (uint8_t)~s;
}

static inline uint8_t erdi_sum8(const uint8_t *d, uint8_t n) {
  uint8_t s = 0;
  for (uint8_t i = 0; i < n; i++) s = (uint8_t)(s + d[i]);
  return s;
}

/* Build the family A single-shot request into out[8]. */
static inline void erdi_single_shot_request(uint8_t out[8]) {
  static const uint8_t r[8] = {0x55, 0xAA, 0x88, 0xFF, 0xFF, 0xFF, 0xFF, 0x84};
  memcpy(out, r, 8);
}

static inline void erdi__drop_one(erdi_parser_t *p) {
  /* drop the first byte, then skip ahead to the next possible header */
  uint8_t head = (p->model->family == ERDI_FAMILY_B) ? 0x5C : 0x55;
  uint8_t i = 1;
  while (i < p->n && p->buf[i] != head) i++;
  p->bad += i;
  memmove(p->buf, p->buf + i, (size_t)(p->n - i));
  p->n = (uint8_t)(p->n - i);
}

/* Push one received byte. Returns true and fills *out when a valid frame completed. */
static inline bool erdi_parser_feed(erdi_parser_t *p, uint8_t byte, erdi_measurement_t *out) {
  const erdi_model_t *m = p->model;
  uint8_t head = (m->family == ERDI_FAMILY_B) ? 0x5C : 0x55;
  if (p->n == 0 && byte != head) {
    p->bad++;
    return false;
  }
  p->buf[p->n++] = byte;
  if (m->family == ERDI_FAMILY_A && p->n == 2 && p->buf[1] != 0xAA) {
    erdi__drop_one(p);
    return false;
  }
  if (p->n < m->frame_len) return false;

  bool ok;
  if (m->family == ERDI_FAMILY_B) {
    ok = erdi_checksum_b(p->buf + 1, (uint8_t)(m->frame_len - 2)) == p->buf[m->frame_len - 1];
  } else {
    ok = erdi_sum8(p->buf, 7) == p->buf[7];
  }
  if (!ok) {
    erdi__drop_one(p);
    return false;
  }
  if (m->family == ERDI_FAMILY_B) {
    uint32_t raw = 0;
    for (int k = m->frame_len - 2; k >= 1; k--) raw = (raw << 8) | p->buf[k];
    out->raw = raw;
    out->out_of_range = (m->oor_raw != 0 && raw >= m->oor_raw);
    out->metres = out->out_of_range ? 0.0f : (float)raw * m->unit_m;
  } else {
    uint32_t raw = ((uint32_t)p->buf[5] << 8) | p->buf[6];
    out->raw = raw;
    out->out_of_range = (p->buf[3] != 0x01);
    out->metres = out->out_of_range ? 0.0f : (float)raw * m->unit_m;
  }
  p->n = 0;
  p->good++;
  return true;
}

#ifdef __cplusplus
}
#endif
#endif /* ERDI_LRF_CORE_H */
