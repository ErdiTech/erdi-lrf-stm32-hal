#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../include/erdi_lrf_core.h"

static int feed_all(erdi_parser_t *p, const uint8_t *d, int n, erdi_measurement_t *last) {
  int frames = 0;
  for (int i = 0; i < n; i++) {
    if (erdi_parser_feed(p, d[i], last)) frames++;
  }
  return frames;
}

int main(void) {
  erdi_parser_t p;
  erdi_measurement_t m;

  /* manual vector: 5C 02 11 EC = 43.54 m on H series, 4.354 m on VB series */
  const uint8_t f[] = {0x5C, 0x02, 0x11, 0xEC};
  erdi_parser_init(&p, erdi_find_model("LRF100H"));
  assert(feed_all(&p, f, 4, &m) == 1);
  assert(fabsf(m.metres - 43.54f) < 1e-3f);
  erdi_parser_init(&p, erdi_find_model("LRF22VB"));
  assert(feed_all(&p, f, 4, &m) == 1);
  assert(fabsf(m.metres - 4.354f) < 1e-4f);

  /* out-of-range marker 20000 mm on LRF22VB */
  const uint8_t oor[] = {0x5C, 0x20, 0x4E, 0x91};
  assert(erdi_checksum_b(oor + 1, 2) == 0x91);
  erdi_parser_init(&p, erdi_find_model("LRF22VB"));
  assert(feed_all(&p, oor, 4, &m) == 1 && m.out_of_range);

  /* garbage and a truncated frame before good frames: resync */
  const uint8_t noisy[] = {0x00, 0x5C, 0x01, 0x5C, 0x02, 0x11, 0xEC, 0x5C, 0x02, 0x11, 0xEC};
  erdi_parser_init(&p, erdi_find_model("LRF100H"));
  assert(feed_all(&p, noisy, (int)sizeof(noisy), &m) == 2);

  /* bad checksum rejected */
  const uint8_t bad[] = {0x5C, 0x02, 0x11, 0xEB};
  erdi_parser_init(&p, erdi_find_model("LRF100H"));
  assert(feed_all(&p, bad, 4, &m) == 0);

  /* LRF1500H five-byte frame */
  uint8_t f5[5] = {0x5C, 0x02, 0x11, 0x03, 0};
  f5[4] = erdi_checksum_b(f5 + 1, 3);
  erdi_parser_init(&p, erdi_find_model("LRF1500H"));
  assert(feed_all(&p, f5, 5, &m) == 1);
  assert(m.raw == 0x031102u);

  /* family A: request bytes and reply scaling */
  uint8_t req[8];
  erdi_single_shot_request(req);
  assert(req[7] == (uint8_t)(0x88 + 4 * 0xFF));
  const uint8_t a[] = {0x55, 0xAA, 0x88, 0x01, 0xFF, 0x00, 0x2A, 0xB1};
  erdi_parser_init(&p, erdi_find_model("LRF3000A1"));
  assert(feed_all(&p, a, 8, &m) == 1);
  assert(fabsf(m.metres - 4.2f) < 1e-4f);
  const uint8_t noecho[] = {0x55, 0xAA, 0x88, 0x00, 0xFF, 0x00, 0x00, 0x86};
  erdi_parser_init(&p, erdi_find_model("LRF3000A1"));
  assert(feed_all(&p, noecho, 8, &m) == 1 && m.out_of_range);

  assert(erdi_find_model("NOPE") == NULL);
  puts("erdi_lrf_core: all assertions passed");
  return 0;
}
