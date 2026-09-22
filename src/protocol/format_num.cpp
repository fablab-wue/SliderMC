#include "protocol_internal.h"

#include <stdint.h>
#include <string.h>

void protocol_format_num(char *dst, size_t n, float v) {
  if (n == 0) {
    return;
  }

  char tmp[32];
  tmp[0] = '0';
  tmp[1] = 0;

  if (v != v) {
    tmp[0] = '-';
    tmp[1] = 0;
  } else if (v != 0.0f) {
    const int neg = v < 0.0f;
    float av = neg ? -v : v;
    if (av <= 1e18f) {
      int decimals = 2;
      if (av < 1.0f) {
        if (av >= 1e-1f) {
          decimals = 3;
        } else if (av >= 1e-2f) {
          decimals = 4;
        } else if (av >= 1e-3f) {
          decimals = 5;
        } else if (av >= 1e-4f) {
          decimals = 6;
        } else if (av >= 1e-5f) {
          decimals = 7;
        } else {
          decimals = 8;
        }
      }

      uint64_t scaled;
      if (av >= 1e12f) {
        scaled = (uint64_t)(av + 0.5f);
        decimals = 0;
      } else {
        static const float kPow10[9] = {1.f, 10.f, 100.f, 1000.f, 10000.f,
                                        1e5f, 1e6f, 1e7f, 1e8f};
        scaled = (uint64_t)(av * kPow10[decimals] + 0.5f);
      }

      while (decimals > 0 && (scaled % 10ull) == 0ull) {
        scaled /= 10ull;
        --decimals;
      }

      char digits[24];
      int nd = 0;
      uint64_t t = scaled;
      do {
        digits[nd++] = (char)('0' + (unsigned)(t % 10ull));
        t /= 10ull;
      } while (t != 0ull);
      while (nd <= decimals) {
        digits[nd++] = '0';
      }

      size_t o = 0;
      if (neg) {
        tmp[o++] = '-';
      }
      for (int i = nd - 1; i >= decimals; --i) {
        tmp[o++] = digits[i];
      }
      if (decimals > 0) {
        tmp[o++] = '.';
        for (int i = decimals - 1; i >= 0; --i) {
          tmp[o++] = digits[i];
        }
      }
      tmp[o] = 0;
    }
  }

  size_t len = strlen(tmp);
  if (len >= n) {
    len = n - 1;
  }
  memcpy(dst, tmp, len);
  dst[len] = 0;
}
