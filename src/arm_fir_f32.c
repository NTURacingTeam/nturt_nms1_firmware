/**
 * @file arm_fir_f32.c
 * @brief CMSIS-DSP arm_fir_f32 implementation in ANSI C.
 * 
 * Reference design from: [ESP-DSP](https://github.com/espressif/esp-dsp).
 *
 */

// glibc include
#include <string.h>

// arm math include
#include <arm_math.h>

// zephyr include
#include <zephyr/kernel.h>

#ifndef __arm__

void arm_fir_init_f32(arm_fir_instance_f32 *S, uint16_t numTaps,
                      const float32_t *pCoeffs, float32_t *pState,
                      uint32_t blockSize) {
  S->numTaps = numTaps;
  S->stateIndex = 0;
  S->pCoeffs = pCoeffs;

  memset(pState, 0, sizeof(float32_t) * blockSize);
  
  S->pState = pState;
}

void arm_fir_f32(arm_fir_instance_f32 *S, const float32_t *pSrc,
                 float32_t *pDst, uint32_t blockSize) {
  for (int i = 0; i < blockSize; i++) {
    float acc = 0;
    int coeff_pos = 0;
    S->pState[S->stateIndex] = pSrc[i];
    S->stateIndex++;
    if (S->stateIndex >= S->numTaps) {
      S->stateIndex = 0;
    }
    for (int n = S->stateIndex; n < S->numTaps; n++) {
      acc += S->pCoeffs[coeff_pos++] * S->pState[n];
    }
    for (int n = 0; n < S->stateIndex; n++) {
      acc += S->pCoeffs[coeff_pos++] * S->pState[n];
    }
    pDst[i] = acc;
  }
}

#endif  // __arm__
