/**
 * @file arm_math.h
 * @brief CMSIS-DSP implementation in ANSI C.
 * 
 */

// glibc include
#include <stdint.h>

#ifndef __arm__
#ifndef ARM_MATH_H
#define ARM_MATH_H

typedef float float32_t;

/**
 * @brief Instance structure for the floating-point FIR filter.
 * 
 * @note @p stateIndex is added to make a circular buffer.
 * 
 */
typedef struct {
  uint16_t numTaps;
  uint16_t stateIndex;
  float32_t *pState;
  const float32_t *pCoeffs;
} arm_fir_instance_f32;

/**
 * @brief  Initialization function for the floating-point FIR filter.
 * @param[in,out] S          points to an instance of the floating-point FIR
 * filter structure.
 * @param[in]     numTaps    Number of filter coefficients in the filter.
 * @param[in]     pCoeffs    points to the filter coefficients.
 * @param[in]     pState     points to the state buffer.
 * @param[in]     blockSize  number of samples that are processed at a time.
 */
void arm_fir_init_f32(arm_fir_instance_f32 *S, uint16_t numTaps,
                      const float32_t *pCoeffs, float32_t *pState,
                      uint32_t blockSize);

/**
 * @brief Processing function for the floating-point FIR filter.
 * @param[in]  S          points to an instance of the floating-point FIR
 * structure.
 * @param[in]  pSrc       points to the block of input data.
 * @param[out] pDst       points to the block of output data.
 * @param[in]  blockSize  number of samples to process.
 */
void arm_fir_f32(arm_fir_instance_f32 *S, const float32_t *pSrc,
                 float32_t *pDst, uint32_t blockSize);

#endif  // ARM_MATH_H
#endif  // __arm__
