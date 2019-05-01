/* Copyright 2015 The MathWorks, Inc. */

/****************************************************
*                                                   *   
* wrapper fuctions for CMSIS  functions             *
*                                                   *  
****************************************************/

#ifndef MW_CMSIS_H
#define MW_CMSIS_H

#include "arm_math.h"
#include "rtwtypes.h"

#define mw_arm_abs_f32(pSrc, pDst, blockSize) arm_abs_f32((float32_t *)pSrc, (float32_t *)pDst, blockSize)
#define mw_arm_abs_q7(pSrc, pDst, blockSize) arm_abs_q7((q7_t *)pSrc, (q7_t *)pDst, blockSize) 
#define mw_arm_abs_q15(pSrc, pDst, blockSize) arm_abs_q15((q15_t *)pSrc, (q15_t *)pDst, blockSize) 
#define mw_arm_abs_q31(pSrc, pDst, blockSize) arm_abs_q31((q31_t *)pSrc, (q31_t *)pDst, blockSize) 

#define mw_arm_sqrt_q15(in, pOut) arm_sqrt_q15((q15_t)in,(q15_t *)pOut)
#define mw_arm_sqrt_q31(in, pOut) arm_sqrt_q31((q31_t)in,(q31_t *)pOut)
#define mw_arm_sqrt_f32(in, pOut) arm_sqrt_f32((float32_t)in,(float32_t *)pOut)

#define mw_arm_float_to_q31(pSrc, pDst, blockSize) arm_float_to_q31((float32_t *)pSrc, (q31_t *)pDst, blockSize)
#define mw_arm_float_to_q15(pSrc, pDst, blockSize) arm_float_to_q15((float32_t *)pSrc, (q15_t *)pDst, blockSize)
#define mw_arm_float_to_q7(pSrc, pDst, blockSize) arm_float_to_q7((float32_t *)pSrc, (q7_t *)pDst, blockSize)

#define mw_arm_q15_to_float(pSrc, pDst, blockSize) arm_q15_to_float((q15_t *)pSrc, (float32_t *)pDst, blockSize)
#define mw_arm_q15_to_q31(pSrc, pDst, blockSize) arm_q15_to_q31((q15_t *)pSrc, (q31_t *)pDst, blockSize)
#define mw_arm_q15_to_q7(pSrc, pDst, blockSize) arm_q15_to_q7((q15_t *)pSrc, (q7_t *)pDst, blockSize)

#define mw_arm_q31_to_float(pSrc, pDst, blockSize) arm_q31_to_float((q31_t *)pSrc, (float32_t *)pDst, blockSize)
#define mw_arm_q31_to_q15(pSrc, pDst, blockSize) arm_q31_to_q15((q31_t *)pSrc, (q15_t *)pDst, blockSize)
#define mw_arm_q31_to_q7(pSrc, pDst, blockSize) arm_q31_to_q7((q31_t *)pSrc, (q7_t *)pDst, blockSize)

#define mw_arm_q7_to_float(pSrc, pDst, blockSize) arm_q7_to_float((q7_t *)pSrc, (float32_t *)pDst, blockSize)
#define mw_arm_q7_to_q31(pSrc, pDst, blockSize) arm_q7_to_q31((q7_t *)pSrc, (q31_t *)pDst, blockSize)
#define mw_arm_q7_to_q15(pSrc, pDst, blockSize) arm_q7_to_q15((q7_t *)pSrc, (q15_t *)pDst, blockSize)

#define mw_arm_add_f32(pSrcA, pSrcB, pDst, blockSize) arm_add_f32((float32_t *)pSrcA, (float32_t *)pSrcB, (float32_t *)pDst, blockSize)
#define mw_arm_add_q31(pSrcA, pSrcB, pDst, blockSize) arm_add_q31((q31_t *)pSrcA, (q31_t *)pSrcB, (q31_t *)pDst, blockSize)
#define mw_arm_add_q15(pSrcA, pSrcB, pDst, blockSize) arm_add_q15((q15_t *)pSrcA, (q15_t *)pSrcB, (q15_t *)pDst, blockSize)
#define mw_arm_add_q7(pSrcA, pSrcB, pDst, blockSize)  arm_add_q7((q7_t *)pSrcA, (q7_t *)pSrcB, (q7_t *)pDst, blockSize)

#define mw_arm_sub_f32(pSrcA, pSrcB, pDst, blockSize) arm_sub_f32((float32_t *)pSrcA, (float32_t *)pSrcB, (float32_t *)pDst, blockSize)
#define mw_arm_sub_q31(pSrcA, pSrcB, pDst, blockSize) arm_sub_q31((q31_t *)pSrcA, (q31_t *)pSrcB, (q31_t *)pDst, blockSize)
#define mw_arm_sub_q15(pSrcA, pSrcB, pDst, blockSize) arm_sub_q15((q15_t *)pSrcA, (q15_t *)pSrcB, (q15_t *)pDst, blockSize)
#define mw_arm_sub_q7(pSrcA, pSrcB, pDst, blockSize)  arm_sub_q7((q7_t *)pSrcA, (q7_t *)pSrcB, (q7_t *)pDst, blockSize)

#define mw_arm_mult_f32(pSrcA, pSrcB, pDst, blockSize) arm_mult_f32((float32_t *)pSrcA, (float32_t *)pSrcB, (float32_t *)pDst, blockSize)
#define mw_arm_mult_q31(pSrcA, pSrcB, pDst, blockSize) arm_mult_q31((q31_t *)pSrcA, (q31_t *)pSrcB, (q31_t *)pDst, blockSize)
#define mw_arm_mult_q15(pSrcA, pSrcB, pDst, blockSize) arm_mult_q15((q15_t *)pSrcA, (q15_t *)pSrcB, (q15_t *)pDst, blockSize)
#define mw_arm_mult_q7(pSrcA, pSrcB, pDst, blockSize)  arm_mult_q7((q7_t *)pSrcA, (q7_t *)pSrcB, (q7_t *)pDst, blockSize)

#define mw_arm_cmplx_conj_f32(pSrc, pDst, numSamples) arm_cmplx_conj_f32((float32_t *)pSrc, (float32_t *)pDst, numSamples)
#define mw_arm_cmplx_conj_q31(pSrc, pDst, numSamples) arm_cmplx_conj_q31((q31_t *)pSrc, (q31_t *)pDst, numSamples)
#define mw_arm_cmplx_conj_q15(pSrc, pDst, numSamples) arm_cmplx_conj_q15((q15_t *)pSrc, (q15_t *)pDst, numSamples)

#define mw_arm_cmplx_mult_cmplx_f32(pSrcA, pSrcB, pDst, blockSize) arm_cmplx_mult_cmplx_f32((float32_t *)pSrcA, (float32_t *)pSrcB, (float32_t *)pDst, blockSize)
#define mw_arm_cmplx_mult_cmplx_q31(pSrcA, pSrcB, pDst, blockSize) arm_cmplx_mult_cmplx_q31((q31_t *)pSrcA, (q31_t *)pSrcB, (q31_t *)pDst, blockSize)
#define mw_arm_cmplx_mult_cmplx_q15(pSrcA, pSrcB, pDst, blockSize) arm_cmplx_mult_cmplx_q15((q15_t *)pSrcA, (q15_t *)pSrcB, (q15_t *)pDst, blockSize)

#define mw_arm_cmplx_mult_real_f32(pSrcA, pSrcB, pDst, blockSize) arm_cmplx_mult_real_f32((float32_t *)pSrcA, (float32_t *)pSrcB, (float32_t *)pDst, blockSize)
#define mw_arm_cmplx_mult_real_q31(pSrcA, pSrcB, pDst, blockSize) arm_cmplx_mult_real_q31((q31_t *)pSrcA, (q31_t *)pSrcB, (q31_t *)pDst, blockSize)
#define mw_arm_cmplx_mult_real_q15(pSrcA, pSrcB, pDst, blockSize) arm_cmplx_mult_real_q15((q15_t *)pSrcA, (q15_t *)pSrcB, (q15_t *)pDst, blockSize)

#define mw_arm_rshift_q15(pSrc, shiftBits, pDst, blockSize) arm_shift_q15 ((q15_t *)pSrc, -(shiftBits),(q15_t *)pDst, blockSize)
#define mw_arm_rshift_q31(pSrc, shiftBits, pDst, blockSize) arm_shift_q31 ((q31_t *)pSrc, -(shiftBits), (q31_t *)pDst, blockSize)
#define mw_arm_rshift_q7(pSrc, shiftBits, pDst, blockSize) arm_shift_q7 ((q7_t *)pSrc,  -(shiftBits), (q7_t *)pDst, blockSize)

#define mw_arm_shift_q15(pSrc, shiftBits, pDst, blockSize) arm_shift_q15 ((q15_t *)pSrc, shiftBits,(q15_t *)pDst, blockSize)
#define mw_arm_shift_q31(pSrc, shiftBits, pDst, blockSize) arm_shift_q31 ((q31_t *)pSrc, shiftBits, (q31_t *)pDst, blockSize)
#define mw_arm_shift_q7(pSrc, shiftBits, pDst, blockSize) arm_shift_q7 ((q7_t *)pSrc, shiftBits, (q7_t *)pDst, blockSize)

/* Wrapper function prototypes for Matrix Addition */
void mw_arm_mat_add_f32(real32_T * pSrcA, real32_T * pSrcB, real32_T * pDst, uint16_t nRows, uint16_t nCols);
void mw_arm_mat_add_q15(int16_T * pSrcA, int16_T * pSrcB, int16_T * pDst, uint16_t nRows, uint16_t nCols);
void mw_arm_mat_add_q31(int32_T * pSrcA, int32_T * pSrcB, int32_T * pDst, uint16_t nRows, uint16_t nCols);
/* Wrapper function prototypes for Matrix Subtraction */
void mw_arm_mat_sub_f32(real32_T * pSrcA, real32_T * pSrcB, real32_T * pDst, uint16_t nRows, uint16_t nCols);
void mw_arm_mat_sub_q15(int16_T * pSrcA, int16_T * pSrcB, int16_T * pDst, uint16_t nRows, uint16_t nCols);
void mw_arm_mat_sub_q31(int32_T * pSrcA, int32_T * pSrcB, int32_T * pDst, uint16_t nRows, uint16_t nCols);

#endif
