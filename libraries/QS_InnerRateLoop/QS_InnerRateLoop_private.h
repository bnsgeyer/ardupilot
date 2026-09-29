/*
 * QS_InnerRateLoop_private.h
 *
 * Classroom License -- for classroom instructional use only.  Not for
 * government, commercial, academic research, or other organizational use.
 *
 * Code generation for model "QS_InnerRateLoop".
 *
 * Model version              : 10.34
 * Simulink Coder version : 23.2 (R2023b) 01-Aug-2023
 * C++ source code generated on : Tue Sep 29 13:52:37 2026
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#ifndef RTW_HEADER_QS_InnerRateLoop_private_h_
#define RTW_HEADER_QS_InnerRateLoop_private_h_
#include "rtwtypes.h"
#include "multiword_types.h"
#include "QS_InnerRateLoop_types.h"
#include "QS_InnerRateLoop.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"

extern uint32_T plook_u32ff_evencg(real32_T u, real32_T bp0, real32_T bpSpace,
  uint32_T maxIndex, real32_T *fraction);
extern real32_T intrp1d_fu32fl_pw(uint32_T bpIndex, real32_T frac, const
  real32_T table[]);
extern uint32_T plook_u32ff_evenxg(real32_T u, real32_T bp0, real32_T bpSpace,
  uint32_T maxIndex, real32_T *fraction);
extern uint32_T plook_u32ff_bincg(real32_T u, const real32_T bp[], uint32_T
  maxIndex, real32_T *fraction);
extern real32_T intrp2d_fu32fl_pw(const uint32_T bpIndex[], const real32_T frac[],
  const real32_T table[], const uint32_T stride);

#endif                              /* RTW_HEADER_QS_InnerRateLoop_private_h_ */
