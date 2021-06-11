/*
 * QS_InnerRateLoop.cpp
 *
 * Student License - for use by students to meet course requirements and
 * perform academic research at degree granting institutions only.  Not
 * for government, commercial, or other organizational use.
 *
 * Code generation for model "QS_InnerRateLoop".
 *
 * Model version              : 1.539
 * Simulink Coder version : 9.1 (R2019a) 23-Nov-2018
 * C++ source code generated on : Wed Feb  9 13:00:44 2022
 *
 * Target selection: grt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "QS_InnerRateLoop.h"
#include "QS_InnerRateLoop_private.h"

uint32_T plook_u32ff_evencg(real32_T u, real32_T bp0, real32_T bpSpace, uint32_T
  maxIndex, real32_T *fraction)
{
  uint32_T bpIndex;
  real32_T invSpc;
  real32_T fbpIndex;

  /* Prelookup - Index and Fraction
     Index Search method: 'even'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'on'
   */
  invSpc = 1.0F / bpSpace;
  fbpIndex = (u - bp0) * invSpc;
  if (fbpIndex < maxIndex) {
    bpIndex = static_cast<uint32_T>(fbpIndex);
    *fraction = (u - (static_cast<real32_T>(bpIndex) * bpSpace + bp0)) * invSpc;
  } else {
    bpIndex = maxIndex - 1U;
    *fraction = 1.0F;
  }

  return bpIndex;
}

real32_T intrp1d_fu32fl_pw(uint32_T bpIndex, real32_T frac, const real32_T
  table[])
{
  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  return (table[bpIndex + 1U] - table[bpIndex]) * frac + table[bpIndex];
}

uint32_T plook_u32ff_evenca(real32_T u, real32_T bp0, real32_T bpSpace, uint32_T
  maxIndex, real32_T *fraction)
{
  uint32_T bpIndex;
  real32_T invSpc;
  real32_T fbpIndex;

  /* Prelookup - Index and Fraction
     Index Search method: 'even'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u <= bp0) {
    bpIndex = 0U;
    *fraction = 0.0F;
  } else {
    invSpc = 1.0F / bpSpace;
    fbpIndex = (u - bp0) * invSpc;
    if (fbpIndex < maxIndex) {
      bpIndex = static_cast<uint32_T>(fbpIndex);
      *fraction = (u - (static_cast<real32_T>(bpIndex) * bpSpace + bp0)) *
        invSpc;
    } else {
      bpIndex = maxIndex;
      *fraction = 0.0F;
    }
  }

  return bpIndex;
}

real32_T intrp1d_fu32fla_pw(uint32_T bpIndex, real32_T frac, const real32_T
  table[], uint32_T maxIndex)
{
  real32_T y;

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'on'
     Overflow mode: 'portable wrapping'
   */
  if (bpIndex == maxIndex) {
    y = table[bpIndex];
  } else {
    y = (table[bpIndex + 1U] - table[bpIndex]) * frac + table[bpIndex];
  }

  return y;
}

uint32_T plook_u32ff_evenxg(real32_T u, real32_T bp0, real32_T bpSpace, uint32_T
  maxIndex, real32_T *fraction)
{
  uint32_T bpIndex;
  real32_T invSpc;
  real32_T fbpIndex;

  /* Prelookup - Index and Fraction
     Index Search method: 'even'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'on'
   */
  invSpc = 1.0F / bpSpace;
  fbpIndex = (u - bp0) * invSpc;
  if (fbpIndex < maxIndex) {
    bpIndex = static_cast<uint32_T>(fbpIndex);
    *fraction = (u - (static_cast<real32_T>(bpIndex) * bpSpace + bp0)) * invSpc;
  } else {
    bpIndex = maxIndex - 1U;
    *fraction = (u - (static_cast<real32_T>((maxIndex - 1U)) * bpSpace + bp0)) *
      invSpc;
  }

  return bpIndex;
}

uint32_T plook_u32ff_bincg(real32_T u, const real32_T bp[], uint32_T maxIndex,
  real32_T *fraction)
{
  uint32_T bpIndex;
  uint32_T iRght;
  uint32_T bpIdx;

  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'off'
     Remove protection against out-of-range input in generated code: 'on'
   */
  /* Binary Search */
  bpIdx = maxIndex >> 1U;
  bpIndex = 0U;
  iRght = maxIndex;
  while (iRght - bpIndex > 1U) {
    if (u < bp[bpIdx]) {
      iRght = bpIdx;
    } else {
      bpIndex = bpIdx;
    }

    bpIdx = (iRght + bpIndex) >> 1U;
  }

  *fraction = (u - bp[bpIndex]) / (bp[bpIndex + 1U] - bp[bpIndex]);
  return bpIndex;
}

real32_T intrp2d_fu32fl_pw(const uint32_T bpIndex[], const real32_T frac[],
  const real32_T table[], const uint32_T stride)
{
  real32_T yL_1d;
  uint32_T offset_1d;

  /* Column-major Interpolation 2-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  offset_1d = bpIndex[1U] * stride + bpIndex[0U];
  yL_1d = (table[offset_1d + 1U] - table[offset_1d]) * frac[0U] +
    table[offset_1d];
  offset_1d += stride;
  return (((table[offset_1d + 1U] - table[offset_1d]) * frac[0U] +
           table[offset_1d]) - yL_1d) * frac[1U] + yL_1d;
}

/* Model step function */
void QS_InnerRateLoopModelClass::step()
{
  int32_T iU;
  uint32_T bpIdx;
  uint32_T bpIdx_0;
  uint32_T bpIndices[2];
  real32_T fractions[2];
  uint32_T bpIndices_0[2];
  real32_T fractions_0[2];
  real32_T rtb_Saturation1_p;
  real32_T rtb_uDLookupTable_cs;
  real32_T rtb_Saturation_h;
  real32_T rtb_Saturation8;
  real32_T rtb_Add5_e;
  real32_T rtb_Add6_i;
  real32_T rtb_TrigonometricFunction6;
  real32_T rtb_TrigonometricFunction3;
  real32_T rtb_TmpSignalConversionAtProduc[9];
  boolean_T rtb_Compare;
  real_T rtb_Sum2_h;
  boolean_T rtb_Compare_ab;
  boolean_T rtb_Compare_m;
  boolean_T rtb_Compare_bt;
  real32_T rtb_Product;
  real32_T rtb_Product_ng;
  real32_T rtb_Saturation2_h;
  real32_T rtb_Product_ee;
  real32_T rtb_Switch2_cz;
  real32_T rtb_Product2;
  real32_T rtb_Gain2;
  real32_T rtb_Sum4_a;
  real32_T rtb_uDLookupTable3;
  real32_T rtb_Sum3_f;
  real32_T rtb_derivativecutofffrequency_b;
  real32_T rtb_Sum4_b;
  real32_T rtb_Add5_ky;
  real32_T rtb_Add7_d;
  real32_T rtb_Sum2_k;
  real32_T rtb_Abs;
  real32_T rtb_Add6_a;
  real32_T rtb_Switch_ep;
  real32_T rtb_Sum_j3;
  real32_T rtb_Product_of;
  real32_T rtb_Gain1;
  real32_T rtb_DiscreteTimeIntegrator1;
  boolean_T rtb_Compare_nh;
  real32_T rtb_Switch3;
  real_T rtb_Gain_e;
  real32_T rtb_Abs_i;
  real32_T rtb_Sum1_c0;
  real32_T rtb_Product_ib;
  real32_T rtb_uDLookupTable1;
  real32_T rtb_DiscreteTimeIntegrator_hs;
  real32_T rtb_Sum1_fe;
  real32_T rtb_Product_jr;
  real32_T rtb_uDLookupTable;
  real32_T rtb_uDLookupTable1_p;
  real32_T rtb_DiscreteTimeIntegrator_cv;
  real32_T rtb_Saturation_k;
  real32_T rtb_uDLookupTable2_bw;
  real32_T rtb_Product_n;
  real32_T rtb_uDLookupTable1_l;
  real32_T rtb_DiscreteTimeIntegrator_nt;
  real32_T rtb_Product_ear;
  real32_T rtb_DiscreteTimeIntegrator_ia;
  real32_T rtb_Product_hg[4];
  real32_T rtb_DeadZone3;
  real32_T rtb_DiscreteTimeIntegrator_fi;
  real32_T rtb_uDLookupTable_a;
  real32_T rtb_DiscreteTimeIntegrator_k1;
  real32_T rtb_Sum1_lq;
  real32_T rtb_DiscreteTimeIntegrator_h;
  real32_T rtb_Sum1_gg;
  real32_T rtb_DiscreteTimeIntegrator_n;
  boolean_T rtb_RelationalOperator_b1;
  real_T rtb_Sum;
  real32_T rtb_DeadZone;
  real32_T rtb_Abs_k;
  real32_T rtb_Product_cw;
  real32_T rtb_Sum1_gw;
  real32_T rtb_uDLookupTable2[36];
  real32_T rtb_uDLookupTable2_k5[16];
  real32_T rtb_Sum1_f[4];
  real32_T rtb_Product_b[4];
  int32_T i;
  real32_T tmp;
  real32_T rtb_Sum1_j_idx_0;
  real32_T rtb_Sum1_j_idx_1;
  real32_T rtb_Sum1_j_idx_2;
  real32_T rtb_Compare_k;
  int32_T tmp_0;
  real32_T rtb_uDLookupTable_l_tmp;
  real32_T rtb_Saturation_fr_tmp;
  real32_T rtb_TrigonometricFunction3_tmp;
  real32_T rtb_TrigonometricFunction6_tmp;
  real32_T rtb_Product_b_tmp;
  real32_T rtb_derivativecutofffrequency_0;
  real32_T rtb_Product_b_tmp_0;

  /* Trigonometry: '<S9>/Trigonometric Function3' incorporates:
   *  Inport: '<Root>/psi_rad'
   *  Trigonometry: '<S72>/Trigonometric Function3'
   */
  rtb_TrigonometricFunction6 = static_cast<real32_T>(cos((real_T)
    QS_InnerRateLoop_U.psi_rad));

  /* Trigonometry: '<S9>/Trigonometric Function6' incorporates:
   *  Inport: '<Root>/psi_rad'
   *  Trigonometry: '<S72>/Trigonometric Function6'
   */
  rtb_uDLookupTable_l_tmp = static_cast<real32_T>(sin((real_T)
    QS_InnerRateLoop_U.psi_rad));

  /* Sum: '<S9>/Add5' incorporates:
   *  Inport: '<Root>/vD_fps (KF)'
   *  Inport: '<Root>/vE_fps (KF)'
   *  Inport: '<Root>/vN_fps (KF)'
   *  Product: '<S9>/Divide'
   *  Product: '<S9>/Divide2'
   *  Product: '<S9>/Divide6'
   *  Trigonometry: '<S9>/Trigonometric Function3'
   *  Trigonometry: '<S9>/Trigonometric Function6'
   */
  rtb_Add5_e = (QS_InnerRateLoop_ConstB.TrigonometricFunction2 *
                rtb_TrigonometricFunction6 * QS_InnerRateLoop_U.vN_fpsKF +
                QS_InnerRateLoop_ConstB.TrigonometricFunction2 *
                rtb_uDLookupTable_l_tmp * QS_InnerRateLoop_U.vE_fpsKF) +
    -QS_InnerRateLoop_ConstB.TrigonometricFunction5 *
    QS_InnerRateLoop_U.vD_fpsKF;

  /* Product: '<S9>/Divide1' incorporates:
   *  Product: '<S9>/Divide7'
   */
  rtb_Add6_i = QS_InnerRateLoop_ConstB.TrigonometricFunction4 *
    QS_InnerRateLoop_ConstB.TrigonometricFunction5;

  /* Sum: '<S9>/Add6' incorporates:
   *  Inport: '<Root>/vD_fps (KF)'
   *  Inport: '<Root>/vE_fps (KF)'
   *  Inport: '<Root>/vN_fps (KF)'
   *  Product: '<S9>/Divide1'
   *  Product: '<S9>/Divide11'
   *  Product: '<S9>/Divide13'
   *  Product: '<S9>/Divide15'
   *  Product: '<S9>/Divide3'
   *  Product: '<S9>/Divide7'
   *  Product: '<S9>/Divide8'
   *  Sum: '<S9>/Add1'
   *  Sum: '<S9>/Add3'
   *  Trigonometry: '<S9>/Trigonometric Function3'
   *  Trigonometry: '<S9>/Trigonometric Function6'
   */
  rtb_Add6_i = ((rtb_Add6_i * rtb_TrigonometricFunction6 -
                 QS_InnerRateLoop_ConstB.TrigonometricFunction1 *
                 rtb_uDLookupTable_l_tmp) * QS_InnerRateLoop_U.vN_fpsKF +
                (rtb_Add6_i * rtb_uDLookupTable_l_tmp +
                 QS_InnerRateLoop_ConstB.TrigonometricFunction1 *
                 rtb_TrigonometricFunction6) * QS_InnerRateLoop_U.vE_fpsKF) +
    QS_InnerRateLoop_ConstB.TrigonometricFunction4 *
    QS_InnerRateLoop_ConstB.TrigonometricFunction2 * QS_InnerRateLoop_U.vD_fpsKF;

  /* Sqrt: '<Root>/Sqrt1' incorporates:
   *  Math: '<Root>/Math Function3'
   *  Math: '<Root>/Math Function4'
   *  Sum: '<Root>/Add1'
   *
   * About '<Root>/Math Function3':
   *  Operator: magnitude^2
   *
   * About '<Root>/Math Function4':
   *  Operator: magnitude^2
   */
  rtb_Saturation8 = static_cast<real32_T>(sqrt((real_T)(rtb_Add5_e * rtb_Add5_e
    + rtb_Add6_i * rtb_Add6_i)));

  /* Saturate: '<Root>/Saturation8' */
  if (rtb_Saturation8 > 6.5F) {
    rtb_Saturation8 = 6.5F;
  } else {
    if (rtb_Saturation8 < 0.0F) {
      rtb_Saturation8 = 0.0F;
    }
  }

  /* End of Saturate: '<Root>/Saturation8' */

  /* Lookup_n-D: '<S64>/1-D Lookup Table2' incorporates:
   *  Constant: '<S64>/Constant'
   *  Delay: '<S13>/Delay1'
   *  Lookup_n-D: '<S63>/1-D Lookup Table2'
   */
  bpIndices[0U] = plook_u32ff_bincg(rtb_Saturation8,
    QS_InnerRateLoop_ConstP.pooled7, 4U, &rtb_Abs_i);
  fractions[0U] = rtb_Abs_i;
  for (iU = 0; iU < 36; iU++) {
    bpIndices[1U] = plook_u32ff_bincg(QS_InnerRateLoop_ConstP.pooled17[iU],
      QS_InnerRateLoop_ConstP.pooled17, 35U, &rtb_Abs_i);
    fractions[1U] = rtb_Abs_i;
    rtb_uDLookupTable2[iU] = intrp2d_fu32fl_pw(bpIndices, fractions,
      QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_g, 5U);
  }

  /* Lookup_n-D: '<S63>/1-D Lookup Table2' incorporates:
   *  Constant: '<S63>/Constant'
   *  Delay: '<S13>/Delay1'
   *  Lookup_n-D: '<S64>/1-D Lookup Table2'
   */
  bpIndices_0[0U] = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U,
    &rtb_Abs_i);
  fractions_0[0U] = rtb_Abs_i;
  for (iU = 0; iU < 16; iU++) {
    bpIndices_0[1U] = plook_u32ff_evencg(QS_InnerRateLoop_ConstP.pooled18[iU],
      1.0F, 1.0F, 15U, &rtb_Abs_i);
    fractions_0[1U] = rtb_Abs_i;
    rtb_uDLookupTable2_k5[iU] = intrp2d_fu32fl_pw(bpIndices_0, fractions_0,
      QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_b, 5U);
  }

  /* Trigonometry: '<S8>/Trigonometric Function2' incorporates:
   *  Inport: '<Root>/theta_rad'
   *  Trigonometry: '<S7>/Trigonometric Function2'
   */
  rtb_Saturation_fr_tmp = static_cast<real32_T>(cos((real_T)
    QS_InnerRateLoop_U.theta_rad));

  /* Product: '<S9>/Divide4' incorporates:
   *  Product: '<S9>/Divide9'
   */
  rtb_uDLookupTable_cs = QS_InnerRateLoop_ConstB.TrigonometricFunction1 *
    QS_InnerRateLoop_ConstB.TrigonometricFunction5;

  /* Sum: '<S9>/Add7' incorporates:
   *  Inport: '<Root>/vD_fps (KF)'
   *  Inport: '<Root>/vE_fps (KF)'
   *  Inport: '<Root>/vN_fps (KF)'
   *  Product: '<S9>/Divide10'
   *  Product: '<S9>/Divide12'
   *  Product: '<S9>/Divide14'
   *  Product: '<S9>/Divide16'
   *  Product: '<S9>/Divide4'
   *  Product: '<S9>/Divide5'
   *  Product: '<S9>/Divide9'
   *  Sum: '<S9>/Add2'
   *  Sum: '<S9>/Add4'
   *  Trigonometry: '<S9>/Trigonometric Function3'
   *  Trigonometry: '<S9>/Trigonometric Function6'
   */
  rtb_uDLookupTable_cs = ((rtb_uDLookupTable_cs * rtb_TrigonometricFunction6 +
    QS_InnerRateLoop_ConstB.TrigonometricFunction4 * rtb_uDLookupTable_l_tmp) *
    QS_InnerRateLoop_U.vN_fpsKF + (rtb_uDLookupTable_cs *
    rtb_uDLookupTable_l_tmp - QS_InnerRateLoop_ConstB.TrigonometricFunction4 *
    rtb_TrigonometricFunction6) * QS_InnerRateLoop_U.vE_fpsKF) +
    QS_InnerRateLoop_ConstB.TrigonometricFunction1 *
    QS_InnerRateLoop_ConstB.TrigonometricFunction2 * QS_InnerRateLoop_U.vD_fpsKF;

  /* Trigonometry: '<S8>/Trigonometric Function5' incorporates:
   *  Inport: '<Root>/theta_rad'
   */
  rtb_Saturation1_p = static_cast<real32_T>(sin((real_T)
    QS_InnerRateLoop_U.theta_rad));

  /* Trigonometry: '<S8>/Trigonometric Function4' incorporates:
   *  Inport: '<Root>/phi_rad'
   *  Trigonometry: '<S7>/Trigonometric Function4'
   */
  rtb_TrigonometricFunction6_tmp = static_cast<real32_T>(sin((real_T)
    QS_InnerRateLoop_U.phi_rad));

  /* Trigonometry: '<S8>/Trigonometric Function1' incorporates:
   *  Inport: '<Root>/phi_rad'
   *  Trigonometry: '<S7>/Trigonometric Function1'
   */
  rtb_TrigonometricFunction3_tmp = static_cast<real32_T>(cos((real_T)
    QS_InnerRateLoop_U.phi_rad));

  /* SignalConversion: '<S64>/TmpSignal ConversionAtProductInport2' incorporates:
   *  Product: '<S8>/Divide'
   *  Product: '<S8>/Divide2'
   *  Product: '<S8>/Divide6'
   *  Sum: '<S8>/Add5'
   *  Trigonometry: '<S8>/Trigonometric Function2'
   */
  rtb_TmpSignalConversionAtProduc[0] = (rtb_Saturation_fr_tmp *
    QS_InnerRateLoop_ConstB.TrigonometricFunction3 * rtb_Add5_e +
    rtb_Saturation_fr_tmp * QS_InnerRateLoop_ConstB.TrigonometricFunction6 *
    rtb_Add6_i) + -rtb_Saturation1_p * rtb_uDLookupTable_cs;

  /* Product: '<S8>/Divide1' incorporates:
   *  Product: '<S8>/Divide7'
   *  Trigonometry: '<S8>/Trigonometric Function4'
   */
  rtb_Product = rtb_TrigonometricFunction6_tmp * rtb_Saturation1_p;

  /* SignalConversion: '<S64>/TmpSignal ConversionAtProductInport2' incorporates:
   *  Product: '<S8>/Divide1'
   *  Product: '<S8>/Divide11'
   *  Product: '<S8>/Divide13'
   *  Product: '<S8>/Divide15'
   *  Product: '<S8>/Divide3'
   *  Product: '<S8>/Divide7'
   *  Product: '<S8>/Divide8'
   *  Sum: '<S8>/Add1'
   *  Sum: '<S8>/Add3'
   *  Sum: '<S8>/Add6'
   *  Trigonometry: '<S8>/Trigonometric Function1'
   *  Trigonometry: '<S8>/Trigonometric Function2'
   *  Trigonometry: '<S8>/Trigonometric Function4'
   */
  rtb_TmpSignalConversionAtProduc[1] = ((rtb_Product *
    QS_InnerRateLoop_ConstB.TrigonometricFunction3 -
    rtb_TrigonometricFunction3_tmp *
    QS_InnerRateLoop_ConstB.TrigonometricFunction6) * rtb_Add5_e + (rtb_Product *
    QS_InnerRateLoop_ConstB.TrigonometricFunction6 +
    rtb_TrigonometricFunction3_tmp *
    QS_InnerRateLoop_ConstB.TrigonometricFunction3) * rtb_Add6_i) +
    rtb_TrigonometricFunction6_tmp * rtb_Saturation_fr_tmp *
    rtb_uDLookupTable_cs;

  /* Product: '<S8>/Divide4' incorporates:
   *  Product: '<S8>/Divide9'
   *  Trigonometry: '<S8>/Trigonometric Function1'
   */
  rtb_Product = rtb_TrigonometricFunction3_tmp * rtb_Saturation1_p;

  /* SignalConversion: '<S64>/TmpSignal ConversionAtProductInport2' incorporates:
   *  Inport: '<Root>/p_rps'
   *  Inport: '<Root>/phi_rad'
   *  Inport: '<Root>/psi_rad'
   *  Inport: '<Root>/q_rps'
   *  Inport: '<Root>/r_rps'
   *  Inport: '<Root>/theta_rad'
   *  Product: '<S8>/Divide10'
   *  Product: '<S8>/Divide12'
   *  Product: '<S8>/Divide14'
   *  Product: '<S8>/Divide16'
   *  Product: '<S8>/Divide4'
   *  Product: '<S8>/Divide5'
   *  Product: '<S8>/Divide9'
   *  Sum: '<S8>/Add2'
   *  Sum: '<S8>/Add4'
   *  Sum: '<S8>/Add7'
   *  Trigonometry: '<S8>/Trigonometric Function1'
   *  Trigonometry: '<S8>/Trigonometric Function2'
   *  Trigonometry: '<S8>/Trigonometric Function4'
   */
  rtb_TmpSignalConversionAtProduc[2] = ((rtb_Product *
    QS_InnerRateLoop_ConstB.TrigonometricFunction3 +
    rtb_TrigonometricFunction6_tmp *
    QS_InnerRateLoop_ConstB.TrigonometricFunction6) * rtb_Add5_e + (rtb_Product *
    QS_InnerRateLoop_ConstB.TrigonometricFunction6 -
    rtb_TrigonometricFunction6_tmp *
    QS_InnerRateLoop_ConstB.TrigonometricFunction3) * rtb_Add6_i) +
    rtb_TrigonometricFunction3_tmp * rtb_Saturation_fr_tmp *
    rtb_uDLookupTable_cs;
  rtb_TmpSignalConversionAtProduc[3] = QS_InnerRateLoop_U.p_rps;
  rtb_TmpSignalConversionAtProduc[4] = QS_InnerRateLoop_U.q_rps;
  rtb_TmpSignalConversionAtProduc[5] = QS_InnerRateLoop_U.r_rps;
  rtb_TmpSignalConversionAtProduc[6] = QS_InnerRateLoop_U.phi_rad;
  rtb_TmpSignalConversionAtProduc[7] = QS_InnerRateLoop_U.theta_rad;
  rtb_TmpSignalConversionAtProduc[8] = QS_InnerRateLoop_U.psi_rad;

  /* RelationalOperator: '<S3>/Compare' incorporates:
   *  Constant: '<S3>/Constant'
   *  Inport: '<Root>/CH8_flag'
   */
  rtb_Compare = (QS_InnerRateLoop_U.CH8_flag > 0.0F);

  /* Sum: '<S96>/Sum2' incorporates:
   *  Delay: '<S96>/Delay1'
   *  Delay: '<S96>/Delay2'
   *  RelationalOperator: '<S96>/Relational Operator2'
   */
  rtb_Sum2_h = static_cast<real_T>((static_cast<int32_T>(rtb_Compare) >
    static_cast<int32_T>(QS_InnerRateLoop_DW.Delay1_DSTATE))) +
    QS_InnerRateLoop_DW.Delay2_DSTATE;

  /* Outputs for Enabled SubSystem: '<S11>/Enabled Subsystem1' incorporates:
   *  EnablePort: '<S98>/Enable'
   */
  /* Logic: '<S11>/Logical Operator3' incorporates:
   *  Inport: '<Root>/input_col'
   *  Inport: '<Root>/input_lat'
   *  Inport: '<Root>/input_lon'
   *  Inport: '<Root>/input_ped'
   *  Inport: '<S98>/col'
   *  Inport: '<S98>/lat'
   *  Inport: '<S98>/lon'
   *  Inport: '<S98>/ped'
   */
  if (!rtb_Compare) {
    QS_InnerRateLoop_B.lon = QS_InnerRateLoop_U.input_lon;
    QS_InnerRateLoop_B.lat = QS_InnerRateLoop_U.input_lat;
    QS_InnerRateLoop_B.col_h = QS_InnerRateLoop_U.input_col;
    QS_InnerRateLoop_B.ped = QS_InnerRateLoop_U.input_ped;
  }

  /* End of Logic: '<S11>/Logical Operator3' */
  /* End of Outputs for SubSystem: '<S11>/Enabled Subsystem1' */

  /* Sum: '<S11>/Sum1' incorporates:
   *  Inport: '<Root>/input_lon'
   */
  rtb_TrigonometricFunction3 = QS_InnerRateLoop_U.input_lon -
    QS_InnerRateLoop_B.lon;

  /* DeadZone: '<S97>/Dead Zone2' */
  if (rtb_TrigonometricFunction3 > 0.05F) {
    rtb_TrigonometricFunction3 -= 0.05F;
  } else if (rtb_TrigonometricFunction3 >= -0.05F) {
    rtb_TrigonometricFunction3 = 0.0F;
  } else {
    rtb_TrigonometricFunction3 -= -0.05F;
  }

  /* End of DeadZone: '<S97>/Dead Zone2' */

  /* Abs: '<S97>/Abs' */
  rtb_TrigonometricFunction3 = static_cast<real32_T>(fabs((real_T)
    rtb_TrigonometricFunction3));

  /* RelationalOperator: '<S99>/Compare' incorporates:
   *  Constant: '<S99>/Constant'
   */
  rtb_Compare_ab = (rtb_TrigonometricFunction3 > 0.0F);

  /* Sum: '<S11>/Sum2' incorporates:
   *  Inport: '<Root>/input_lat'
   */
  rtb_TrigonometricFunction3 = QS_InnerRateLoop_U.input_lat -
    QS_InnerRateLoop_B.lat;

  /* DeadZone: '<S97>/Dead Zone1' */
  if (rtb_TrigonometricFunction3 > 0.05F) {
    rtb_TrigonometricFunction3 -= 0.05F;
  } else if (rtb_TrigonometricFunction3 >= -0.05F) {
    rtb_TrigonometricFunction3 = 0.0F;
  } else {
    rtb_TrigonometricFunction3 -= -0.05F;
  }

  /* End of DeadZone: '<S97>/Dead Zone1' */

  /* Abs: '<S97>/Abs1' */
  rtb_TrigonometricFunction3 = static_cast<real32_T>(fabs((real_T)
    rtb_TrigonometricFunction3));

  /* RelationalOperator: '<S100>/Compare' incorporates:
   *  Constant: '<S100>/Constant'
   */
  rtb_Compare_m = (rtb_TrigonometricFunction3 > 0.0F);

  /* Sum: '<S11>/Sum3' incorporates:
   *  Inport: '<Root>/input_col'
   */
  rtb_TrigonometricFunction3 = QS_InnerRateLoop_U.input_col -
    QS_InnerRateLoop_B.col_h;

  /* DeadZone: '<S97>/Dead Zone3' */
  if (rtb_TrigonometricFunction3 > 0.05F) {
    rtb_TrigonometricFunction3 -= 0.05F;
  } else if (rtb_TrigonometricFunction3 >= -0.05F) {
    rtb_TrigonometricFunction3 = 0.0F;
  } else {
    rtb_TrigonometricFunction3 -= -0.05F;
  }

  /* End of DeadZone: '<S97>/Dead Zone3' */

  /* Abs: '<S97>/Abs2' */
  rtb_TrigonometricFunction3 = static_cast<real32_T>(fabs((real_T)
    rtb_TrigonometricFunction3));

  /* RelationalOperator: '<S101>/Compare' incorporates:
   *  Constant: '<S101>/Constant'
   */
  rtb_Compare_bt = (rtb_TrigonometricFunction3 > 0.0F);

  /* Sum: '<S11>/Sum4' incorporates:
   *  Inport: '<Root>/input_ped'
   */
  rtb_TrigonometricFunction3 = QS_InnerRateLoop_U.input_ped -
    QS_InnerRateLoop_B.ped;

  /* DeadZone: '<S97>/Dead Zone4' */
  if (rtb_TrigonometricFunction3 > 0.05F) {
    rtb_TrigonometricFunction3 -= 0.05F;
  } else if (rtb_TrigonometricFunction3 >= -0.05F) {
    rtb_TrigonometricFunction3 = 0.0F;
  } else {
    rtb_TrigonometricFunction3 -= -0.05F;
  }

  /* End of DeadZone: '<S97>/Dead Zone4' */

  /* Abs: '<S97>/Abs3' */
  rtb_TrigonometricFunction3 = static_cast<real32_T>(fabs((real_T)
    rtb_TrigonometricFunction3));

  /* Logic: '<S11>/Logical Operator2' incorporates:
   *  Constant: '<S102>/Constant'
   *  Logic: '<S97>/Logical Operator'
   *  Logic: '<S97>/Logical Operator1'
   *  RelationalOperator: '<S102>/Compare'
   */
  rtb_Compare_ab = ((!rtb_Compare_ab) && (!rtb_Compare_m) && (!rtb_Compare_bt) &&
                    (!(rtb_TrigonometricFunction3 > 0.0F)) && rtb_Compare);

  /* Logic: '<S11>/Logical Operator1' incorporates:
   *  Abs: '<S11>/Abs'
   *  Constant: '<S95>/Constant'
   *  RelationalOperator: '<S95>/Compare'
   */
  rtb_Compare_m = ((fabs(rtb_Sum2_h) > 0.0) && rtb_Compare_ab);

  /* DiscreteIntegrator: '<S94>/Discrete-Time Integrator' */
  if (rtb_Compare_m && (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRese <= 0))
  {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_a = 0.0F;
  }

  /* Lookup_n-D: '<S92>/1-D Lookup Table2' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* DiscreteIntegrator: '<S69>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/pos North (KF)'
   */
  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOADI != 0) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_m =
      QS_InnerRateLoop_U.posNorthKF;
  }

  if ((rtb_Compare_m && (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_a <=
                         0)) || ((!rtb_Compare_m) &&
       (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_a == 1))) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_m =
      QS_InnerRateLoop_U.posNorthKF;
  }

  /* Product: '<S92>/Product' incorporates:
   *  DiscreteIntegrator: '<S69>/Discrete-Time Integrator'
   *  Inport: '<Root>/pos North (KF)'
   *  Lookup_n-D: '<S92>/1-D Lookup Table2'
   *  Sum: '<S77>/Sum'
   */
  rtb_Product = (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_m -
                 QS_InnerRateLoop_U.posNorthKF) * intrp1d_fu32fl_pw(bpIdx,
    rtb_Abs_i, QS_InnerRateLoop_ConstP.pooled20);

  /* Sum: '<S77>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S94>/Discrete-Time Integrator'
   */
  rtb_Saturation_h = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_a +
    rtb_Product;

  /* Saturate: '<S70>/Saturation' */
  if (rtb_Saturation_h > 10.0F) {
    rtb_Saturation_h = 10.0F;
  } else {
    if (rtb_Saturation_h < -10.0F) {
      rtb_Saturation_h = -10.0F;
    }
  }

  /* End of Saturate: '<S70>/Saturation' */

  /* DiscreteIntegrator: '<S91>/Discrete-Time Integrator' */
  if (rtb_Compare_m && (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_o <= 0))
  {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_h = 0.0F;
  }

  /* Lookup_n-D: '<S89>/1-D Lookup Table2' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* DiscreteIntegrator: '<S69>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/pos East (KF)'
   */
  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_IC_LOAD != 0) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTATE =
      QS_InnerRateLoop_U.posEastKF;
  }

  if ((rtb_Compare_m && (QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevRes <=
                         0)) || ((!rtb_Compare_m) &&
       (QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevRes == 1))) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTATE =
      QS_InnerRateLoop_U.posEastKF;
  }

  /* Product: '<S89>/Product' incorporates:
   *  DiscreteIntegrator: '<S69>/Discrete-Time Integrator1'
   *  Inport: '<Root>/pos East (KF)'
   *  Lookup_n-D: '<S89>/1-D Lookup Table2'
   *  Sum: '<S76>/Sum'
   */
  rtb_Product_ng = (QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTATE -
                    QS_InnerRateLoop_U.posEastKF) * intrp1d_fu32fl_pw(bpIdx,
    rtb_Abs_i, QS_InnerRateLoop_ConstP.pooled20);

  /* Sum: '<S76>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S91>/Discrete-Time Integrator'
   */
  rtb_Saturation1_p = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_h +
    rtb_Product_ng;

  /* Saturate: '<S70>/Saturation1' */
  if (rtb_Saturation1_p > 10.0F) {
    rtb_Saturation1_p = 10.0F;
  } else {
    if (rtb_Saturation1_p < -10.0F) {
      rtb_Saturation1_p = -10.0F;
    }
  }

  /* End of Saturate: '<S70>/Saturation1' */

  /* DiscreteIntegrator: '<S88>/Discrete-Time Integrator' */
  if (rtb_Compare_m && (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_b <= 0))
  {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_c = 0.0F;
  }

  /* Lookup_n-D: '<S87>/1-D Lookup Table2' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* DiscreteIntegrator: '<S69>/Discrete-Time Integrator2' incorporates:
   *  Inport: '<Root>/pos Down (KF)'
   */
  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_IC_LOAD != 0) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_DSTATE =
      QS_InnerRateLoop_U.posDownKF;
  }

  if ((rtb_Compare_m && (QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_PrevRes <=
                         0)) || ((!rtb_Compare_m) &&
       (QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_PrevRes == 1))) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_DSTATE =
      QS_InnerRateLoop_U.posDownKF;
  }

  /* Product: '<S87>/Product' incorporates:
   *  DiscreteIntegrator: '<S69>/Discrete-Time Integrator2'
   *  Inport: '<Root>/pos Down (KF)'
   *  Lookup_n-D: '<S87>/1-D Lookup Table2'
   *  Sum: '<S75>/Sum'
   */
  rtb_Product_ee = (QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_DSTATE -
                    QS_InnerRateLoop_U.posDownKF) * intrp1d_fu32fl_pw(bpIdx,
    rtb_Abs_i, QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_k);

  /* Sum: '<S75>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S88>/Discrete-Time Integrator'
   */
  rtb_Saturation2_h = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_c +
    rtb_Product_ee;

  /* Saturate: '<S70>/Saturation2' */
  if (rtb_Saturation2_h > 15.0F) {
    rtb_Saturation2_h = 15.0F;
  } else {
    if (rtb_Saturation2_h < -15.0F) {
      rtb_Saturation2_h = -15.0F;
    }
  }

  /* End of Saturate: '<S70>/Saturation2' */

  /* Product: '<S12>/Product2' incorporates:
   *  DataTypeConversion: '<Root>/Data Type Conversion'
   *  Sum: '<S12>/Sum21'
   *  UnitDelay: '<S12>/Unit Delay'
   */
  rtb_Product2 = (static_cast<real32_T>(rtb_Compare_m) +
                  QS_InnerRateLoop_DW.UnitDelay_DSTATE) * static_cast<real32_T>
    (rtb_Compare_m);

  /* Gain: '<S12>/Gain2' */
  rtb_Gain2 = 0.0025F * rtb_Product2;

  /* Switch: '<S12>/Switch1' incorporates:
   *  Constant: '<S12>/Constant'
   *  Constant: '<S12>/Constant1'
   *  Sum: '<S12>/Sum'
   */
  if (rtb_Gain2 >= 20.0F) {
    rtb_uDLookupTable3 = rtb_Gain2 - 20.0F;
  } else {
    rtb_uDLookupTable3 = 0.0F;
  }

  /* End of Switch: '<S12>/Switch1' */

  /* Product: '<S12>/Divide1' incorporates:
   *  Inport: '<Root>/Rp'
   *  Inport: '<Root>/Rv'
   */
  rtb_Sum4_a = rtb_uDLookupTable3 / QS_InnerRateLoop_U.Rv *
    QS_InnerRateLoop_U.Rp;

  /* Lookup_n-D: '<S12>/1-D Lookup Table3' */
  bpIdx = plook_u32ff_evenca(rtb_Sum4_a, 0.0F, 0.2F, 23U, &rtb_Abs_i);
  rtb_uDLookupTable3 = intrp1d_fu32fla_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.pooled26, 23U);

  /* Gain: '<S12>/Gain1' */
  rtb_Sum3_f = 0.0174532924F * rtb_uDLookupTable3;

  /* Trigonometry: '<S103>/Trigonometric Function3' */
  rtb_derivativecutofffrequency_b = static_cast<real32_T>(cos((real_T)rtb_Sum3_f));

  /* Trigonometry: '<S103>/Trigonometric Function6' */
  rtb_Sum3_f = static_cast<real32_T>(sin((real_T)rtb_Sum3_f));

  /* Lookup_n-D: '<S12>/1-D Lookup Table1' */
  bpIdx = plook_u32ff_evenca(rtb_Sum4_a, 0.0F, 0.2F, 23U, &rtb_Abs_i);

  /* Product: '<S12>/Divide3' incorporates:
   *  Inport: '<Root>/Rv'
   *  Lookup_n-D: '<S12>/1-D Lookup Table1'
   */
  rtb_Sum4_b = intrp1d_fu32fla_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.pooled26, 23U) / QS_InnerRateLoop_U.Rv;

  /* Lookup_n-D: '<S12>/1-D Lookup Table' */
  bpIdx = plook_u32ff_evenca(rtb_Sum4_a, 0.0F, 0.2F, 23U, &rtb_Abs_i);

  /* Product: '<S12>/Divide2' incorporates:
   *  Inport: '<Root>/Rv'
   *  Lookup_n-D: '<S12>/1-D Lookup Table'
   */
  rtb_Add5_ky = intrp1d_fu32fla_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable_tableData, 23U) /
    QS_InnerRateLoop_U.Rv;

  /* Lookup_n-D: '<S12>/1-D Lookup Table2' */
  bpIdx = plook_u32ff_evenca(rtb_Sum4_a, 0.0F, 0.2F, 23U, &rtb_Abs_i);

  /* Product: '<S12>/Divide4' incorporates:
   *  Inport: '<Root>/Rv'
   *  Lookup_n-D: '<S12>/1-D Lookup Table2'
   */
  rtb_Sum4_a = intrp1d_fu32fla_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.pooled26, 23U) / QS_InnerRateLoop_U.Rv;

  /* Product: '<S103>/Divide4' incorporates:
   *  Product: '<S103>/Divide9'
   */
  rtb_Add7_d = QS_InnerRateLoop_ConstB.TrigonometricFunction1_k *
    QS_InnerRateLoop_ConstB.TrigonometricFunction5_o;

  /* Sum: '<S103>/Add7' incorporates:
   *  Product: '<S103>/Divide10'
   *  Product: '<S103>/Divide12'
   *  Product: '<S103>/Divide14'
   *  Product: '<S103>/Divide16'
   *  Product: '<S103>/Divide4'
   *  Product: '<S103>/Divide5'
   *  Product: '<S103>/Divide9'
   *  Sum: '<S103>/Add2'
   *  Sum: '<S103>/Add4'
   */
  rtb_Add7_d = ((rtb_Add7_d * rtb_derivativecutofffrequency_b +
                 QS_InnerRateLoop_ConstB.TrigonometricFunction4_n * rtb_Sum3_f) *
                rtb_Sum4_b + (rtb_Add7_d * rtb_Sum3_f -
    QS_InnerRateLoop_ConstB.TrigonometricFunction4_n *
    rtb_derivativecutofffrequency_b) * rtb_Add5_ky) +
    QS_InnerRateLoop_ConstB.TrigonometricFunction1_k *
    QS_InnerRateLoop_ConstB.TrigonometricFunction2_l * rtb_Sum4_a;

  /* Product: '<S72>/Divide4' incorporates:
   *  Product: '<S72>/Divide9'
   */
  rtb_Sum2_k = QS_InnerRateLoop_ConstB.TrigonometricFunction1_g *
    QS_InnerRateLoop_ConstB.TrigonometricFunction5_j;

  /* Sum: '<S70>/Sum2' incorporates:
   *  Product: '<S72>/Divide10'
   *  Product: '<S72>/Divide12'
   *  Product: '<S72>/Divide14'
   *  Product: '<S72>/Divide16'
   *  Product: '<S72>/Divide4'
   *  Product: '<S72>/Divide5'
   *  Product: '<S72>/Divide9'
   *  Sum: '<S72>/Add2'
   *  Sum: '<S72>/Add4'
   *  Sum: '<S72>/Add7'
   */
  rtb_Sum2_k = (((rtb_Sum2_k * rtb_TrigonometricFunction6 +
                  QS_InnerRateLoop_ConstB.TrigonometricFunction4_k *
                  rtb_uDLookupTable_l_tmp) * rtb_Saturation_h + (rtb_Sum2_k *
    rtb_uDLookupTable_l_tmp - QS_InnerRateLoop_ConstB.TrigonometricFunction4_k *
    rtb_TrigonometricFunction6) * rtb_Saturation1_p) +
                QS_InnerRateLoop_ConstB.TrigonometricFunction1_g *
                QS_InnerRateLoop_ConstB.TrigonometricFunction2_a *
                rtb_Saturation2_h) + rtb_Add7_d;

  /* Saturate: '<S13>/Saturation2' */
  if (rtb_Sum2_k > 15.0F) {
    rtb_Abs = 15.0F;
  } else if (rtb_Sum2_k < -15.0F) {
    rtb_Abs = -15.0F;
  } else {
    rtb_Abs = rtb_Sum2_k;
  }

  /* End of Saturate: '<S13>/Saturation2' */

  /* Outputs for Enabled SubSystem: '<S13>/Enabled Subsystem1' incorporates:
   *  EnablePort: '<S104>/Enable'
   */
  /* Logic: '<S13>/Logical Operator2' incorporates:
   *  Inport: '<Root>/engage'
   *  Inport: '<Root>/input_col'
   *  Inport: '<S104>/col'
   */
  if (!QS_InnerRateLoop_U.engage) {
    QS_InnerRateLoop_B.col = QS_InnerRateLoop_U.input_col;
  }

  /* End of Logic: '<S13>/Logical Operator2' */
  /* End of Outputs for SubSystem: '<S13>/Enabled Subsystem1' */

  /* Switch: '<S108>/Switch' incorporates:
   *  Gain: '<S13>/wcmd'
   */
  if (rtb_Compare_m) {
    rtb_Switch_ep = rtb_Abs;
  } else {
    /* Sum: '<S13>/Sum2' incorporates:
     *  Inport: '<Root>/input_col'
     */
    rtb_DeadZone = QS_InnerRateLoop_U.input_col - QS_InnerRateLoop_B.col;

    /* DeadZone: '<S13>/Dead Zone' */
    if (rtb_DeadZone > 0.05F) {
      rtb_DeadZone -= 0.05F;
    } else if (rtb_DeadZone >= -0.05F) {
      rtb_DeadZone = 0.0F;
    } else {
      rtb_DeadZone -= -0.05F;
    }

    /* End of DeadZone: '<S13>/Dead Zone' */
    rtb_Switch_ep = -6.0F * rtb_DeadZone;
  }

  /* End of Switch: '<S108>/Switch' */

  /* Switch: '<S108>/Switch1' incorporates:
   *  RelationalOperator: '<S108>/Relational Operator'
   *  Sum: '<S108>/Sum1'
   *  UnitDelay: '<S108>/Unit Delay'
   *  UnitDelay: '<S108>/Unit Delay1'
   *  UnitDelay: '<S108>/Unit Delay2'
   */
  if (QS_InnerRateLoop_DW.UnitDelay_DSTATE_ax == rtb_Compare_m) {
    rtb_DeadZone = QS_InnerRateLoop_DW.UnitDelay1_DSTATE;
  } else {
    rtb_DeadZone = QS_InnerRateLoop_DW.UnitDelay2_DSTATE - rtb_Switch_ep;
  }

  /* End of Switch: '<S108>/Switch1' */

  /* Saturate: '<S108>/Saturation' */
  if (rtb_DeadZone > 0.0025F) {
    rtb_Sum_j3 = 0.0025F;
  } else if (rtb_DeadZone < -0.0025F) {
    rtb_Sum_j3 = -0.0025F;
  } else {
    rtb_Sum_j3 = rtb_DeadZone;
  }

  /* End of Saturate: '<S108>/Saturation' */

  /* Sum: '<S108>/Sum' */
  rtb_Sum_j3 = rtb_DeadZone - rtb_Sum_j3;

  /* Sum: '<S108>/Sum2' */
  rtb_Switch_ep += rtb_Sum_j3;

  /* Lookup_n-D: '<S42>/1-D Lookup Table2' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Product: '<S42>/Product' incorporates:
   *  Lookup_n-D: '<S42>/1-D Lookup Table2'
   *  Sum: '<S39>/Sum'
   *  UnitDelay: '<S39>/Unit Delay'
   */
  rtb_Product_of = (rtb_Switch_ep - QS_InnerRateLoop_DW.UnitDelay_DSTATE_a) /
    intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
                      QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_j);

  /* Product: '<S103>/Divide1' incorporates:
   *  Product: '<S103>/Divide7'
   */
  rtb_DeadZone = QS_InnerRateLoop_ConstB.TrigonometricFunction4_n *
    QS_InnerRateLoop_ConstB.TrigonometricFunction5_o;

  /* Sum: '<S103>/Add6' incorporates:
   *  Product: '<S103>/Divide1'
   *  Product: '<S103>/Divide11'
   *  Product: '<S103>/Divide13'
   *  Product: '<S103>/Divide15'
   *  Product: '<S103>/Divide3'
   *  Product: '<S103>/Divide7'
   *  Product: '<S103>/Divide8'
   *  Sum: '<S103>/Add1'
   *  Sum: '<S103>/Add3'
   */
  rtb_Add6_a = ((rtb_DeadZone * rtb_derivativecutofffrequency_b -
                 QS_InnerRateLoop_ConstB.TrigonometricFunction1_k * rtb_Sum3_f) *
                rtb_Sum4_b + (rtb_DeadZone * rtb_Sum3_f +
    QS_InnerRateLoop_ConstB.TrigonometricFunction1_k *
    rtb_derivativecutofffrequency_b) * rtb_Add5_ky) +
    QS_InnerRateLoop_ConstB.TrigonometricFunction4_n *
    QS_InnerRateLoop_ConstB.TrigonometricFunction2_l * rtb_Sum4_a;

  /* Gain: '<S70>/Gain1' */
  rtb_Gain1 = 0.0F * rtb_Add6_a;

  /* DiscreteIntegrator: '<S32>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevR_g <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_o = 0.0F;
  }

  rtb_DiscreteTimeIntegrator1 =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_o;

  /* Switch: '<S23>/Switch' incorporates:
   *  Constant: '<S23>/Constant'
   */
  if (rtb_Compare_m) {
    rtb_Compare_k = rtb_Gain1;
  } else {
    rtb_Compare_k = 0.0F;
  }

  /* End of Switch: '<S23>/Switch' */

  /* Sum: '<S23>/Sum3' incorporates:
   *  DiscreteIntegrator: '<S32>/Discrete-Time Integrator1'
   */
  rtb_DeadZone = rtb_Compare_k +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_o;

  /* Gain: '<S30>/derivative cutoff frequency 1' incorporates:
   *  DiscreteIntegrator: '<S30>/Discrete-Time Integrator'
   *  Gain: '<S30>/derivative cutoff frequency '
   *  Sum: '<S30>/Sum1'
   */
  rtb_derivativecutofffrequency_0 = (rtb_DeadZone -
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_b) * 100.0F;

  /* Sum: '<S103>/Add5' incorporates:
   *  Product: '<S103>/Divide'
   *  Product: '<S103>/Divide2'
   *  Product: '<S103>/Divide6'
   */
  rtb_Add5_ky = (QS_InnerRateLoop_ConstB.TrigonometricFunction2_l *
                 rtb_derivativecutofffrequency_b * rtb_Sum4_b +
                 QS_InnerRateLoop_ConstB.TrigonometricFunction2_l * rtb_Sum3_f *
                 rtb_Add5_ky) +
    -QS_InnerRateLoop_ConstB.TrigonometricFunction5_o * rtb_Sum4_a;

  /* Gain: '<S70>/Gain' */
  rtb_derivativecutofffrequency_b = 0.0F * rtb_Add5_ky;

  /* DiscreteIntegrator: '<S48>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevR_j <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_m = 0.0F;
  }

  rtb_Sum3_f = QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_m;

  /* Switch: '<S25>/Switch' incorporates:
   *  Constant: '<S25>/Constant'
   */
  if (rtb_Compare_m) {
    rtb_Compare_k = rtb_derivativecutofffrequency_b;
  } else {
    rtb_Compare_k = 0.0F;
  }

  /* End of Switch: '<S25>/Switch' */

  /* Sum: '<S25>/Sum4' incorporates:
   *  DiscreteIntegrator: '<S48>/Discrete-Time Integrator1'
   */
  rtb_Sum4_b = rtb_Compare_k +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_m;

  /* Gain: '<S46>/derivative cutoff frequency 1' incorporates:
   *  DiscreteIntegrator: '<S46>/Discrete-Time Integrator'
   *  Gain: '<S46>/derivative cutoff frequency '
   *  Sum: '<S46>/Sum1'
   */
  rtb_Compare_k = (rtb_Sum4_b -
                   QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_o) * 100.0F;

  /* Logic: '<S12>/Logical Operator2' incorporates:
   *  Constant: '<S12>/Constant7'
   *  DataTypeConversion: '<Root>/Data Type Conversion'
   *  RelationalOperator: '<S12>/Relational Operator4'
   */
  rtb_Compare_bt = (rtb_Compare_m && (rtb_Gain2 <= 18.0F));

  /* Switch: '<S12>/Switch' incorporates:
   *  Constant: '<S12>/Constant5'
   *  RelationalOperator: '<S12>/Relational Operator3'
   *  Sum: '<S12>/Sum3'
   */
  if (rtb_Gain2 >= 12.0F) {
    rtb_Sum4_a = rtb_uDLookupTable3;
  } else {
    rtb_Sum4_a = 0.0F;
  }

  /* End of Switch: '<S12>/Switch' */

  /* RateLimiter: '<S12>/Rate Limiter' */
  rtb_uDLookupTable3 = rtb_Sum4_a - QS_InnerRateLoop_DW.PrevY;
  if (rtb_uDLookupTable3 > 0.25F) {
    rtb_Sum4_a = QS_InnerRateLoop_DW.PrevY + 0.25F;
  } else {
    if (rtb_uDLookupTable3 < -0.25F) {
      rtb_Sum4_a = QS_InnerRateLoop_DW.PrevY + -0.25F;
    }
  }

  QS_InnerRateLoop_DW.PrevY = rtb_Sum4_a;

  /* End of RateLimiter: '<S12>/Rate Limiter' */

  /* Outputs for Enabled SubSystem: '<Root>/Determine Heading at Start of Manuever  All trajectories relative to this heading' incorporates:
   *  EnablePort: '<S4>/Enable'
   */
  /* Logic: '<Root>/Logical Operator2' */
  if (!rtb_Compare_m) {
    /* Gain: '<S4>/Gain' incorporates:
     *  Inport: '<Root>/psi_rad'
     */
    QS_InnerRateLoop_B.Gain = 57.2957802F * QS_InnerRateLoop_U.psi_rad;
  }

  /* End of Logic: '<Root>/Logical Operator2' */
  /* End of Outputs for SubSystem: '<Root>/Determine Heading at Start of Manuever  All trajectories relative to this heading' */

  /* Sum: '<Root>/Sum4' */
  rtb_Sum4_a += QS_InnerRateLoop_B.Gain;

  /* Gain: '<S69>/Gain' */
  rtb_uDLookupTable3 = 0.0174532924F * rtb_Sum4_a;

  /* Delay: '<S13>/Delay' */
  if (QS_InnerRateLoop_DW.icLoad != 0) {
    QS_InnerRateLoop_DW.Delay_DSTATE = rtb_uDLookupTable3;
  }

  /* Switch: '<S13>/Switch3' incorporates:
   *  Delay: '<S13>/Delay'
   *  Delay: '<S13>/Delay1'
   *  Inport: '<Root>/psi_rad'
   */
  if (QS_InnerRateLoop_DW.Delay1_DSTATE_j[0]) {
    rtb_Switch3 = QS_InnerRateLoop_DW.Delay_DSTATE;
  } else {
    rtb_Switch3 = QS_InnerRateLoop_U.psi_rad;
  }

  /* End of Switch: '<S13>/Switch3' */

  /* DiscreteIntegrator: '<S13>/Discrete-Time Integrator' */
  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOA_b != 0) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_mz = rtb_Switch3;
  }

  if ((rtb_Compare_m && (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_p <=
                         0)) || ((!rtb_Compare_m) &&
       (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_p == 1))) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_mz = rtb_Switch3;
  }

  /* Switch: '<S13>/Switch' incorporates:
   *  DiscreteIntegrator: '<S13>/Discrete-Time Integrator'
   */
  if (rtb_Compare_m) {
    rtb_Switch3 = rtb_uDLookupTable3;
  } else {
    rtb_Switch3 = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_mz;
  }

  /* End of Switch: '<S13>/Switch' */

  /* DiscreteIntegrator: '<S59>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_bp <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE = 0.0;
  }

  /* Gain: '<S59>/Gain' incorporates:
   *  DiscreteIntegrator: '<S59>/Discrete-Time Integrator'
   */
  rtb_Gain_e = 400.0 * QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE;

  /* Switch: '<S26>/Switch2' incorporates:
   *  Constant: '<S26>/Constant1'
   */
  if (rtb_Gain_e >= 100.0) {
    /* Switch: '<S26>/Switch' incorporates:
     *  Constant: '<S26>/Constant'
     *  DiscreteIntegrator: '<S58>/Discrete-Time Integrator'
     *  Gain: '<S58>/derivative cutoff frequency 1'
     *  Sum: '<S58>/Sum1'
     */
    if (rtb_Compare_bt) {
      rtb_Switch2_cz = 0.0F;
    } else {
      rtb_Switch2_cz = (rtb_Switch3 -
                        QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_mr) *
        100.0F;
    }

    /* End of Switch: '<S26>/Switch' */
  } else {
    rtb_Switch2_cz = 0.0F;
  }

  /* End of Switch: '<S26>/Switch2' */

  /* DiscreteIntegrator: '<S40>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_m <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_f = 0.0F;
  }

  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_f >= 0.1F) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_f = 0.1F;
  } else {
    if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_f <= -0.1F) {
      QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_f = -0.1F;
    }
  }

  /* Lookup_n-D: '<S37>/1-D Lookup Table2' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* DiscreteIntegrator: '<S39>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_n <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_fs = 0.0F;
  }

  /* Product: '<S37>/Product' incorporates:
   *  DiscreteIntegrator: '<S39>/Discrete-Time Integrator'
   *  Lookup_n-D: '<S37>/1-D Lookup Table2'
   *  Sum: '<S24>/Sum2'
   */
  rtb_Product_ib = (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_fs -
                    rtb_uDLookupTable_cs) * intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_o);

  /* Sum: '<S24>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S40>/Discrete-Time Integrator'
   */
  rtb_Sum1_c0 = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_f +
    rtb_Product_ib;

  /* Lookup_n-D: '<S24>/1-D Lookup Table' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_uDLookupTable_cs = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable_tableData_k);

  /* Lookup_n-D: '<S24>/1-D Lookup Table1' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_uDLookupTable1 = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable1_tableData);

  /* DiscreteIntegrator: '<S41>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_by <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_c4 = 0.0F;
  }

  rtb_DiscreteTimeIntegrator_hs =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_c4;

  /* DiscreteIntegrator: '<S33>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_e <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_o2 = 0.0F;
  }

  /* Lookup_n-D: '<S28>/1-D Lookup Table2' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Product: '<S28>/Product' incorporates:
   *  Inport: '<Root>/phi_rad'
   *  Lookup_n-D: '<S28>/1-D Lookup Table2'
   *  Sum: '<S23>/Sum'
   */
  rtb_Product_jr = (rtb_DeadZone - QS_InnerRateLoop_U.phi_rad) *
    intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
                      QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_a);

  /* Lookup_n-D: '<S27>/1-D Lookup Table2' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Sum: '<S23>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S33>/Discrete-Time Integrator'
   *  Gain: '<S30>/derivative cutoff frequency 1'
   *  Inport: '<Root>/p_rps'
   *  Lookup_n-D: '<S27>/1-D Lookup Table2'
   *  Product: '<S27>/Product'
   *  Sum: '<S23>/Sum2'
   */
  rtb_Sum1_fe = (rtb_derivativecutofffrequency_0 - QS_InnerRateLoop_U.p_rps) *
    intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
                      QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_fu) +
    (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_o2 + rtb_Product_jr);

  /* Lookup_n-D: '<S23>/1-D Lookup Table' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_uDLookupTable = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable_tableData_l);

  /* Lookup_n-D: '<S23>/1-D Lookup Table1' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_uDLookupTable1_p = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable1_tableData_n);

  /* DiscreteIntegrator: '<S34>/Discrete-Time Integrator' */
  rtb_DiscreteTimeIntegrator_cv =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_bw;

  /* DiscreteIntegrator: '<S49>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_d <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_j = 0.0F;
  }

  /* Lookup_n-D: '<S44>/1-D Lookup Table2' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Product: '<S44>/Product' incorporates:
   *  Inport: '<Root>/theta_rad'
   *  Lookup_n-D: '<S44>/1-D Lookup Table2'
   *  Sum: '<S25>/Sum'
   */
  rtb_Product_n = (rtb_Sum4_b - QS_InnerRateLoop_U.theta_rad) *
    intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
                      QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_ex);

  /* Lookup_n-D: '<S43>/1-D Lookup Table2' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Sum: '<S25>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S49>/Discrete-Time Integrator'
   *  Gain: '<S46>/derivative cutoff frequency 1'
   *  Inport: '<Root>/q_rps'
   *  Lookup_n-D: '<S43>/1-D Lookup Table2'
   *  Product: '<S43>/Product'
   *  Sum: '<S25>/Sum2'
   */
  rtb_uDLookupTable2_bw = (rtb_Compare_k - QS_InnerRateLoop_U.q_rps) *
    intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
                      QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_c) +
    (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_j + rtb_Product_n);

  /* Lookup_n-D: '<S25>/1-D Lookup Table' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_Saturation_k = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable_tableData_m);

  /* Lookup_n-D: '<S25>/1-D Lookup Table1' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_uDLookupTable1_l = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable1_tableData_d);

  /* DiscreteIntegrator: '<S50>/Discrete-Time Integrator' */
  rtb_DiscreteTimeIntegrator_nt =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_di;

  /* DiscreteIntegrator: '<S60>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_j <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_l = 0.0F;
  }

  /* Lookup_n-D: '<S54>/1-D Lookup Table2' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Switch: '<S26>/Switch1' incorporates:
   *  Constant: '<S26>/Constant'
   */
  if (rtb_Gain_e >= 100.0) {
    /* Sum: '<S26>/Sum1' incorporates:
     *  Inport: '<Root>/psi_rad'
     */
    rtb_Sum1_gw = rtb_Switch3 - QS_InnerRateLoop_U.psi_rad;

    /* Switch: '<S62>/Switch' incorporates:
     *  Constant: '<S62>/Constant1'
     *  Constant: '<S62>/Constant2'
     *  Gain: '<S62>/Gain1'
     *  Switch: '<S62>/Switch1'
     */
    if (rtb_Sum1_gw >= 0.0F) {
      rtb_Abs_k = rtb_Sum1_gw;
      iU = 1;
    } else {
      rtb_Abs_k = -rtb_Sum1_gw;
      iU = -1;
    }

    /* End of Switch: '<S62>/Switch' */

    /* Product: '<S62>/Product' incorporates:
     *  Constant: '<S62>/Constant'
     *  Gain: '<S62>/Gain'
     *  Product: '<S62>/Divide'
     *  Rounding: '<S62>/Rounding Function'
     *  Sum: '<S62>/Subtract'
     */
    rtb_Product_cw = (rtb_Abs_k - static_cast<real32_T>(floor((real_T)(rtb_Abs_k
      * 0.159154937F))) * 6.28318548F) * static_cast<real32_T>(iU);

    /* Switch: '<S53>/Switch' incorporates:
     *  Abs: '<S53>/Abs'
     */
    if (static_cast<real32_T>(fabs((real_T)rtb_Product_cw)) > 3.14159274F) {
      /* Switch: '<S53>/Switch1' incorporates:
       *  Constant: '<S53>/Constant1'
       *  Constant: '<S53>/Constant2'
       *  Sum: '<S53>/Add'
       *  Sum: '<S53>/Subtract'
       */
      if (rtb_Product_cw >= 0.0F) {
        rtb_Product_cw -= 6.28318548F;
      } else {
        rtb_Product_cw += 6.28318548F;
      }

      /* End of Switch: '<S53>/Switch1' */
    }

    /* End of Switch: '<S53>/Switch' */
  } else {
    rtb_Product_cw = 0.0F;
  }

  /* End of Switch: '<S26>/Switch1' */

  /* Product: '<S54>/Product' incorporates:
   *  Lookup_n-D: '<S54>/1-D Lookup Table2'
   */
  rtb_Product_ear = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_au) * rtb_Product_cw;

  /* Lookup_n-D: '<S56>/1-D Lookup Table2' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Sum: '<S26>/Sum5' incorporates:
   *  DiscreteIntegrator: '<S60>/Discrete-Time Integrator'
   *  Inport: '<Root>/r_rps'
   *  Lookup_n-D: '<S56>/1-D Lookup Table2'
   *  Product: '<S56>/Product'
   *  Sum: '<S26>/Sum3'
   */
  rtb_Product_cw = (rtb_Switch2_cz - QS_InnerRateLoop_U.r_rps) *
    intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
                      QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_c0) +
    (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_l + rtb_Product_ear);

  /* Lookup_n-D: '<S26>/1-D Lookup Table' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_Sum1_gw = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable_tableData_h);

  /* Lookup_n-D: '<S26>/1-D Lookup Table1' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_Abs_k = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable1_tableData_f);

  /* DiscreteIntegrator: '<S61>/Discrete-Time Integrator' */
  rtb_DiscreteTimeIntegrator_ia =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_hv;

  /* Sum: '<S5>/Sum' */
  rtb_Product_hg[0] = rtb_Product_of;

  /* Gain: '<S31>/derivative cutoff frequency 1' incorporates:
   *  DiscreteIntegrator: '<S31>/Discrete-Time Integrator'
   *  Gain: '<S30>/derivative cutoff frequency 1'
   *  Gain: '<S31>/derivative cutoff frequency '
   *  Sum: '<S31>/Sum1'
   */
  rtb_Product_b_tmp = (rtb_derivativecutofffrequency_0 -
                       QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_hq) *
    100.0F;

  /* Sum: '<S5>/Sum' incorporates:
   *  Gain: '<S31>/derivative cutoff frequency 1'
   */
  rtb_Product_hg[1] = rtb_Product_b_tmp;

  /* Sum: '<S25>/Sum3' incorporates:
   *  DiscreteIntegrator: '<S47>/Discrete-Time Integrator'
   *  Gain: '<S46>/derivative cutoff frequency 1'
   *  Gain: '<S47>/derivative cutoff frequency '
   *  Gain: '<S47>/derivative cutoff frequency 1'
   *  Sum: '<S47>/Sum1'
   */
  rtb_Product_b_tmp_0 = (rtb_Compare_k -
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_mk) * 100.0F;

  /* Sum: '<S5>/Sum' incorporates:
   *  Gain: '<S47>/derivative cutoff frequency 1'
   *  Sum: '<S25>/Sum3'
   */
  rtb_Product_hg[2] = rtb_Product_b_tmp_0;

  /* Gain: '<S57>/derivative cutoff frequency 1' incorporates:
   *  DiscreteIntegrator: '<S57>/Discrete-Time Integrator'
   *  Gain: '<S57>/derivative cutoff frequency '
   *  Sum: '<S57>/Sum1'
   */
  rtb_Switch2_cz = (rtb_Switch2_cz -
                    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_d) *
    100.0F;

  /* Sum: '<S5>/Sum' incorporates:
   *  DiscreteIntegrator: '<S34>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S41>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S50>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S61>/Discrete-Time Integrator'
   *  Gain: '<S57>/derivative cutoff frequency 1'
   *  Product: '<S34>/Product1'
   *  Product: '<S41>/Product1'
   *  Product: '<S50>/Product1'
   *  Product: '<S61>/Product1'
   *  Product: '<S64>/Product'
   *  Sum: '<S34>/Sum1'
   *  Sum: '<S41>/Sum1'
   *  Sum: '<S50>/Sum1'
   *  Sum: '<S61>/Sum1'
   */
  rtb_Product_hg[3] = rtb_Switch2_cz;
  rtb_Sum1_f[0] = rtb_Sum1_c0 * rtb_uDLookupTable_cs / rtb_uDLookupTable1 +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_c4;
  rtb_Sum1_f[1] = rtb_Sum1_fe * rtb_uDLookupTable / rtb_uDLookupTable1_p +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_bw;
  rtb_Sum1_f[2] = rtb_uDLookupTable2_bw * rtb_Saturation_k /
    rtb_uDLookupTable1_l + QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_di;
  rtb_Sum1_f[3] = rtb_Product_cw * rtb_Sum1_gw / rtb_Abs_k +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_hv;
  for (iU = 0; iU < 4; iU++) {
    tmp = 0.0F;
    tmp_0 = 0;
    for (i = 0; i < 9; i++) {
      tmp += rtb_uDLookupTable2[tmp_0 + iU] * rtb_TmpSignalConversionAtProduc[i];
      tmp_0 += 4;
    }

    rtb_Product_b[iU] = (rtb_Product_hg[iU] + rtb_Sum1_f[iU]) - tmp;
  }

  /* Product: '<S63>/Product' */
  for (iU = 0; iU < 4; iU++) {
    rtb_Product_hg[iU] = 0.0F;
    rtb_Product_hg[iU] += rtb_uDLookupTable2_k5[iU] * rtb_Product_b[0];
    rtb_Product_hg[iU] += rtb_uDLookupTable2_k5[iU + 4] * rtb_Product_b[1];
    rtb_Product_hg[iU] += rtb_uDLookupTable2_k5[iU + 8] * rtb_Product_b[2];
    rtb_Product_hg[iU] += rtb_uDLookupTable2_k5[iU + 12] * rtb_Product_b[3];
  }

  /* End of Product: '<S63>/Product' */

  /* Outputs for Enabled SubSystem: '<Root>/Enabled Subsystem Grab and Freeze Value Upon Engagement' incorporates:
   *  EnablePort: '<S6>/Enable'
   */
  /* Logic: '<Root>/Logical Operator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (!QS_InnerRateLoop_U.engage) {
    /* Saturate: '<Root>/Saturation' incorporates:
     *  Inport: '<Root>/mixer_in_throttle'
     *  Inport: '<S6>/In1'
     */
    if (QS_InnerRateLoop_U.mixer_in_throttle > 2.0F) {
      QS_InnerRateLoop_B.In1[0] = 2.0F;
    } else if (QS_InnerRateLoop_U.mixer_in_throttle < -1.0F) {
      QS_InnerRateLoop_B.In1[0] = -1.0F;
    } else {
      QS_InnerRateLoop_B.In1[0] = QS_InnerRateLoop_U.mixer_in_throttle;
    }

    /* End of Saturate: '<Root>/Saturation' */

    /* Saturate: '<Root>/Saturation1' incorporates:
     *  Inport: '<Root>/mixer_in_y'
     *  Inport: '<S6>/In1'
     */
    if (QS_InnerRateLoop_U.mixer_in_y > 2.0F) {
      QS_InnerRateLoop_B.In1[1] = 2.0F;
    } else if (QS_InnerRateLoop_U.mixer_in_y < -2.0F) {
      QS_InnerRateLoop_B.In1[1] = -2.0F;
    } else {
      QS_InnerRateLoop_B.In1[1] = QS_InnerRateLoop_U.mixer_in_y;
    }

    /* End of Saturate: '<Root>/Saturation1' */

    /* Saturate: '<Root>/Saturation2' incorporates:
     *  Inport: '<Root>/mixer_in_x'
     *  Inport: '<S6>/In1'
     */
    if (QS_InnerRateLoop_U.mixer_in_x > 2.0F) {
      QS_InnerRateLoop_B.In1[2] = 2.0F;
    } else if (QS_InnerRateLoop_U.mixer_in_x < -2.0F) {
      QS_InnerRateLoop_B.In1[2] = -2.0F;
    } else {
      QS_InnerRateLoop_B.In1[2] = QS_InnerRateLoop_U.mixer_in_x;
    }

    /* End of Saturate: '<Root>/Saturation2' */

    /* Saturate: '<Root>/Saturation3' incorporates:
     *  Inport: '<Root>/mixer_in_z'
     *  Inport: '<S6>/In1'
     */
    if (QS_InnerRateLoop_U.mixer_in_z > 2.0F) {
      QS_InnerRateLoop_B.In1[3] = 2.0F;
    } else if (QS_InnerRateLoop_U.mixer_in_z < -2.0F) {
      QS_InnerRateLoop_B.In1[3] = -2.0F;
    } else {
      QS_InnerRateLoop_B.In1[3] = QS_InnerRateLoop_U.mixer_in_z;
    }

    /* End of Saturate: '<Root>/Saturation3' */
  }

  /* End of Logic: '<Root>/Logical Operator' */
  /* End of Outputs for SubSystem: '<Root>/Enabled Subsystem Grab and Freeze Value Upon Engagement' */

  /* DiscreteIntegrator: '<S65>/Discrete-Time Integrator' */
  rtb_DiscreteTimeIntegrator_fi =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_co;

  /* Sum: '<S65>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S65>/Discrete-Time Integrator'
   *  Gain: '<S65>/Gain'
   */
  rtb_DeadZone3 = 6.66666651F * rtb_Product_hg[0] +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_co;

  /* DiscreteIntegrator: '<S66>/Discrete-Time Integrator' */
  rtb_DiscreteTimeIntegrator_k1 =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_i;

  /* Sum: '<S66>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S66>/Discrete-Time Integrator'
   *  Gain: '<S66>/Gain'
   */
  rtb_uDLookupTable_a = 6.66666651F * rtb_Product_hg[1] +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_i;

  /* DiscreteIntegrator: '<S67>/Discrete-Time Integrator' */
  rtb_DiscreteTimeIntegrator_h =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_ch;

  /* Sum: '<S67>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S67>/Discrete-Time Integrator'
   *  Gain: '<S67>/Gain'
   */
  rtb_Sum1_lq = 6.66666651F * rtb_Product_hg[2] +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_ch;

  /* DiscreteIntegrator: '<S68>/Discrete-Time Integrator' */
  rtb_DiscreteTimeIntegrator_n =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_a5;

  /* Sum: '<S68>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S68>/Discrete-Time Integrator'
   *  Gain: '<S68>/Gain'
   */
  rtb_Sum1_gg = 0.364F * rtb_Product_hg[3] +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_a5;

  /* Sum: '<Root>/Sum1' */
  rtb_Sum1_j_idx_0 = QS_InnerRateLoop_B.In1[0] + rtb_DeadZone3;
  rtb_Sum1_j_idx_1 = QS_InnerRateLoop_B.In1[1] + rtb_uDLookupTable_a;
  rtb_Sum1_j_idx_2 = QS_InnerRateLoop_B.In1[2] + rtb_Sum1_lq;
  rtb_TrigonometricFunction3 = QS_InnerRateLoop_B.In1[3] + rtb_Sum1_gg;

  /* Saturate: '<Root>/Saturation4' */
  if (rtb_Sum1_j_idx_0 > 0.9F) {
    rtb_Abs_i = 0.9F;
  } else if (rtb_Sum1_j_idx_0 < 0.05F) {
    rtb_Abs_i = 0.05F;
  } else {
    rtb_Abs_i = rtb_Sum1_j_idx_0;
  }

  /* End of Saturate: '<Root>/Saturation4' */

  /* Outport: '<Root>/mixer_throttle' */
  QS_InnerRateLoop_Y.mixer_throttle = rtb_Abs_i;

  /* Sum: '<S14>/Sum' */
  rtb_Abs_i = rtb_Sum1_j_idx_0 - rtb_Abs_i;

  /* Abs: '<S14>/Abs' */
  rtb_Abs_i = static_cast<real32_T>(fabs((real_T)rtb_Abs_i));

  /* Switch: '<S40>/Switch' incorporates:
   *  Constant: '<S115>/Constant'
   *  Constant: '<S40>/Constant'
   *  Lookup_n-D: '<S38>/1-D Lookup Table2'
   *  Product: '<S38>/Product'
   *  RelationalOperator: '<S115>/Compare'
   */
  if (rtb_Abs_i > 0.0F) {
    rtb_Product_ib = 0.0F;
  } else {
    /* Lookup_n-D: '<S38>/1-D Lookup Table2' */
    bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
    rtb_Product_ib *= intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
      QS_InnerRateLoop_ConstP.pooled8);
  }

  /* End of Switch: '<S40>/Switch' */

  /* Saturate: '<Root>/Saturation7' */
  if (rtb_Sum1_j_idx_1 > 1.0F) {
    rtb_Abs_i = 1.0F;
  } else if (rtb_Sum1_j_idx_1 < -1.0F) {
    rtb_Abs_i = -1.0F;
  } else {
    rtb_Abs_i = rtb_Sum1_j_idx_1;
  }

  /* End of Saturate: '<Root>/Saturation7' */

  /* Outport: '<Root>/mixer_x' */
  QS_InnerRateLoop_Y.mixer_x = rtb_Abs_i;

  /* Sum: '<S15>/Sum' */
  rtb_Abs_i = rtb_Sum1_j_idx_1 - rtb_Abs_i;

  /* Abs: '<S15>/Abs' */
  rtb_Abs_i = static_cast<real32_T>(fabs((real_T)rtb_Abs_i));

  /* Switch: '<S33>/Switch' incorporates:
   *  Constant: '<S116>/Constant'
   *  Constant: '<S33>/Constant'
   *  Lookup_n-D: '<S29>/1-D Lookup Table2'
   *  Product: '<S29>/Product'
   *  RelationalOperator: '<S116>/Compare'
   */
  if (rtb_Abs_i > 0.0F) {
    rtb_Product_jr = 0.0F;
  } else {
    /* Lookup_n-D: '<S29>/1-D Lookup Table2' */
    bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
    rtb_Product_jr *= intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
      QS_InnerRateLoop_ConstP.pooled6);
  }

  /* End of Switch: '<S33>/Switch' */

  /* Saturate: '<Root>/Saturation6' */
  if (rtb_Sum1_j_idx_2 > 1.0F) {
    rtb_Abs_i = 1.0F;
  } else if (rtb_Sum1_j_idx_2 < -1.0F) {
    rtb_Abs_i = -1.0F;
  } else {
    rtb_Abs_i = rtb_Sum1_j_idx_2;
  }

  /* End of Saturate: '<Root>/Saturation6' */

  /* Outport: '<Root>/mixer_y' */
  QS_InnerRateLoop_Y.mixer_y = rtb_Abs_i;

  /* Sum: '<S16>/Sum' */
  rtb_Abs_i = rtb_Sum1_j_idx_2 - rtb_Abs_i;

  /* Abs: '<S16>/Abs' */
  rtb_Abs_i = static_cast<real32_T>(fabs((real_T)rtb_Abs_i));

  /* Switch: '<S49>/Switch' incorporates:
   *  Constant: '<S117>/Constant'
   *  Constant: '<S49>/Constant'
   *  Lookup_n-D: '<S45>/1-D Lookup Table2'
   *  Product: '<S45>/Product'
   *  RelationalOperator: '<S117>/Compare'
   */
  if (rtb_Abs_i > 0.0F) {
    rtb_Product_n = 0.0F;
  } else {
    /* Lookup_n-D: '<S45>/1-D Lookup Table2' */
    bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
    rtb_Product_n *= intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
      QS_InnerRateLoop_ConstP.uDLookupTable2_tableData);
  }

  /* End of Switch: '<S49>/Switch' */

  /* Saturate: '<Root>/Saturation5' */
  if (rtb_TrigonometricFunction3 > 1.0F) {
    rtb_Abs_i = 1.0F;
  } else if (rtb_TrigonometricFunction3 < -1.0F) {
    rtb_Abs_i = -1.0F;
  } else {
    rtb_Abs_i = rtb_TrigonometricFunction3;
  }

  /* End of Saturate: '<Root>/Saturation5' */

  /* Outport: '<Root>/mixer_z' */
  QS_InnerRateLoop_Y.mixer_z = rtb_Abs_i;

  /* Sum: '<S17>/Sum' */
  rtb_Abs_i -= rtb_TrigonometricFunction3;

  /* Abs: '<S17>/Abs' */
  rtb_Abs_i = static_cast<real32_T>(fabs((real_T)rtb_Abs_i));

  /* Switch: '<S60>/Switch' incorporates:
   *  Constant: '<S118>/Constant'
   *  Constant: '<S60>/Constant'
   *  Lookup_n-D: '<S55>/1-D Lookup Table2'
   *  Product: '<S55>/Product'
   *  RelationalOperator: '<S118>/Compare'
   */
  if (rtb_Abs_i > 0.0F) {
    rtb_Product_ear = 0.0F;
  } else {
    /* Lookup_n-D: '<S55>/1-D Lookup Table2' */
    bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
    rtb_Product_ear *= intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
      QS_InnerRateLoop_ConstP.pooled6);
  }

  /* End of Switch: '<S60>/Switch' */

  /* Outport: '<Root>/yaw_sweep' */
  QS_InnerRateLoop_Y.yaw_sweep = rtb_Sum1_gg;

  /* Outport: '<Root>/pitch_sweep' */
  QS_InnerRateLoop_Y.pitch_sweep = rtb_Sum1_lq;

  /* Outport: '<Root>/roll_sweep' */
  QS_InnerRateLoop_Y.roll_sweep = rtb_uDLookupTable_a;

  /* Outport: '<Root>/coll_sweep' */
  QS_InnerRateLoop_Y.coll_sweep = rtb_DeadZone3;

  /* Product: '<S61>/Product' incorporates:
   *  Product: '<S61>/Product2'
   *  Sum: '<S61>/Sum'
   *  Sum: '<S61>/Sum2'
   *  UnitDelay: '<S61>/Unit Delay'
   */
  rtb_Sum1_lq = ((rtb_Abs_k - rtb_Sum1_gw) * rtb_Product_cw / rtb_Abs_k -
                 QS_InnerRateLoop_DW.UnitDelay_DSTATE_o) * rtb_Sum1_gw;

  /* Product: '<S50>/Product2' incorporates:
   *  Sum: '<S50>/Sum2'
   */
  rtb_Sum1_gg = (rtb_uDLookupTable1_l - rtb_Saturation_k) *
    rtb_uDLookupTable2_bw / rtb_uDLookupTable1_l;

  /* Outport: '<Root>/vz_cmd' incorporates:
   *  DiscreteIntegrator: '<S39>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_Y.vz_cmd =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_fs;

  /* Switch: '<S59>/Switch' incorporates:
   *  Constant: '<S59>/Constant'
   *  Constant: '<S59>/Constant1'
   *  Inport: '<Root>/engage'
   *  RelationalOperator: '<S59>/Relational Operator'
   */
  if (rtb_Gain_e >= 100.0) {
    rtb_Gain_e = 0.0;
  } else {
    rtb_Gain_e = QS_InnerRateLoop_U.engage;
  }

  /* End of Switch: '<S59>/Switch' */

  /* Outport: '<Root>/psi_cmd' */
  QS_InnerRateLoop_Y.psi_cmd = rtb_Switch3;

  /* Trigonometry: '<S71>/Trigonometric Function3' */
  rtb_DeadZone3 = static_cast<real32_T>(cos((real_T)rtb_uDLookupTable3));

  /* Trigonometry: '<S71>/Trigonometric Function6' */
  rtb_Abs_k = static_cast<real32_T>(sin((real_T)rtb_uDLookupTable3));

  /* Product: '<S71>/Divide4' incorporates:
   *  Product: '<S71>/Divide9'
   */
  rtb_Abs_i = QS_InnerRateLoop_ConstB.TrigonometricFunction1_m *
    QS_InnerRateLoop_ConstB.TrigonometricFunction5_f;

  /* Product: '<S71>/Divide1' incorporates:
   *  Product: '<S71>/Divide7'
   */
  rtb_Product_cw = QS_InnerRateLoop_ConstB.TrigonometricFunction4_km *
    QS_InnerRateLoop_ConstB.TrigonometricFunction5_f;

  /* Sum: '<S71>/Add5' incorporates:
   *  Product: '<S71>/Divide'
   *  Product: '<S71>/Divide1'
   *  Product: '<S71>/Divide13'
   *  Product: '<S71>/Divide14'
   *  Product: '<S71>/Divide3'
   *  Product: '<S71>/Divide4'
   *  Product: '<S71>/Divide5'
   *  Sum: '<S71>/Add1'
   *  Sum: '<S71>/Add2'
   */
  rtb_Sum1_j_idx_0 = ((rtb_Abs_i * rtb_DeadZone3 +
                       QS_InnerRateLoop_ConstB.TrigonometricFunction4_km *
                       rtb_Abs_k) * rtb_Add7_d + (rtb_Product_cw * rtb_DeadZone3
    - QS_InnerRateLoop_ConstB.TrigonometricFunction1_m * rtb_Abs_k) * rtb_Add6_a)
    + QS_InnerRateLoop_ConstB.TrigonometricFunction2_ag * rtb_DeadZone3 *
    rtb_Add5_ky;

  /* Outport: '<Root>/Vnorth_cmd' */
  QS_InnerRateLoop_Y.Vnorth_cmd = rtb_Sum1_j_idx_0;

  /* Sum: '<S71>/Add6' incorporates:
   *  Product: '<S71>/Divide10'
   *  Product: '<S71>/Divide15'
   *  Product: '<S71>/Divide16'
   *  Product: '<S71>/Divide6'
   *  Product: '<S71>/Divide7'
   *  Product: '<S71>/Divide8'
   *  Product: '<S71>/Divide9'
   *  Sum: '<S71>/Add3'
   *  Sum: '<S71>/Add4'
   */
  rtb_Sum1_j_idx_1 = ((rtb_Abs_i * rtb_Abs_k -
                       QS_InnerRateLoop_ConstB.TrigonometricFunction4_km *
                       rtb_DeadZone3) * rtb_Add7_d + (rtb_Product_cw * rtb_Abs_k
    + QS_InnerRateLoop_ConstB.TrigonometricFunction1_m * rtb_DeadZone3) *
                      rtb_Add6_a) +
    QS_InnerRateLoop_ConstB.TrigonometricFunction2_ag * rtb_Abs_k * rtb_Add5_ky;

  /* Outport: '<Root>/Veast_cmd' */
  QS_InnerRateLoop_Y.Veast_cmd = rtb_Sum1_j_idx_1;

  /* Sum: '<S71>/Add7' incorporates:
   *  Product: '<S71>/Divide11'
   *  Product: '<S71>/Divide12'
   *  Product: '<S71>/Divide2'
   */
  rtb_Add7_d = (QS_InnerRateLoop_ConstB.TrigonometricFunction1_m *
                QS_InnerRateLoop_ConstB.TrigonometricFunction2_ag * rtb_Add7_d +
                QS_InnerRateLoop_ConstB.TrigonometricFunction4_km *
                QS_InnerRateLoop_ConstB.TrigonometricFunction2_ag * rtb_Add6_a)
    + -QS_InnerRateLoop_ConstB.TrigonometricFunction5_f * rtb_Add5_ky;

  /* Outport: '<Root>/Vdown_cmd' */
  QS_InnerRateLoop_Y.Vdown_cmd = rtb_Add7_d;

  /* Outport: '<Root>/vehheadingcmd' */
  QS_InnerRateLoop_Y.vehheadingcmd = rtb_Sum4_a;

  /* DiscreteIntegrator: '<S84>/Discrete-Time Integrator' */
  if (rtb_Compare_m && (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_h <= 0))
  {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_n = 0.0F;
  }

  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_n >= 0.0174520072F) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_n = 0.0174520072F;
  } else {
    if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_n <= -0.0174520072F) {
      QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_n = -0.0174520072F;
    }
  }

  /* Lookup_n-D: '<S82>/1-D Lookup Table2' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Product: '<S82>/Product' incorporates:
   *  Lookup_n-D: '<S82>/1-D Lookup Table2'
   *  Product: '<S72>/Divide'
   *  Product: '<S72>/Divide2'
   *  Product: '<S72>/Divide6'
   *  Sum: '<S72>/Add5'
   *  Sum: '<S74>/Sum'
   *  Sum: '<S74>/Sum4'
   */
  rtb_uDLookupTable1_l = ((((QS_InnerRateLoop_ConstB.TrigonometricFunction2_a *
    rtb_TrigonometricFunction6 * rtb_Saturation_h +
    QS_InnerRateLoop_ConstB.TrigonometricFunction2_a * rtb_uDLookupTable_l_tmp *
    rtb_Saturation1_p) + -QS_InnerRateLoop_ConstB.TrigonometricFunction5_j *
    rtb_Saturation2_h) + rtb_Add5_ky) - rtb_Add5_e) * intrp1d_fu32fl_pw(bpIdx,
    rtb_Abs_i, QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_kg);

  /* Sum: '<S74>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S84>/Discrete-Time Integrator'
   */
  rtb_DeadZone3 = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_n +
    rtb_uDLookupTable1_l;

  /* Lookup_n-D: '<S74>/1-D Lookup Table' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_Abs_k = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable_tableData_d);

  /* Lookup_n-D: '<S74>/1-D Lookup Table1' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_Product_cw = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable1_tableData_m);

  /* DiscreteIntegrator: '<S85>/Discrete-Time Integrator' */
  rtb_Add5_e = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_g;

  /* Gain: '<S74>/Gain5' incorporates:
   *  DiscreteIntegrator: '<S85>/Discrete-Time Integrator'
   *  Product: '<S85>/Product1'
   *  Sum: '<S85>/Sum1'
   */
  rtb_Abs_i = -(rtb_DeadZone3 * rtb_Abs_k / rtb_Product_cw +
                QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_g);

  /* Sum: '<S13>/Sum' */
  rtb_Sum1_gw = rtb_Abs_i + rtb_derivativecutofffrequency_b;

  /* Saturate: '<S13>/Saturation4' */
  if (rtb_Sum1_gw > 0.610820234F) {
    rtb_Sum1_j_idx_2 = 0.610820234F;
  } else if (rtb_Sum1_gw < -0.610820234F) {
    rtb_Sum1_j_idx_2 = -0.610820234F;
  } else {
    rtb_Sum1_j_idx_2 = rtb_Sum1_gw;
  }

  /* End of Saturate: '<S13>/Saturation4' */

  /* Switch: '<S106>/Switch' incorporates:
   *  DeadZone: '<S13>/Dead Zone2'
   *  Gain: '<S13>/thetacmd'
   *  Inport: '<Root>/input_lon'
   */
  if (rtb_Compare_m) {
    /* Switch: '<S13>/Switch2' incorporates:
     *  Sum: '<S13>/Sum1'
     */
    if (rtb_Compare_bt) {
      /* Saturate: '<S13>/Saturation5' */
      if (rtb_Abs_i > 0.261780113F) {
        rtb_Abs_i = 0.261780113F;
      } else {
        if (rtb_Abs_i < -0.261780113F) {
          rtb_Abs_i = -0.261780113F;
        }
      }

      /* End of Saturate: '<S13>/Saturation5' */
    } else {
      rtb_Abs_i = rtb_Sum1_j_idx_2 - rtb_derivativecutofffrequency_b;
    }

    /* End of Switch: '<S13>/Switch2' */
  } else {
    if (QS_InnerRateLoop_U.input_lon > 0.05F) {
      /* DeadZone: '<S13>/Dead Zone2' incorporates:
       *  Inport: '<Root>/input_lon'
       */
      tmp = QS_InnerRateLoop_U.input_lon - 0.05F;
    } else if (QS_InnerRateLoop_U.input_lon >= -0.05F) {
      /* DeadZone: '<S13>/Dead Zone2' */
      tmp = 0.0F;
    } else {
      /* DeadZone: '<S13>/Dead Zone2' incorporates:
       *  Inport: '<Root>/input_lon'
       */
      tmp = QS_InnerRateLoop_U.input_lon - -0.05F;
    }

    rtb_Abs_i = 0.785340309F * tmp;
  }

  /* End of Switch: '<S106>/Switch' */

  /* Switch: '<S106>/Switch1' incorporates:
   *  RelationalOperator: '<S106>/Relational Operator'
   *  Sum: '<S106>/Sum1'
   *  UnitDelay: '<S106>/Unit Delay'
   *  UnitDelay: '<S106>/Unit Delay1'
   *  UnitDelay: '<S106>/Unit Delay2'
   */
  if (QS_InnerRateLoop_DW.UnitDelay_DSTATE_e2 == rtb_Compare_m) {
    rtb_uDLookupTable_a = QS_InnerRateLoop_DW.UnitDelay1_DSTATE_f;
  } else {
    rtb_uDLookupTable_a = QS_InnerRateLoop_DW.UnitDelay2_DSTATE_k - rtb_Abs_i;
  }

  /* End of Switch: '<S106>/Switch1' */

  /* Saturate: '<S106>/Saturation' */
  if (rtb_uDLookupTable_a > 0.0025F) {
    rtb_Sum4_a = 0.0025F;
  } else if (rtb_uDLookupTable_a < -0.0025F) {
    rtb_Sum4_a = -0.0025F;
  } else {
    rtb_Sum4_a = rtb_uDLookupTable_a;
  }

  /* End of Saturate: '<S106>/Saturation' */

  /* Sum: '<S106>/Sum' */
  rtb_derivativecutofffrequency_b = rtb_uDLookupTable_a - rtb_Sum4_a;

  /* Sum: '<S106>/Sum2' */
  rtb_Sum4_a = rtb_Abs_i + rtb_derivativecutofffrequency_b;

  /* Lookup_n-D: '<S51>/1-D Lookup Table2' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Lookup_n-D: '<S52>/1-D Lookup Table2' */
  bpIdx_0 = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U,
    &rtb_uDLookupTable2_bw);

  /* Product: '<S52>/Product' incorporates:
   *  Lookup_n-D: '<S51>/1-D Lookup Table2'
   *  Lookup_n-D: '<S52>/1-D Lookup Table2'
   *  Product: '<S51>/Product'
   *  Sum: '<S48>/Sum'
   *  Sum: '<S48>/Sum1'
   *  UnitDelay: '<S48>/Unit Delay'
   *  UnitDelay: '<S48>/Unit Delay1'
   */
  rtb_Add5_ky = ((rtb_Sum4_a - QS_InnerRateLoop_DW.UnitDelay1_DSTATE_c) *
                 intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_or) -
                 QS_InnerRateLoop_DW.UnitDelay_DSTATE_j) * intrp1d_fu32fl_pw
    (bpIdx_0, rtb_uDLookupTable2_bw,
     QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_gp);

  /* Sum: '<S111>/Sum' */
  rtb_Sum1_gw = rtb_Sum1_j_idx_2 - rtb_Sum1_gw;

  /* Abs: '<S111>/Abs' */
  rtb_Sum1_gw = static_cast<real32_T>(fabs((real_T)rtb_Sum1_gw));

  /* RelationalOperator: '<S114>/Compare' incorporates:
   *  Constant: '<S114>/Constant'
   */
  rtb_Compare_nh = (rtb_Sum1_gw > 0.0F);

  /* Switch: '<S84>/Switch' incorporates:
   *  Constant: '<S84>/Constant'
   *  Lookup_n-D: '<S83>/1-D Lookup Table2'
   *  Product: '<S83>/Product'
   */
  if (rtb_Compare_nh) {
    rtb_Sum1_j_idx_2 = 0.0F;
  } else {
    /* Lookup_n-D: '<S83>/1-D Lookup Table2' */
    bpIdx = plook_u32ff_evenxg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
    rtb_Sum1_j_idx_2 = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
      QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_f) * rtb_uDLookupTable1_l;
  }

  /* End of Switch: '<S84>/Switch' */

  /* DiscreteIntegrator: '<S80>/Discrete-Time Integrator' */
  if (rtb_Compare_m && (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_dc <= 0))
  {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cf = 0.0F;
  }

  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cf >= 0.0174520072F) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cf = 0.0174520072F;
  } else {
    if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cf <= -0.0174520072F) {
      QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cf = -0.0174520072F;
    }
  }

  /* Lookup_n-D: '<S78>/1-D Lookup Table2' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Product: '<S72>/Divide1' incorporates:
   *  Product: '<S72>/Divide7'
   */
  rtb_uDLookupTable1_l = QS_InnerRateLoop_ConstB.TrigonometricFunction4_k *
    QS_InnerRateLoop_ConstB.TrigonometricFunction5_j;

  /* Product: '<S78>/Product' incorporates:
   *  Lookup_n-D: '<S78>/1-D Lookup Table2'
   *  Product: '<S72>/Divide1'
   *  Product: '<S72>/Divide11'
   *  Product: '<S72>/Divide13'
   *  Product: '<S72>/Divide15'
   *  Product: '<S72>/Divide3'
   *  Product: '<S72>/Divide7'
   *  Product: '<S72>/Divide8'
   *  Sum: '<S72>/Add1'
   *  Sum: '<S72>/Add3'
   *  Sum: '<S72>/Add6'
   *  Sum: '<S73>/Sum'
   *  Sum: '<S73>/Sum4'
   */
  rtb_TrigonometricFunction6 = (((((rtb_uDLookupTable1_l *
    rtb_TrigonometricFunction6 -
    QS_InnerRateLoop_ConstB.TrigonometricFunction1_g * rtb_uDLookupTable_l_tmp) *
    rtb_Saturation_h + (rtb_uDLookupTable1_l * rtb_uDLookupTable_l_tmp +
                        QS_InnerRateLoop_ConstB.TrigonometricFunction1_g *
                        rtb_TrigonometricFunction6) * rtb_Saturation1_p) +
    QS_InnerRateLoop_ConstB.TrigonometricFunction4_k *
    QS_InnerRateLoop_ConstB.TrigonometricFunction2_a * rtb_Saturation2_h) +
    rtb_Add6_a) - rtb_Add6_i) * intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_n);

  /* Sum: '<S73>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S80>/Discrete-Time Integrator'
   */
  rtb_Sum1_gw = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cf +
    rtb_TrigonometricFunction6;

  /* Lookup_n-D: '<S73>/1-D Lookup Table' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_uDLookupTable_a = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable_tableData_hs);

  /* Lookup_n-D: '<S73>/1-D Lookup Table1' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_uDLookupTable1_l = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable1_tableData_p);

  /* DiscreteIntegrator: '<S81>/Discrete-Time Integrator' */
  rtb_Add6_i = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cc;

  /* Sum: '<S81>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S81>/Discrete-Time Integrator'
   *  Product: '<S81>/Product1'
   */
  rtb_TrigonometricFunction3 = rtb_Sum1_gw * rtb_uDLookupTable_a /
    rtb_uDLookupTable1_l + QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cc;

  /* Sum: '<S13>/Sum3' */
  rtb_uDLookupTable2_bw = rtb_TrigonometricFunction3 + rtb_Gain1;

  /* Saturate: '<S13>/Saturation1' */
  if (rtb_uDLookupTable2_bw > 0.610820234F) {
    rtb_Add6_a = 0.610820234F;
  } else if (rtb_uDLookupTable2_bw < -0.610820234F) {
    rtb_Add6_a = -0.610820234F;
  } else {
    rtb_Add6_a = rtb_uDLookupTable2_bw;
  }

  /* End of Saturate: '<S13>/Saturation1' */

  /* Sum: '<S109>/Sum' */
  rtb_uDLookupTable2_bw = rtb_Add6_a - rtb_uDLookupTable2_bw;

  /* Abs: '<S109>/Abs' */
  rtb_uDLookupTable2_bw = static_cast<real32_T>(fabs((real_T)
    rtb_uDLookupTable2_bw));

  /* RelationalOperator: '<S112>/Compare' incorporates:
   *  Constant: '<S112>/Constant'
   */
  rtb_RelationalOperator_b1 = (rtb_uDLookupTable2_bw > 0.0F);

  /* Switch: '<S91>/Switch' incorporates:
   *  Constant: '<S91>/Constant'
   *  Logic: '<S76>/Logical Operator'
   *  Lookup_n-D: '<S90>/1-D Lookup Table2'
   *  Product: '<S90>/Product'
   */
  if (rtb_Compare_nh || rtb_RelationalOperator_b1) {
    rtb_Product_ng = 0.0F;
  } else {
    /* Lookup_n-D: '<S90>/1-D Lookup Table2' */
    bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
    rtb_Product_ng *= intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
      QS_InnerRateLoop_ConstP.pooled13);
  }

  /* End of Switch: '<S91>/Switch' */

  /* Switch: '<S80>/Switch' incorporates:
   *  Constant: '<S80>/Constant'
   *  Lookup_n-D: '<S79>/1-D Lookup Table2'
   *  Product: '<S79>/Product'
   */
  if (rtb_RelationalOperator_b1) {
    rtb_TrigonometricFunction6 = 0.0F;
  } else {
    /* Lookup_n-D: '<S79>/1-D Lookup Table2' */
    bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
    rtb_TrigonometricFunction6 *= intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
      QS_InnerRateLoop_ConstP.pooled8);
  }

  /* End of Switch: '<S80>/Switch' */

  /* Switch: '<S94>/Switch' incorporates:
   *  Constant: '<S94>/Constant'
   *  Logic: '<S77>/Logical Operator'
   *  Lookup_n-D: '<S93>/1-D Lookup Table2'
   *  Product: '<S93>/Product'
   */
  if (rtb_Compare_nh || rtb_RelationalOperator_b1) {
    rtb_Product = 0.0F;
  } else {
    /* Lookup_n-D: '<S93>/1-D Lookup Table2' */
    bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
    rtb_Product *= intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
      QS_InnerRateLoop_ConstP.pooled13);
  }

  /* End of Switch: '<S94>/Switch' */

  /* Switch: '<S107>/Switch' incorporates:
   *  DeadZone: '<S13>/Dead Zone1'
   *  Gain: '<S13>/phicmd'
   *  Inport: '<Root>/input_lat'
   */
  if (rtb_Compare_m) {
    /* Switch: '<S13>/Switch1' incorporates:
     *  Sum: '<S13>/Sum4'
     */
    if (rtb_Compare_bt) {
      /* Saturate: '<S13>/Saturation3' */
      if (rtb_TrigonometricFunction3 > 0.174520075F) {
        rtb_Abs_i = 0.174520075F;
      } else if (rtb_TrigonometricFunction3 < -0.174520075F) {
        rtb_Abs_i = -0.174520075F;
      } else {
        rtb_Abs_i = rtb_TrigonometricFunction3;
      }

      /* End of Saturate: '<S13>/Saturation3' */
    } else {
      rtb_Abs_i = rtb_Add6_a - rtb_Gain1;
    }

    /* End of Switch: '<S13>/Switch1' */
  } else {
    if (QS_InnerRateLoop_U.input_lat > 0.05F) {
      /* DeadZone: '<S13>/Dead Zone1' incorporates:
       *  Inport: '<Root>/input_lat'
       */
      tmp = QS_InnerRateLoop_U.input_lat - 0.05F;
    } else if (QS_InnerRateLoop_U.input_lat >= -0.05F) {
      /* DeadZone: '<S13>/Dead Zone1' */
      tmp = 0.0F;
    } else {
      /* DeadZone: '<S13>/Dead Zone1' incorporates:
       *  Inport: '<Root>/input_lat'
       */
      tmp = QS_InnerRateLoop_U.input_lat - -0.05F;
    }

    rtb_Abs_i = 0.785340309F * tmp;
  }

  /* End of Switch: '<S107>/Switch' */

  /* Switch: '<S107>/Switch1' incorporates:
   *  RelationalOperator: '<S107>/Relational Operator'
   *  Sum: '<S107>/Sum1'
   *  UnitDelay: '<S107>/Unit Delay'
   *  UnitDelay: '<S107>/Unit Delay1'
   *  UnitDelay: '<S107>/Unit Delay2'
   */
  if (QS_InnerRateLoop_DW.UnitDelay_DSTATE_j1 == rtb_Compare_m) {
    rtb_uDLookupTable2_bw = QS_InnerRateLoop_DW.UnitDelay1_DSTATE_fj;
  } else {
    rtb_uDLookupTable2_bw = QS_InnerRateLoop_DW.UnitDelay2_DSTATE_a - rtb_Abs_i;
  }

  /* End of Switch: '<S107>/Switch1' */

  /* Saturate: '<S107>/Saturation' */
  if (rtb_uDLookupTable2_bw > 0.0025F) {
    rtb_uDLookupTable_l_tmp = 0.0025F;
  } else if (rtb_uDLookupTable2_bw < -0.0025F) {
    rtb_uDLookupTable_l_tmp = -0.0025F;
  } else {
    rtb_uDLookupTable_l_tmp = rtb_uDLookupTable2_bw;
  }

  /* End of Saturate: '<S107>/Saturation' */

  /* Sum: '<S107>/Sum' */
  rtb_Gain1 = rtb_uDLookupTable2_bw - rtb_uDLookupTable_l_tmp;

  /* Sum: '<S107>/Sum2' */
  rtb_TrigonometricFunction3 = rtb_Abs_i + rtb_Gain1;

  /* Lookup_n-D: '<S35>/1-D Lookup Table2' */
  bpIdx = plook_u32ff_evenxg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Sum: '<S32>/Sum' incorporates:
   *  Lookup_n-D: '<S35>/1-D Lookup Table2'
   *  Product: '<S35>/Product'
   *  Sum: '<S32>/Sum1'
   *  UnitDelay: '<S32>/Unit Delay'
   *  UnitDelay: '<S32>/Unit Delay1'
   */
  rtb_Add6_a = (rtb_TrigonometricFunction3 -
                QS_InnerRateLoop_DW.UnitDelay1_DSTATE_a) * intrp1d_fu32fl_pw
    (bpIdx, rtb_Abs_i, QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_ob) -
    QS_InnerRateLoop_DW.UnitDelay_DSTATE_g;

  /* Lookup_n-D: '<S36>/1-D Lookup Table2' */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Sum: '<S85>/Sum' incorporates:
   *  Product: '<S85>/Product2'
   *  Sum: '<S85>/Sum2'
   *  UnitDelay: '<S85>/Unit Delay'
   */
  rtb_Product_cw = (rtb_Product_cw - rtb_Abs_k) * rtb_DeadZone3 / rtb_Product_cw
    - QS_InnerRateLoop_DW.UnitDelay_DSTATE_oc;

  /* Outport: '<Root>/theta_cmd' */
  QS_InnerRateLoop_Y.theta_cmd = rtb_Sum4_b;

  /* Outport: '<Root>/phi_cmd' */
  QS_InnerRateLoop_Y.phi_cmd = rtb_DeadZone;

  /* Sum: '<S110>/Sum' */
  rtb_Abs -= rtb_Sum2_k;

  /* Abs: '<S110>/Abs' */
  rtb_Abs = static_cast<real32_T>(fabs((real_T)rtb_Abs));

  /* Switch: '<S88>/Switch' incorporates:
   *  Constant: '<S113>/Constant'
   *  Constant: '<S88>/Constant'
   *  Lookup_n-D: '<S86>/1-D Lookup Table2'
   *  Product: '<S86>/Product'
   *  RelationalOperator: '<S113>/Compare'
   */
  if (rtb_Abs > 0.0F) {
    rtb_Saturation8 = 0.0F;
  } else {
    /* Lookup_n-D: '<S86>/1-D Lookup Table2' */
    bpIdx_0 = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U,
      &rtb_uDLookupTable2_bw);
    rtb_Saturation8 = intrp1d_fu32fl_pw(bpIdx_0, rtb_uDLookupTable2_bw,
      QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_e) * rtb_Product_ee;
  }

  /* End of Switch: '<S88>/Switch' */

  /* RelationalOperator: '<S12>/Relational Operator' incorporates:
   *  Constant: '<S12>/Constant3'
   */
  rtb_RelationalOperator_b1 = (rtb_Gain2 >= 18.0F);

  /* Outport: '<Root>/ScoreDisplay' incorporates:
   *  Logic: '<S12>/Logical Operator1'
   *  RelationalOperator: '<S12>/Relational Operator2'
   */
  QS_InnerRateLoop_Y.ScoreDisplay = (rtb_RelationalOperator_b1 && (rtb_Gain2 <=
    QS_InnerRateLoop_ConstB.Sum2));

  /* Outport: '<Root>/ScoreOn' incorporates:
   *  Logic: '<S12>/Logical Operator'
   *  RelationalOperator: '<S12>/Relational Operator1'
   */
  QS_InnerRateLoop_Y.ScoreOn = (rtb_RelationalOperator_b1 && (rtb_Gain2 <=
    QS_InnerRateLoop_ConstB.Sum1));

  /* Outport: '<Root>/Vdown_corr' */
  QS_InnerRateLoop_Y.Vdown_corr = rtb_Saturation2_h;

  /* Outport: '<Root>/Veast_corr' */
  QS_InnerRateLoop_Y.Veast_corr = rtb_Saturation1_p;

  /* Outport: '<Root>/Vnorth_corr' */
  QS_InnerRateLoop_Y.Vnorth_corr = rtb_Saturation_h;

  /* Outport: '<Root>/TrajectoryON' */
  QS_InnerRateLoop_Y.TrajectoryON = rtb_Compare_m;

  /* Sum: '<S19>/Sum' incorporates:
   *  Constant: '<S1>/Constant'
   *  Inport: '<Root>/Ax_mpss'
   *  Inport: '<Root>/Ay_mpss'
   *  Inport: '<Root>/Az_mpss'
   *  Inport: '<Root>/theta_rad'
   *  Product: '<S7>/Divide11'
   *  Product: '<S7>/Divide12'
   *  Product: '<S7>/Divide2'
   *  Sum: '<S1>/Sum2'
   *  Sum: '<S7>/Add7'
   *  Trigonometry: '<S7>/Trigonometric Function5'
   *  UnitDelay: '<S19>/Unit Delay'
   */
  rtb_Sum = (((rtb_TrigonometricFunction3_tmp * rtb_Saturation_fr_tmp *
               QS_InnerRateLoop_U.Az_mpss + rtb_TrigonometricFunction6_tmp *
               rtb_Saturation_fr_tmp * QS_InnerRateLoop_U.Ay_mpss) +
              -static_cast<real32_T>(sin((real_T)QS_InnerRateLoop_U.theta_rad)) *
              QS_InnerRateLoop_U.Ax_mpss) + 9.81) -
    QS_InnerRateLoop_DW.UnitDelay_DSTATE_j2;

  /* Sum: '<S20>/Sum' incorporates:
   *  Gain: '<Root>/Gain'
   *  Inport: '<Root>/Baro_Alt_m'
   *  UnitDelay: '<S20>/Unit Delay'
   */
  rtb_Saturation1_p = -QS_InnerRateLoop_U.Baro_Alt_m -
    QS_InnerRateLoop_DW.UnitDelay_DSTATE_da;

  /* Sum: '<S21>/Sum' incorporates:
   *  Gain: '<Root>/Gain'
   *  Inport: '<Root>/Baro_Alt_m'
   *  UnitDelay: '<S21>/Unit Delay'
   */
  rtb_Saturation_h = -QS_InnerRateLoop_U.Baro_Alt_m -
    QS_InnerRateLoop_DW.UnitDelay_DSTATE_ag;

  /* DiscreteIntegrator: '<S21>/Discrete-Time Integrator' incorporates:
   *  Gain: '<Root>/Gain'
   *  Inport: '<Root>/Baro_Alt_m'
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOA_j != 0) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_e =
      -QS_InnerRateLoop_U.Baro_Alt_m;
  }

  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_l <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_e =
      -QS_InnerRateLoop_U.Baro_Alt_m;
  }

  /* DiscreteIntegrator: '<S22>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_em <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cg = 0.0F;
  }

  rtb_Saturation2_h = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cg;

  /* Outport: '<Root>/CF_Alt' incorporates:
   *  DiscreteIntegrator: '<S21>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S22>/Discrete-Time Integrator'
   *  Sum: '<S1>/Sum1'
   */
  QS_InnerRateLoop_Y.CF_Alt =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_e +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cg;

  /* DiscreteIntegrator: '<S20>/Discrete-Time Integrator' incorporates:
   *  Gain: '<Root>/Gain'
   *  Inport: '<Root>/Baro_Alt_m'
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOA_m != 0) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_o3 =
      -QS_InnerRateLoop_U.Baro_Alt_m;
  }

  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_hn <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_o3 =
      -QS_InnerRateLoop_U.Baro_Alt_m;
  }

  /* Sum: '<S20>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S20>/Discrete-Time Integrator'
   *  Gain: '<Root>/Gain'
   *  Inport: '<Root>/Baro_Alt_m'
   */
  rtb_Product_ee = -QS_InnerRateLoop_U.Baro_Alt_m -
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_o3;

  /* Sum: '<S18>/Sum3' incorporates:
   *  Gain: '<S18>/Gain1'
   *  Sum: '<S18>/Sum5'
   *  UnitDelay: '<S18>/Unit Delay'
   *  UnitDelay: '<S18>/Unit Delay1'
   */
  rtb_Gain2 = (rtb_Product_ee - QS_InnerRateLoop_DW.UnitDelay1_DSTATE_a1) *
    1.41442716F - QS_InnerRateLoop_DW.UnitDelay_DSTATE_ak;

  /* DiscreteIntegrator: '<S18>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_IC_LO_e != 0) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_g = rtb_Product_ee;
  }

  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_Prev_gs <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_g = rtb_Product_ee;
  }

  /* DiscreteIntegrator: '<S19>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_pr <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_nv = 0.0F;
  }

  /* Sum: '<S1>/Sum' incorporates:
   *  DiscreteIntegrator: '<S18>/Discrete-Time Integrator1'
   *  DiscreteIntegrator: '<S19>/Discrete-Time Integrator'
   */
  rtb_DeadZone3 = QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_g +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_nv;

  /* Outport: '<Root>/CF_Vz' */
  QS_InnerRateLoop_Y.CF_Vz = rtb_DeadZone3;

  /* DiscreteIntegrator: '<S18>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_lm <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_gh = 0.0F;
  }

  /* DiscreteIntegrator: '<S32>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_f <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_dn = 0.0F;
  }

  /* DiscreteIntegrator: '<S48>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_f0 <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_nq = 0.0F;
  }

  /* DeadZone: '<S13>/Dead Zone3' incorporates:
   *  Inport: '<Root>/input_ped'
   */
  if (QS_InnerRateLoop_U.input_ped > 0.05F) {
    tmp = QS_InnerRateLoop_U.input_ped - 0.05F;
  } else if (QS_InnerRateLoop_U.input_ped >= -0.05F) {
    tmp = 0.0F;
  } else {
    tmp = QS_InnerRateLoop_U.input_ped - -0.05F;
  }

  /* End of DeadZone: '<S13>/Dead Zone3' */

  /* Sum: '<S105>/Sum' incorporates:
   *  Gain: '<S13>/rcmd'
   *  UnitDelay: '<S105>/Unit Delay'
   */
  rtb_Product_ee = 3.14136124F * tmp - QS_InnerRateLoop_DW.UnitDelay_DSTATE_k;

  /* DiscreteIntegrator: '<S105>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if ((QS_InnerRateLoop_U.engage &&
       (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_bv <= 0)) ||
      ((!QS_InnerRateLoop_U.engage) &&
       (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_bv == 1))) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_gy = 0.0F;
  }

  /* Update for Delay: '<S96>/Delay1' */
  QS_InnerRateLoop_DW.Delay1_DSTATE = rtb_Compare;

  /* Switch: '<S96>/Switch' */
  if (rtb_Compare_ab) {
    /* Update for Delay: '<S96>/Delay2' */
    QS_InnerRateLoop_DW.Delay2_DSTATE = rtb_Sum2_h;
  } else {
    /* Update for Delay: '<S96>/Delay2' incorporates:
     *  Constant: '<S96>/Constant1'
     */
    QS_InnerRateLoop_DW.Delay2_DSTATE = 0.0;
  }

  /* End of Switch: '<S96>/Switch' */

  /* Update for DiscreteIntegrator: '<S94>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_a += 0.0025F * rtb_Product;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRese = static_cast<int8_T>
    (rtb_Compare_m);

  /* Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOADI = 0U;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_m += 0.0025F *
    rtb_Sum1_j_idx_0;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_a = static_cast<int8_T>
    (rtb_Compare_m);

  /* Update for DiscreteIntegrator: '<S91>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_h += 0.0025F *
    rtb_Product_ng;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_o = static_cast<int8_T>
    (rtb_Compare_m);

  /* Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator1' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_IC_LOAD = 0U;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTATE += 0.0025F *
    rtb_Sum1_j_idx_1;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevRes = static_cast<int8_T>
    (rtb_Compare_m);

  /* Update for DiscreteIntegrator: '<S88>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_c += 0.0025F *
    rtb_Saturation8;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_b = static_cast<int8_T>
    (rtb_Compare_m);

  /* Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator2' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_IC_LOAD = 0U;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_DSTATE += 0.0025F * rtb_Add7_d;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_PrevRes = static_cast<int8_T>
    (rtb_Compare_m);

  /* Update for UnitDelay: '<S12>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE = rtb_Product2;

  /* Update for UnitDelay: '<S108>/Unit Delay1' */
  QS_InnerRateLoop_DW.UnitDelay1_DSTATE = rtb_Sum_j3;

  /* Update for UnitDelay: '<S108>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_ax = rtb_Compare_m;

  /* Update for UnitDelay: '<S108>/Unit Delay2' */
  QS_InnerRateLoop_DW.UnitDelay2_DSTATE = rtb_Switch_ep;

  /* Update for UnitDelay: '<S39>/Unit Delay' incorporates:
   *  DiscreteIntegrator: '<S39>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_a =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_fs;

  /* Update for DiscreteIntegrator: '<S31>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_hq += rtb_Product_b_tmp *
    0.0025F;

  /* Update for DiscreteIntegrator: '<S30>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_b +=
    rtb_derivativecutofffrequency_0 * 0.0025F;

  /* Update for DiscreteIntegrator: '<S32>/Discrete-Time Integrator1' incorporates:
   *  DiscreteIntegrator: '<S32>/Discrete-Time Integrator'
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_o += 0.0025F *
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_dn;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevR_g = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S47>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_mk += rtb_Product_b_tmp_0 *
    0.0025F;

  /* Update for DiscreteIntegrator: '<S46>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_o += rtb_Compare_k * 0.0025F;

  /* Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator1' incorporates:
   *  DiscreteIntegrator: '<S48>/Discrete-Time Integrator'
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_m += 0.0025F *
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_nq;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevR_j = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S57>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_d += rtb_Switch2_cz *
    0.0025F;

  /* Update for DiscreteIntegrator: '<S58>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S58>/derivative cutoff frequency '
   *  Sum: '<S58>/Sum'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_mr += (rtb_Switch3 -
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_mr) * 100.0F * 0.0025F;

  /* Update for Delay: '<S13>/Delay' */
  QS_InnerRateLoop_DW.icLoad = 0U;
  QS_InnerRateLoop_DW.Delay_DSTATE = rtb_uDLookupTable3;

  /* Update for Delay: '<S13>/Delay1' */
  QS_InnerRateLoop_DW.Delay1_DSTATE_j[0] = QS_InnerRateLoop_DW.Delay1_DSTATE_j[1];
  QS_InnerRateLoop_DW.Delay1_DSTATE_j[1] = rtb_Compare_m;

  /* Update for DiscreteIntegrator: '<S13>/Discrete-Time Integrator' incorporates:
   *  DiscreteIntegrator: '<S105>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOA_b = 0U;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_mz += 0.0025F *
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_gy;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_p = static_cast<int8_T>
    (rtb_Compare_m);

  /* Update for DiscreteIntegrator: '<S59>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE += 0.0025 * rtb_Gain_e;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_bp = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S40>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_f += 0.0025F *
    rtb_Product_ib;
  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_f >= 0.1F) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_f = 0.1F;
  } else {
    if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_f <= -0.1F) {
      QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_f = -0.1F;
    }
  }

  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_m = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* End of Update for DiscreteIntegrator: '<S40>/Discrete-Time Integrator' */

  /* Update for DiscreteIntegrator: '<S39>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_fs += 0.0025F *
    rtb_Product_of;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_n = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S41>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   *  Product: '<S41>/Product'
   *  Product: '<S41>/Product2'
   *  Sum: '<S41>/Sum'
   *  Sum: '<S41>/Sum2'
   *  UnitDelay: '<S41>/Unit Delay'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_c4 += ((rtb_uDLookupTable1 -
    rtb_uDLookupTable_cs) * rtb_Sum1_c0 / rtb_uDLookupTable1 -
    QS_InnerRateLoop_DW.UnitDelay_DSTATE_h) * rtb_uDLookupTable_cs * 0.0025F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_by = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S33>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_o2 += 0.0025F *
    rtb_Product_jr;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_e = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S34>/Discrete-Time Integrator' incorporates:
   *  Product: '<S34>/Product'
   *  Product: '<S34>/Product2'
   *  Sum: '<S34>/Sum'
   *  Sum: '<S34>/Sum2'
   *  UnitDelay: '<S34>/Unit Delay'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_bw += ((rtb_uDLookupTable1_p
    - rtb_uDLookupTable) * rtb_Sum1_fe / rtb_uDLookupTable1_p -
    QS_InnerRateLoop_DW.UnitDelay_DSTATE_b) * rtb_uDLookupTable * 0.0025F;

  /* Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_j += 0.0025F * rtb_Product_n;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_d = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator' incorporates:
   *  Product: '<S50>/Product'
   *  Sum: '<S50>/Sum'
   *  UnitDelay: '<S50>/Unit Delay'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_di += (rtb_Sum1_gg -
    QS_InnerRateLoop_DW.UnitDelay_DSTATE_eo) * rtb_Saturation_k * 0.0025F;

  /* Update for DiscreteIntegrator: '<S60>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_l += 0.0025F *
    rtb_Product_ear;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_j = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_hv += 0.0025F * rtb_Sum1_lq;

  /* Update for DiscreteIntegrator: '<S65>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S65>/Gain1'
   *  Gain: '<S65>/Gain2'
   *  Sum: '<S65>/Sum'
   *  UnitDelay: '<S65>/Unit Delay'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_co += (-5.66666651F *
    rtb_Product_hg[0] - QS_InnerRateLoop_DW.UnitDelay_DSTATE_f) * 100.0F *
    0.0025F;

  /* Update for DiscreteIntegrator: '<S66>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S66>/Gain1'
   *  Gain: '<S66>/Gain2'
   *  Sum: '<S66>/Sum'
   *  UnitDelay: '<S66>/Unit Delay'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_i += (-5.66666651F *
    rtb_Product_hg[1] - QS_InnerRateLoop_DW.UnitDelay_DSTATE_av) * 100.0F *
    0.0025F;

  /* Update for DiscreteIntegrator: '<S67>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S67>/Gain1'
   *  Gain: '<S67>/Gain2'
   *  Sum: '<S67>/Sum'
   *  UnitDelay: '<S67>/Unit Delay'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_ch += (-5.66666651F *
    rtb_Product_hg[2] - QS_InnerRateLoop_DW.UnitDelay_DSTATE_d) * 100.0F *
    0.0025F;

  /* Update for DiscreteIntegrator: '<S68>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S68>/Gain1'
   *  Gain: '<S68>/Gain2'
   *  Sum: '<S68>/Sum'
   *  UnitDelay: '<S68>/Unit Delay'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_a5 += (0.636F *
    rtb_Product_hg[3] - QS_InnerRateLoop_DW.UnitDelay_DSTATE_e) * 5.46F *
    0.0025F;

  /* Update for UnitDelay: '<S65>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_f = rtb_DiscreteTimeIntegrator_fi;

  /* Update for UnitDelay: '<S66>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_av = rtb_DiscreteTimeIntegrator_k1;

  /* Update for UnitDelay: '<S67>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_d = rtb_DiscreteTimeIntegrator_h;

  /* Update for UnitDelay: '<S68>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_e = rtb_DiscreteTimeIntegrator_n;

  /* Update for UnitDelay: '<S61>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_o = rtb_DiscreteTimeIntegrator_ia;

  /* Update for UnitDelay: '<S50>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_eo = rtb_DiscreteTimeIntegrator_nt;

  /* Update for UnitDelay: '<S34>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_b = rtb_DiscreteTimeIntegrator_cv;

  /* Update for UnitDelay: '<S41>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_h = rtb_DiscreteTimeIntegrator_hs;

  /* Update for DiscreteIntegrator: '<S84>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_n += 0.0025F *
    rtb_Sum1_j_idx_2;
  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_n >= 0.0174520072F) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_n = 0.0174520072F;
  } else {
    if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_n <= -0.0174520072F) {
      QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_n = -0.0174520072F;
    }
  }

  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_h = static_cast<int8_T>
    (rtb_Compare_m);

  /* End of Update for DiscreteIntegrator: '<S84>/Discrete-Time Integrator' */

  /* Update for DiscreteIntegrator: '<S85>/Discrete-Time Integrator' incorporates:
   *  Product: '<S85>/Product'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_g += rtb_Abs_k *
    rtb_Product_cw * 0.0025F;

  /* Update for UnitDelay: '<S106>/Unit Delay2' */
  QS_InnerRateLoop_DW.UnitDelay2_DSTATE_k = rtb_Sum4_a;

  /* Update for UnitDelay: '<S106>/Unit Delay1' */
  QS_InnerRateLoop_DW.UnitDelay1_DSTATE_f = rtb_derivativecutofffrequency_b;

  /* Update for UnitDelay: '<S106>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_e2 = rtb_Compare_m;

  /* Update for UnitDelay: '<S48>/Unit Delay1' */
  QS_InnerRateLoop_DW.UnitDelay1_DSTATE_c = rtb_Sum3_f;

  /* Update for UnitDelay: '<S48>/Unit Delay' incorporates:
   *  DiscreteIntegrator: '<S48>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_j =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_nq;

  /* Update for DiscreteIntegrator: '<S80>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cf += 0.0025F *
    rtb_TrigonometricFunction6;
  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cf >= 0.0174520072F) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cf = 0.0174520072F;
  } else {
    if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cf <= -0.0174520072F) {
      QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cf = -0.0174520072F;
    }
  }

  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_dc = static_cast<int8_T>
    (rtb_Compare_m);

  /* End of Update for DiscreteIntegrator: '<S80>/Discrete-Time Integrator' */

  /* Update for DiscreteIntegrator: '<S81>/Discrete-Time Integrator' incorporates:
   *  Product: '<S81>/Product'
   *  Product: '<S81>/Product2'
   *  Sum: '<S81>/Sum'
   *  Sum: '<S81>/Sum2'
   *  UnitDelay: '<S81>/Unit Delay'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cc += ((rtb_uDLookupTable1_l
    - rtb_uDLookupTable_a) * rtb_Sum1_gw / rtb_uDLookupTable1_l -
    QS_InnerRateLoop_DW.UnitDelay_DSTATE_gi) * rtb_uDLookupTable_a * 0.0025F;

  /* Update for UnitDelay: '<S107>/Unit Delay2' */
  QS_InnerRateLoop_DW.UnitDelay2_DSTATE_a = rtb_TrigonometricFunction3;

  /* Update for UnitDelay: '<S107>/Unit Delay1' */
  QS_InnerRateLoop_DW.UnitDelay1_DSTATE_fj = rtb_Gain1;

  /* Update for UnitDelay: '<S107>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_j1 = rtb_Compare_m;

  /* Update for UnitDelay: '<S32>/Unit Delay1' */
  QS_InnerRateLoop_DW.UnitDelay1_DSTATE_a = rtb_DiscreteTimeIntegrator1;

  /* Update for UnitDelay: '<S32>/Unit Delay' incorporates:
   *  DiscreteIntegrator: '<S32>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_g =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_dn;

  /* Update for UnitDelay: '<S81>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_gi = rtb_Add6_i;

  /* Update for UnitDelay: '<S85>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_oc = rtb_Add5_e;

  /* Update for UnitDelay: '<S19>/Unit Delay' incorporates:
   *  DiscreteIntegrator: '<S19>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_j2 =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_nv;

  /* Update for UnitDelay: '<S20>/Unit Delay' incorporates:
   *  DiscreteIntegrator: '<S20>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_da =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_o3;

  /* Update for UnitDelay: '<S21>/Unit Delay' incorporates:
   *  DiscreteIntegrator: '<S21>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_ag =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_e;

  /* Update for DiscreteIntegrator: '<S21>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S21>/Gain'
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOA_j = 0U;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_e += 0.5F * rtb_Saturation_h
    * 0.0025F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_l = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S22>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S22>/Gain'
   *  Inport: '<Root>/engage'
   *  Sum: '<S22>/Sum'
   *  UnitDelay: '<S22>/Unit Delay'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cg += (rtb_DeadZone3 -
    QS_InnerRateLoop_DW.UnitDelay_DSTATE_o1) * 0.5F * 0.0025F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_em = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S20>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S20>/Gain'
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOA_m = 0U;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_o3 += 0.5F *
    rtb_Saturation1_p * 0.0025F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_hn = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for UnitDelay: '<S18>/Unit Delay1' incorporates:
   *  DiscreteIntegrator: '<S18>/Discrete-Time Integrator1'
   */
  QS_InnerRateLoop_DW.UnitDelay1_DSTATE_a1 =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_g;

  /* Update for UnitDelay: '<S18>/Unit Delay' incorporates:
   *  DiscreteIntegrator: '<S18>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_ak =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_gh;

  /* Update for DiscreteIntegrator: '<S18>/Discrete-Time Integrator1' incorporates:
   *  DiscreteIntegrator: '<S18>/Discrete-Time Integrator'
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_IC_LO_e = 0U;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_g += 0.0025F *
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_gh;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_Prev_gs = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S19>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S19>/Gain'
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_nv += static_cast<real32_T>
    ((0.5 * rtb_Sum * 0.0025));
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_pr = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for UnitDelay: '<S22>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_o1 = rtb_Saturation2_h;

  /* Update for DiscreteIntegrator: '<S18>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S18>/Gain'
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_gh += 2.828F * rtb_Gain2 *
    0.0025F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_lm = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S32>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   *  Lookup_n-D: '<S36>/1-D Lookup Table2'
   *  Product: '<S36>/Product'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_dn += intrp1d_fu32fl_pw(bpIdx,
    rtb_Abs_i, QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_l) * rtb_Add6_a *
    0.0025F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_f = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_nq += 0.0025F * rtb_Add5_ky;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_f0 = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for UnitDelay: '<S105>/Unit Delay' incorporates:
   *  DiscreteIntegrator: '<S105>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_k =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_gy;

  /* Update for DiscreteIntegrator: '<S105>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S105>/Gain'
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_gy += 10.0F * rtb_Product_ee *
    0.0025F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_bv = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);
}

/* Model initialize function */
void QS_InnerRateLoopModelClass::initialize()
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus((&QS_InnerRateLoop_M), (NULL));

  /* block I/O */
  (void) memset(((void *) &QS_InnerRateLoop_B), 0,
                sizeof(B_QS_InnerRateLoop_T));

  /* states (dwork) */
  (void) memset((void *)&QS_InnerRateLoop_DW, 0,
                sizeof(DW_QS_InnerRateLoop_T));

  /* external inputs */
  (void)memset(&QS_InnerRateLoop_U, 0, sizeof(ExtU_QS_InnerRateLoop_T));

  /* external outputs */
  (void) memset((void *)&QS_InnerRateLoop_Y, 0,
                sizeof(ExtY_QS_InnerRateLoop_T));

  /* InitializeConditions for Delay: '<S96>/Delay1' */
  QS_InnerRateLoop_DW.Delay1_DSTATE = false;

  /* InitializeConditions for Delay: '<S96>/Delay2' */
  QS_InnerRateLoop_DW.Delay2_DSTATE = 0.0;

  /* InitializeConditions for DiscreteIntegrator: '<S94>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_a = 0.0F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRese = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S69>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOADI = 1U;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_a = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S91>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_h = 0.0F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_o = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S69>/Discrete-Time Integrator1' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_IC_LOAD = 1U;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevRes = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S88>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_c = 0.0F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_b = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S69>/Discrete-Time Integrator2' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_IC_LOAD = 1U;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_PrevRes = 2;

  /* InitializeConditions for UnitDelay: '<S12>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE = 0.0F;

  /* InitializeConditions for UnitDelay: '<S108>/Unit Delay1' */
  QS_InnerRateLoop_DW.UnitDelay1_DSTATE = 0.0F;

  /* InitializeConditions for UnitDelay: '<S108>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_ax = false;

  /* InitializeConditions for UnitDelay: '<S108>/Unit Delay2' */
  QS_InnerRateLoop_DW.UnitDelay2_DSTATE = 0.0F;

  /* InitializeConditions for UnitDelay: '<S39>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_a = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S31>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_hq = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S30>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_b = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S32>/Discrete-Time Integrator1' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_o = 0.0F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevR_g = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S47>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_mk = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S46>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_o = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S48>/Discrete-Time Integrator1' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_m = 0.0F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevR_j = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S57>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_d = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S58>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_mr = 0.0F;

  /* InitializeConditions for RateLimiter: '<S12>/Rate Limiter' */
  QS_InnerRateLoop_DW.PrevY = 0.0F;

  /* InitializeConditions for Delay: '<S13>/Delay' */
  QS_InnerRateLoop_DW.icLoad = 1U;

  /* InitializeConditions for Delay: '<S13>/Delay1' */
  QS_InnerRateLoop_DW.Delay1_DSTATE_j[0] = false;
  QS_InnerRateLoop_DW.Delay1_DSTATE_j[1] = false;

  /* InitializeConditions for DiscreteIntegrator: '<S13>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOA_b = 1U;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_p = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S59>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE = 0.0;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_bp = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S40>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_f = 0.0F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_m = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S39>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_fs = 0.0F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_n = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S41>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_c4 = 0.0F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_by = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S33>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_o2 = 0.0F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_e = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S34>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_bw = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S49>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_j = 0.0F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_d = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S50>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_di = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S60>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_l = 0.0F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_j = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S61>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_hv = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S65>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_co = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S66>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_i = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S67>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_ch = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S68>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_a5 = 0.0F;

  /* InitializeConditions for UnitDelay: '<S65>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_f = 0.0F;

  /* InitializeConditions for UnitDelay: '<S66>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_av = 0.0F;

  /* InitializeConditions for UnitDelay: '<S67>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_d = 0.0F;

  /* InitializeConditions for UnitDelay: '<S68>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_e = 0.0F;

  /* InitializeConditions for UnitDelay: '<S61>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_o = 0.0F;

  /* InitializeConditions for UnitDelay: '<S50>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_eo = 0.0F;

  /* InitializeConditions for UnitDelay: '<S34>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_b = 0.0F;

  /* InitializeConditions for UnitDelay: '<S41>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_h = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S84>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_n = 0.0F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_h = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S85>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_g = 0.0F;

  /* InitializeConditions for UnitDelay: '<S106>/Unit Delay2' */
  QS_InnerRateLoop_DW.UnitDelay2_DSTATE_k = 0.0F;

  /* InitializeConditions for UnitDelay: '<S106>/Unit Delay1' */
  QS_InnerRateLoop_DW.UnitDelay1_DSTATE_f = 0.0F;

  /* InitializeConditions for UnitDelay: '<S106>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_e2 = false;

  /* InitializeConditions for UnitDelay: '<S48>/Unit Delay1' */
  QS_InnerRateLoop_DW.UnitDelay1_DSTATE_c = 0.0F;

  /* InitializeConditions for UnitDelay: '<S48>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_j = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S80>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cf = 0.0F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_dc = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S81>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cc = 0.0F;

  /* InitializeConditions for UnitDelay: '<S107>/Unit Delay2' */
  QS_InnerRateLoop_DW.UnitDelay2_DSTATE_a = 0.0F;

  /* InitializeConditions for UnitDelay: '<S107>/Unit Delay1' */
  QS_InnerRateLoop_DW.UnitDelay1_DSTATE_fj = 0.0F;

  /* InitializeConditions for UnitDelay: '<S107>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_j1 = false;

  /* InitializeConditions for UnitDelay: '<S32>/Unit Delay1' */
  QS_InnerRateLoop_DW.UnitDelay1_DSTATE_a = 0.0F;

  /* InitializeConditions for UnitDelay: '<S32>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_g = 0.0F;

  /* InitializeConditions for UnitDelay: '<S81>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_gi = 0.0F;

  /* InitializeConditions for UnitDelay: '<S85>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_oc = 0.0F;

  /* InitializeConditions for UnitDelay: '<S19>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_j2 = 0.0F;

  /* InitializeConditions for UnitDelay: '<S20>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_da = 0.0F;

  /* InitializeConditions for UnitDelay: '<S21>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_ag = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S21>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOA_j = 1U;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_l = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S22>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cg = 0.0F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_em = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S20>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOA_m = 1U;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_hn = 2;

  /* InitializeConditions for UnitDelay: '<S18>/Unit Delay1' */
  QS_InnerRateLoop_DW.UnitDelay1_DSTATE_a1 = 0.0F;

  /* InitializeConditions for UnitDelay: '<S18>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_ak = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S18>/Discrete-Time Integrator1' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_IC_LO_e = 1U;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_Prev_gs = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S19>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_nv = 0.0F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_pr = 2;

  /* InitializeConditions for UnitDelay: '<S22>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_o1 = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S18>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_gh = 0.0F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_lm = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S32>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_dn = 0.0F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_f = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S48>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_nq = 0.0F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_f0 = 2;

  /* InitializeConditions for UnitDelay: '<S105>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_k = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S105>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_gy = 0.0F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_bv = 2;

  /* SystemInitialize for Enabled SubSystem: '<S11>/Enabled Subsystem1' */
  /* SystemInitialize for Outport: '<S98>/lon_engaged' */
  QS_InnerRateLoop_B.lon = 0.0F;

  /* SystemInitialize for Outport: '<S98>/lat_engaged' */
  QS_InnerRateLoop_B.lat = 0.0F;

  /* SystemInitialize for Outport: '<S98>/col_engaged' */
  QS_InnerRateLoop_B.col_h = 0.0F;

  /* SystemInitialize for Outport: '<S98>/ped_engaged' */
  QS_InnerRateLoop_B.ped = 0.0F;

  /* End of SystemInitialize for SubSystem: '<S11>/Enabled Subsystem1' */

  /* SystemInitialize for Enabled SubSystem: '<S13>/Enabled Subsystem1' */
  /* SystemInitialize for Outport: '<S104>/col_engaged' */
  QS_InnerRateLoop_B.col = 0.0F;

  /* End of SystemInitialize for SubSystem: '<S13>/Enabled Subsystem1' */

  /* SystemInitialize for Enabled SubSystem: '<Root>/Determine Heading at Start of Manuever  All trajectories relative to this heading' */
  /* SystemInitialize for Outport: '<S4>/psi_engaged (deg)' */
  QS_InnerRateLoop_B.Gain = 0.0F;

  /* End of SystemInitialize for SubSystem: '<Root>/Determine Heading at Start of Manuever  All trajectories relative to this heading' */

  /* SystemInitialize for Enabled SubSystem: '<Root>/Enabled Subsystem Grab and Freeze Value Upon Engagement' */
  /* SystemInitialize for Outport: '<S6>/Out1' */
  QS_InnerRateLoop_B.In1[0] = 0.0F;
  QS_InnerRateLoop_B.In1[1] = 0.0F;
  QS_InnerRateLoop_B.In1[2] = 0.0F;
  QS_InnerRateLoop_B.In1[3] = 0.0F;

  /* End of SystemInitialize for SubSystem: '<Root>/Enabled Subsystem Grab and Freeze Value Upon Engagement' */
}

/* Model terminate function */
void QS_InnerRateLoopModelClass::terminate()
{
  /* (no terminate code required) */
}

/* Constructor */
QS_InnerRateLoopModelClass::QS_InnerRateLoopModelClass()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
QS_InnerRateLoopModelClass::~QS_InnerRateLoopModelClass()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_QS_InnerRateLoop_T * QS_InnerRateLoopModelClass::getRTM()
{
  return (&QS_InnerRateLoop_M);
}
