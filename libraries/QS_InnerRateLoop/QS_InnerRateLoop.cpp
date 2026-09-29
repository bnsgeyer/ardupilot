/*
 * QS_InnerRateLoop.cpp
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

#include "QS_InnerRateLoop.h"
#include <math.h>
#include "rtwtypes.h"
#include "QS_InnerRateLoop_private.h"

uint32_T plook_u32ff_evencg(real32_T u, real32_T bp0, real32_T bpSpace, uint32_T
  maxIndex, real32_T *fraction)
{
  real32_T fbpIndex;
  real32_T invSpc;
  uint32_T bpIndex;

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
    *fraction = (u - (static_cast<real32_T>(static_cast<uint32_T>(fbpIndex)) *
                      bpSpace + bp0)) * invSpc;
  } else {
    bpIndex = maxIndex - 1U;
    *fraction = 1.0F;
  }

  return bpIndex;
}

real32_T intrp1d_fu32fl_pw(uint32_T bpIndex, real32_T frac, const real32_T
  table[])
{
  real32_T yL_0d0;

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  yL_0d0 = table[bpIndex];
  return (table[bpIndex + 1U] - yL_0d0) * frac + yL_0d0;
}

uint32_T plook_u32ff_evenxg(real32_T u, real32_T bp0, real32_T bpSpace, uint32_T
  maxIndex, real32_T *fraction)
{
  real32_T fbpIndex;
  real32_T invSpc;
  uint32_T bpIndex;

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
    *fraction = (u - (static_cast<real32_T>(static_cast<uint32_T>(fbpIndex)) *
                      bpSpace + bp0)) * invSpc;
  } else {
    bpIndex = maxIndex - 1U;
    *fraction = (u - (static_cast<real32_T>(maxIndex - 1U) * bpSpace + bp0)) *
      invSpc;
  }

  return bpIndex;
}

uint32_T plook_u32ff_bincg(real32_T u, const real32_T bp[], uint32_T maxIndex,
  real32_T *fraction)
{
  uint32_T bpIdx;
  uint32_T bpIndex;
  uint32_T iRght;

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
  real32_T yL_0d0;
  real32_T yL_0d1;
  uint32_T offset_1d;

  /* Column-major Interpolation 2-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'off'
     Overflow mode: 'portable wrapping'
   */
  offset_1d = bpIndex[1U] * stride + bpIndex[0U];
  yL_0d0 = table[offset_1d];
  yL_0d0 += (table[offset_1d + 1U] - yL_0d0) * frac[0U];
  offset_1d += stride;
  yL_0d1 = table[offset_1d];
  return (((table[offset_1d + 1U] - yL_0d1) * frac[0U] + yL_0d1) - yL_0d0) *
    frac[1U] + yL_0d0;
}

/* Model step function */
void QS_InnerRateLoop::step()
{
  real_T rtb_Gain_on;
  real_T rtb_Sum;
  int32_T i;
  int32_T iU;
  int32_T tmp;
  real32_T rtb_uDLookupTable2[36];
  real32_T rtb_uDLookupTable2_k5[16];
  real32_T rtb_Add5_0[9];
  real32_T rtb_Product_b[4];
  real32_T rtb_Product_hg[4];
  real32_T rtb_Sum1_f[4];
  real32_T fractions[2];
  real32_T fractions_0[2];
  real32_T rtb_Abs;
  real32_T rtb_Abs_i;
  real32_T rtb_Add5;
  real32_T rtb_Add5_e;
  real32_T rtb_Add6;
  real32_T rtb_Add6_i;
  real32_T rtb_Add7_n;
  real32_T rtb_DeadZone3;
  real32_T rtb_DiscreteTimeIntegrator1_d;
  real32_T rtb_DiscreteTimeIntegrator1_g;
  real32_T rtb_DiscreteTimeIntegrator_bs;
  real32_T rtb_DiscreteTimeIntegrator_dh;
  real32_T rtb_DiscreteTimeIntegrator_ff;
  real32_T rtb_DiscreteTimeIntegrator_fi;
  real32_T rtb_DiscreteTimeIntegrator_g;
  real32_T rtb_Gain2;
  real32_T rtb_Gain3;
  real32_T rtb_Gain4;
  real32_T rtb_Gain_ir;
  real32_T rtb_Product;
  real32_T rtb_Product_b_tmp;
  real32_T rtb_Product_b_tmp_0;
  real32_T rtb_Product_ear;
  real32_T rtb_Product_ee;
  real32_T rtb_Product_ib;
  real32_T rtb_Product_jr;
  real32_T rtb_Product_n;
  real32_T rtb_Product_ng;
  real32_T rtb_Product_of;
  real32_T rtb_Saturation8;
  real32_T rtb_Sum1_b3;
  real32_T rtb_Sum1_c;
  real32_T rtb_Sum1_c0;
  real32_T rtb_Sum1_cq;
  real32_T rtb_Sum1_e;
  real32_T rtb_Sum1_eo;
  real32_T rtb_Sum1_j_idx_0;
  real32_T rtb_Sum1_j_idx_1;
  real32_T rtb_Sum1_j_idx_2;
  real32_T rtb_Sum1_j_idx_3;
  real32_T rtb_Sum1_ky;
  real32_T rtb_Sum1_mn;
  real32_T rtb_Sum3_f;
  real32_T rtb_Sum4_f;
  real32_T rtb_Sum_hb;
  real32_T rtb_Sum_j;
  real32_T rtb_Sum_lu;
  real32_T rtb_Sum_nx;
  real32_T rtb_Switch2;
  real32_T rtb_TrigonometricFunction6;
  real32_T rtb_derivativecutofffrequency_0;
  real32_T rtb_derivativecutofffrequency_a;
  real32_T rtb_derivativecutofffrequency_f;
  real32_T rtb_uDLookupTable;
  real32_T rtb_uDLookupTable1;
  real32_T rtb_uDLookupTable1_g0;
  real32_T rtb_uDLookupTable1_p;
  real32_T rtb_uDLookupTable2_n;
  real32_T rtb_uDLookupTable_cs;
  real32_T rtb_uDLookupTable_d;
  real32_T rtb_uDLookupTable_e;
  real32_T rtb_wcmd;
  uint32_T bpIndices[2];
  uint32_T bpIndices_0[2];
  uint32_T bpIdx;
  uint32_T bpIdx_0;
  boolean_T rtb_LogicalOperator;
  boolean_T tmp_0;

  /* Trigonometry: '<S10>/Trigonometric Function2' incorporates:
   *  Inport: '<Root>/theta_rad'
   *  Trigonometry: '<S11>/Trigonometric Function2'
   */
  rtb_Product_of = static_cast<real32_T>(cos(static_cast<real_T>
    (QS_InnerRateLoop_U.theta_rad)));

  /* Trigonometry: '<S10>/Trigonometric Function5' incorporates:
   *  Inport: '<Root>/theta_rad'
   *  Trigonometry: '<S11>/Trigonometric Function5'
   */
  rtb_Sum3_f = static_cast<real32_T>(sin(static_cast<real_T>
    (QS_InnerRateLoop_U.theta_rad)));

  /* Trigonometry: '<S10>/Trigonometric Function4' incorporates:
   *  Inport: '<Root>/phi_rad'
   *  Trigonometry: '<S11>/Trigonometric Function4'
   */
  rtb_DiscreteTimeIntegrator1_g = static_cast<real32_T>(sin(static_cast<real_T>
    (QS_InnerRateLoop_U.phi_rad)));

  /* Trigonometry: '<S10>/Trigonometric Function1' incorporates:
   *  Inport: '<Root>/phi_rad'
   *  Trigonometry: '<S11>/Trigonometric Function1'
   */
  rtb_derivativecutofffrequency_a = static_cast<real32_T>(cos(static_cast<real_T>
    (QS_InnerRateLoop_U.phi_rad)));

  /* Product: '<S10>/Divide11' incorporates:
   *  Product: '<S11>/Divide11'
   *  Trigonometry: '<S10>/Trigonometric Function2'
   *  Trigonometry: '<S10>/Trigonometric Function4'
   */
  rtb_TrigonometricFunction6 = rtb_DiscreteTimeIntegrator1_g * rtb_Product_of;

  /* Product: '<S10>/Divide12' incorporates:
   *  Product: '<S11>/Divide12'
   *  Trigonometry: '<S10>/Trigonometric Function1'
   *  Trigonometry: '<S10>/Trigonometric Function2'
   */
  rtb_Product = rtb_derivativecutofffrequency_a * rtb_Product_of;

  /* Sum: '<S20>/Sum' incorporates:
   *  Constant: '<S1>/Constant'
   *  Inport: '<Root>/Ax_mpss'
   *  Inport: '<Root>/Ay_mpss'
   *  Inport: '<Root>/Az_mpss'
   *  Product: '<S10>/Divide11'
   *  Product: '<S10>/Divide12'
   *  Product: '<S10>/Divide2'
   *  Sum: '<S10>/Add7'
   *  Sum: '<S1>/Sum2'
   *  Trigonometry: '<S10>/Trigonometric Function5'
   *  UnitDelay: '<S20>/Unit Delay'
   */
  rtb_Sum = (((rtb_Product * QS_InnerRateLoop_U.Az_mpss +
               rtb_TrigonometricFunction6 * QS_InnerRateLoop_U.Ay_mpss) +
              -rtb_Sum3_f * QS_InnerRateLoop_U.Ax_mpss) + 9.81) -
    QS_InnerRateLoop_DW.UnitDelay_DSTATE;

  /* Logic: '<Root>/Logical Operator2' incorporates:
   *  DiscreteIntegrator: '<S70>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S70>/Discrete-Time Integrator1'
   *  DiscreteIntegrator: '<S70>/Discrete-Time Integrator2'
   *  DiscreteIntegrator: '<S8>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S96>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S9>/Discrete-Time Integrator'
   *  Inport: '<Root>/engage'
   *  Logic: '<Root>/Logical Operator'
   *  Logic: '<Root>/Logical Operator1'
   */
  tmp_0 = !QS_InnerRateLoop_U.engage;

  /* Outputs for Enabled SubSystem: '<Root>/Determine Heading at Start of Manuever  All trajectories relative to this heading' incorporates:
   *  EnablePort: '<S4>/Enable'
   */
  if (tmp_0) {
    /* Outport: '<Root>/vehheadingcmd' incorporates:
     *  Gain: '<S4>/Gain'
     *  Inport: '<Root>/psi_rad'
     */
    QS_InnerRateLoop_Y.vehheadingcmd = 57.2957802F * QS_InnerRateLoop_U.psi_rad;
  }

  /* End of Logic: '<Root>/Logical Operator2' */
  /* End of Outputs for SubSystem: '<Root>/Determine Heading at Start of Manuever  All trajectories relative to this heading' */

  /* Sum: '<S21>/Sum' incorporates:
   *  Gain: '<Root>/Gain'
   *  Inport: '<Root>/Baro_Alt_m'
   *  UnitDelay: '<S21>/Unit Delay'
   */
  rtb_Sum_j = -QS_InnerRateLoop_U.Baro_Alt_m -
    QS_InnerRateLoop_DW.UnitDelay_DSTATE_d;

  /* Sum: '<S22>/Sum' incorporates:
   *  Gain: '<Root>/Gain'
   *  Inport: '<Root>/Baro_Alt_m'
   *  UnitDelay: '<S22>/Unit Delay'
   */
  rtb_Sum_nx = -QS_InnerRateLoop_U.Baro_Alt_m -
    QS_InnerRateLoop_DW.UnitDelay_DSTATE_a;

  /* DiscreteIntegrator: '<S22>/Discrete-Time Integrator' incorporates:
   *  Gain: '<Root>/Gain'
   *  Inport: '<Root>/Baro_Alt_m'
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOADI != 0) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_e =
      -QS_InnerRateLoop_U.Baro_Alt_m;
  }

  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRese <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_e =
      -QS_InnerRateLoop_U.Baro_Alt_m;
  }

  /* DiscreteIntegrator: '<S23>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_e <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_c = 0.0F;
  }

  rtb_DiscreteTimeIntegrator_ff =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_c;

  /* Outport: '<Root>/CF_Alt' incorporates:
   *  DiscreteIntegrator: '<S22>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S23>/Discrete-Time Integrator'
   *  Sum: '<S1>/Sum1'
   */
  QS_InnerRateLoop_Y.CF_Alt =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_e +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_c;

  /* DiscreteIntegrator: '<S21>/Discrete-Time Integrator' incorporates:
   *  Gain: '<Root>/Gain'
   *  Inport: '<Root>/Baro_Alt_m'
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOA_m != 0) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_o =
      -QS_InnerRateLoop_U.Baro_Alt_m;
  }

  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_h <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_o =
      -QS_InnerRateLoop_U.Baro_Alt_m;
  }

  /* Sum: '<S21>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S21>/Discrete-Time Integrator'
   *  Gain: '<Root>/Gain'
   *  Inport: '<Root>/Baro_Alt_m'
   */
  rtb_Sum1_mn = -QS_InnerRateLoop_U.Baro_Alt_m -
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_o;

  /* Sum: '<S19>/Sum3' incorporates:
   *  Gain: '<S19>/Gain1'
   *  Sum: '<S19>/Sum5'
   *  UnitDelay: '<S19>/Unit Delay'
   *  UnitDelay: '<S19>/Unit Delay1'
   */
  rtb_Gain_ir = (rtb_Sum1_mn - QS_InnerRateLoop_DW.UnitDelay1_DSTATE) *
    1.41442716F - QS_InnerRateLoop_DW.UnitDelay_DSTATE_ak;

  /* DiscreteIntegrator: '<S19>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_IC_LOAD != 0) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTATE = rtb_Sum1_mn;
  }

  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevRes <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTATE = rtb_Sum1_mn;
  }

  /* DiscreteIntegrator: '<S20>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_p <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_n = 0.0F;
  }

  /* Sum: '<S1>/Sum' incorporates:
   *  DiscreteIntegrator: '<S19>/Discrete-Time Integrator1'
   *  DiscreteIntegrator: '<S20>/Discrete-Time Integrator'
   */
  rtb_Gain3 = QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTATE +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_n;

  /* Outport: '<Root>/CF_Vz' */
  QS_InnerRateLoop_Y.CF_Vz = rtb_Gain3;

  /* Sum: '<S23>/Sum' incorporates:
   *  UnitDelay: '<S23>/Unit Delay'
   */
  rtb_Sum1_mn = rtb_Gain3 - QS_InnerRateLoop_DW.UnitDelay_DSTATE_o;

  /* DeadZone: '<Root>/Dead Zone1' incorporates:
   *  Inport: '<Root>/input_lat'
   */
  if (QS_InnerRateLoop_U.input_lat > 0.07F) {
    rtb_Sum1_j_idx_0 = QS_InnerRateLoop_U.input_lat - 0.07F;
  } else if (QS_InnerRateLoop_U.input_lat >= -0.07F) {
    rtb_Sum1_j_idx_0 = 0.0F;
  } else {
    rtb_Sum1_j_idx_0 = QS_InnerRateLoop_U.input_lat - -0.07F;
  }

  /* Lookup_n-D: '<Root>/1-D Lookup Table1' incorporates:
   *  DeadZone: '<Root>/Dead Zone1'
   */
  bpIdx = plook_u32ff_evencg(rtb_Sum1_j_idx_0, -1.0F, 0.199999988F, 10U,
    &rtb_Abs_i);

  /* Sum: '<S9>/Sum' incorporates:
   *  Lookup_n-D: '<Root>/1-D Lookup Table1'
   *  UnitDelay: '<S9>/Unit Delay'
   */
  rtb_Sum_lu = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.pooled18) - QS_InnerRateLoop_DW.UnitDelay_DSTATE_c;

  /* DeadZone: '<Root>/Dead Zone2' incorporates:
   *  Inport: '<Root>/input_lon'
   */
  if (QS_InnerRateLoop_U.input_lon > 0.07F) {
    rtb_Sum1_j_idx_0 = QS_InnerRateLoop_U.input_lon - 0.07F;
  } else if (QS_InnerRateLoop_U.input_lon >= -0.07F) {
    rtb_Sum1_j_idx_0 = 0.0F;
  } else {
    rtb_Sum1_j_idx_0 = QS_InnerRateLoop_U.input_lon - -0.07F;
  }

  /* Lookup_n-D: '<Root>/1-D Lookup Table2' incorporates:
   *  DeadZone: '<Root>/Dead Zone2'
   */
  bpIdx = plook_u32ff_evencg(rtb_Sum1_j_idx_0, -1.0F, 0.199999988F, 10U,
    &rtb_Abs_i);

  /* Sum: '<S8>/Sum' incorporates:
   *  Lookup_n-D: '<Root>/1-D Lookup Table2'
   *  UnitDelay: '<S8>/Unit Delay'
   */
  rtb_Sum_hb = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.pooled18) - QS_InnerRateLoop_DW.UnitDelay_DSTATE_dr;

  /* Trigonometry: '<S12>/Trigonometric Function3' incorporates:
   *  Inport: '<Root>/psi_rad'
   */
  rtb_Gain3 = static_cast<real32_T>(cos(static_cast<real_T>
    (QS_InnerRateLoop_U.psi_rad)));

  /* Trigonometry: '<S12>/Trigonometric Function6' incorporates:
   *  Inport: '<Root>/psi_rad'
   */
  rtb_uDLookupTable_cs = static_cast<real32_T>(sin(static_cast<real_T>
    (QS_InnerRateLoop_U.psi_rad)));

  /* Sum: '<S12>/Add5' incorporates:
   *  Inport: '<Root>/vD_fps (KF)'
   *  Inport: '<Root>/vE_fps (KF)'
   *  Inport: '<Root>/vN_fps (KF)'
   *  Product: '<S12>/Divide'
   *  Product: '<S12>/Divide2'
   *  Product: '<S12>/Divide6'
   */
  rtb_Add5_e = (QS_InnerRateLoop_ConstB.TrigonometricFunction2 * rtb_Gain3 *
                QS_InnerRateLoop_U.vN_fpsKF +
                QS_InnerRateLoop_ConstB.TrigonometricFunction2 *
                rtb_uDLookupTable_cs * QS_InnerRateLoop_U.vE_fpsKF) +
    -QS_InnerRateLoop_ConstB.TrigonometricFunction5 *
    QS_InnerRateLoop_U.vD_fpsKF;

  /* Product: '<S12>/Divide1' incorporates:
   *  Product: '<S12>/Divide7'
   */
  rtb_Add6_i = QS_InnerRateLoop_ConstB.TrigonometricFunction4 *
    QS_InnerRateLoop_ConstB.TrigonometricFunction5;

  /* Sum: '<S12>/Add6' incorporates:
   *  Inport: '<Root>/vD_fps (KF)'
   *  Inport: '<Root>/vE_fps (KF)'
   *  Inport: '<Root>/vN_fps (KF)'
   *  Product: '<S12>/Divide1'
   *  Product: '<S12>/Divide11'
   *  Product: '<S12>/Divide13'
   *  Product: '<S12>/Divide15'
   *  Product: '<S12>/Divide3'
   *  Product: '<S12>/Divide7'
   *  Product: '<S12>/Divide8'
   *  Sum: '<S12>/Add1'
   *  Sum: '<S12>/Add3'
   */
  rtb_Add6_i = ((rtb_Add6_i * rtb_Gain3 -
                 QS_InnerRateLoop_ConstB.TrigonometricFunction1 *
                 rtb_uDLookupTable_cs) * QS_InnerRateLoop_U.vN_fpsKF +
                (rtb_Add6_i * rtb_uDLookupTable_cs +
                 QS_InnerRateLoop_ConstB.TrigonometricFunction1 * rtb_Gain3) *
                QS_InnerRateLoop_U.vE_fpsKF) +
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
  rtb_Saturation8 = static_cast<real32_T>(sqrt(static_cast<real_T>(rtb_Add5_e *
    rtb_Add5_e + rtb_Add6_i * rtb_Add6_i)));

  /* Saturate: '<Root>/Saturation8' */
  if (rtb_Saturation8 > 6.5F) {
    rtb_Saturation8 = 6.5F;
  }

  /* End of Saturate: '<Root>/Saturation8' */

  /* Lookup_n-D: '<S65>/1-D Lookup Table2' incorporates:
   *  Constant: '<S65>/Constant'
   *  Lookup_n-D: '<S64>/1-D Lookup Table2'
   *  Saturate: '<Root>/Saturation8'
   */
  bpIndices[0U] = plook_u32ff_bincg(rtb_Saturation8,
    QS_InnerRateLoop_ConstP.pooled8, 4U, &rtb_Abs_i);
  fractions[0U] = rtb_Abs_i;
  for (iU = 0; iU < 36; iU++) {
    bpIndices[1U] = plook_u32ff_bincg(QS_InnerRateLoop_ConstP.pooled21[iU],
      QS_InnerRateLoop_ConstP.pooled21, 35U, &rtb_Abs_i);
    fractions[1U] = rtb_Abs_i;
    rtb_uDLookupTable2[iU] = intrp2d_fu32fl_pw(bpIndices, fractions,
      QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_g, 5U);
  }

  /* Lookup_n-D: '<S64>/1-D Lookup Table2' incorporates:
   *  Constant: '<S64>/Constant'
   *  Lookup_n-D: '<S65>/1-D Lookup Table2'
   *  Saturate: '<Root>/Saturation8'
   */
  bpIndices_0[0U] = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U,
    &rtb_Abs_i);
  fractions_0[0U] = rtb_Abs_i;
  for (iU = 0; iU < 16; iU++) {
    bpIndices_0[1U] = plook_u32ff_evencg(QS_InnerRateLoop_ConstP.pooled22[iU],
      1.0F, 1.0F, 15U, &rtb_Abs_i);
    fractions_0[1U] = rtb_Abs_i;
    rtb_uDLookupTable2_k5[iU] = intrp2d_fu32fl_pw(bpIndices_0, fractions_0,
      QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_b, 5U);
  }

  /* Product: '<S12>/Divide4' incorporates:
   *  Product: '<S12>/Divide9'
   */
  rtb_Gain2 = QS_InnerRateLoop_ConstB.TrigonometricFunction1 *
    QS_InnerRateLoop_ConstB.TrigonometricFunction5;

  /* Sum: '<S12>/Add7' incorporates:
   *  Inport: '<Root>/vD_fps (KF)'
   *  Inport: '<Root>/vE_fps (KF)'
   *  Inport: '<Root>/vN_fps (KF)'
   *  Product: '<S12>/Divide10'
   *  Product: '<S12>/Divide12'
   *  Product: '<S12>/Divide14'
   *  Product: '<S12>/Divide16'
   *  Product: '<S12>/Divide4'
   *  Product: '<S12>/Divide5'
   *  Product: '<S12>/Divide9'
   *  Sum: '<S12>/Add2'
   *  Sum: '<S12>/Add4'
   */
  rtb_uDLookupTable_cs = ((rtb_Gain2 * rtb_Gain3 +
    QS_InnerRateLoop_ConstB.TrigonometricFunction4 * rtb_uDLookupTable_cs) *
    QS_InnerRateLoop_U.vN_fpsKF + (rtb_Gain2 * rtb_uDLookupTable_cs -
    QS_InnerRateLoop_ConstB.TrigonometricFunction4 * rtb_Gain3) *
    QS_InnerRateLoop_U.vE_fpsKF) +
    QS_InnerRateLoop_ConstB.TrigonometricFunction1 *
    QS_InnerRateLoop_ConstB.TrigonometricFunction2 * QS_InnerRateLoop_U.vD_fpsKF;

  /* SignalConversion generated from: '<S65>/Product' incorporates:
   *  Product: '<S11>/Divide'
   *  Product: '<S11>/Divide2'
   *  Product: '<S11>/Divide6'
   *  Sum: '<S11>/Add5'
   */
  rtb_Add5_0[0] = (rtb_Product_of *
                   QS_InnerRateLoop_ConstB.TrigonometricFunction3 * rtb_Add5_e +
                   rtb_Product_of *
                   QS_InnerRateLoop_ConstB.TrigonometricFunction6 * rtb_Add6_i)
    + -rtb_Sum3_f * rtb_uDLookupTable_cs;

  /* Product: '<S11>/Divide1' incorporates:
   *  Product: '<S11>/Divide7'
   */
  rtb_Product_of = rtb_DiscreteTimeIntegrator1_g * rtb_Sum3_f;

  /* SignalConversion generated from: '<S65>/Product' incorporates:
   *  Product: '<S11>/Divide1'
   *  Product: '<S11>/Divide11'
   *  Product: '<S11>/Divide13'
   *  Product: '<S11>/Divide15'
   *  Product: '<S11>/Divide3'
   *  Product: '<S11>/Divide7'
   *  Product: '<S11>/Divide8'
   *  Sum: '<S11>/Add1'
   *  Sum: '<S11>/Add3'
   *  Sum: '<S11>/Add6'
   */
  rtb_Add5_0[1] = ((rtb_Product_of *
                    QS_InnerRateLoop_ConstB.TrigonometricFunction3 -
                    rtb_derivativecutofffrequency_a *
                    QS_InnerRateLoop_ConstB.TrigonometricFunction6) * rtb_Add5_e
                   + (rtb_Product_of *
                      QS_InnerRateLoop_ConstB.TrigonometricFunction6 +
                      rtb_derivativecutofffrequency_a *
                      QS_InnerRateLoop_ConstB.TrigonometricFunction3) *
                   rtb_Add6_i) + rtb_TrigonometricFunction6 *
    rtb_uDLookupTable_cs;

  /* Product: '<S11>/Divide4' incorporates:
   *  Product: '<S11>/Divide9'
   */
  rtb_Product_of = rtb_derivativecutofffrequency_a * rtb_Sum3_f;

  /* SignalConversion generated from: '<S65>/Product' incorporates:
   *  Product: '<S11>/Divide10'
   *  Product: '<S11>/Divide12'
   *  Product: '<S11>/Divide14'
   *  Product: '<S11>/Divide16'
   *  Product: '<S11>/Divide4'
   *  Product: '<S11>/Divide5'
   *  Product: '<S11>/Divide9'
   *  Sum: '<S11>/Add2'
   *  Sum: '<S11>/Add4'
   *  Sum: '<S11>/Add7'
   */
  rtb_Add5_0[2] = ((rtb_Product_of *
                    QS_InnerRateLoop_ConstB.TrigonometricFunction3 +
                    rtb_DiscreteTimeIntegrator1_g *
                    QS_InnerRateLoop_ConstB.TrigonometricFunction6) * rtb_Add5_e
                   + (rtb_Product_of *
                      QS_InnerRateLoop_ConstB.TrigonometricFunction6 -
                      rtb_DiscreteTimeIntegrator1_g *
                      QS_InnerRateLoop_ConstB.TrigonometricFunction3) *
                   rtb_Add6_i) + rtb_Product * rtb_uDLookupTable_cs;

  /* DiscreteIntegrator: '<S14>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/psi_rad'
   */
  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOA_g != 0) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_m =
      QS_InnerRateLoop_U.psi_rad;
  }

  /* Trigonometry: '<S73>/Trigonometric Function3' incorporates:
   *  DiscreteIntegrator: '<S14>/Discrete-Time Integrator'
   */
  rtb_derivativecutofffrequency_a = static_cast<real32_T>(cos(static_cast<real_T>
    (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_m)));

  /* Trigonometry: '<S73>/Trigonometric Function6' incorporates:
   *  DiscreteIntegrator: '<S14>/Discrete-Time Integrator'
   */
  rtb_TrigonometricFunction6 = static_cast<real32_T>(sin(static_cast<real_T>
    (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_m)));

  /* DiscreteIntegrator: '<S95>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_n <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_a = 0.0F;
  }

  /* Lookup_n-D: '<S93>/1-D Lookup Table2' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* DiscreteIntegrator: '<S70>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   *  Inport: '<Root>/pos North (KF)'
   */
  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOA_d != 0) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_mo =
      QS_InnerRateLoop_U.posNorthKF;
  }

  if ((QS_InnerRateLoop_U.engage &&
       (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_a <= 0)) || (tmp_0 &&
       (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_a == 1))) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_mo =
      QS_InnerRateLoop_U.posNorthKF;
  }

  /* Product: '<S93>/Product' incorporates:
   *  DiscreteIntegrator: '<S70>/Discrete-Time Integrator'
   *  Inport: '<Root>/pos North (KF)'
   *  Lookup_n-D: '<S93>/1-D Lookup Table2'
   *  Sum: '<S78>/Sum'
   */
  rtb_Product = (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_mo -
                 QS_InnerRateLoop_U.posNorthKF) * intrp1d_fu32fl_pw(bpIdx,
    rtb_Abs_i, QS_InnerRateLoop_ConstP.pooled23);

  /* Sum: '<S78>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S95>/Discrete-Time Integrator'
   */
  rtb_Sum1_j_idx_0 = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_a +
    rtb_Product;

  /* Saturate: '<S71>/Saturation' */
  if (rtb_Sum1_j_idx_0 > 1.0F) {
    rtb_Sum1_j_idx_0 = 1.0F;
  } else if (rtb_Sum1_j_idx_0 < -1.0F) {
    rtb_Sum1_j_idx_0 = -1.0F;
  }

  /* Gain: '<S71>/Gain2' incorporates:
   *  Saturate: '<S71>/Saturation'
   */
  rtb_Gain2 = 0.0F * rtb_Sum1_j_idx_0;

  /* DiscreteIntegrator: '<S92>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_o <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_h = 0.0F;
  }

  /* Lookup_n-D: '<S90>/1-D Lookup Table2' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* DiscreteIntegrator: '<S70>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/engage'
   *  Inport: '<Root>/pos East (KF)'
   */
  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_IC_LO_k != 0) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_l =
      QS_InnerRateLoop_U.posEastKF;
  }

  if ((QS_InnerRateLoop_U.engage &&
       (QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevR_b <= 0)) || (tmp_0 &&
       (QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevR_b == 1))) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_l =
      QS_InnerRateLoop_U.posEastKF;
  }

  /* Product: '<S90>/Product' incorporates:
   *  DiscreteIntegrator: '<S70>/Discrete-Time Integrator1'
   *  Inport: '<Root>/pos East (KF)'
   *  Lookup_n-D: '<S90>/1-D Lookup Table2'
   *  Sum: '<S77>/Sum'
   */
  rtb_Product_ng = (QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_l -
                    QS_InnerRateLoop_U.posEastKF) * intrp1d_fu32fl_pw(bpIdx,
    rtb_Abs_i, QS_InnerRateLoop_ConstP.pooled23);

  /* Sum: '<S77>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S92>/Discrete-Time Integrator'
   */
  rtb_Sum1_j_idx_0 = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_h +
    rtb_Product_ng;

  /* Saturate: '<S71>/Saturation1' */
  if (rtb_Sum1_j_idx_0 > 1.0F) {
    rtb_Sum1_j_idx_0 = 1.0F;
  } else if (rtb_Sum1_j_idx_0 < -1.0F) {
    rtb_Sum1_j_idx_0 = -1.0F;
  }

  /* Gain: '<S71>/Gain3' incorporates:
   *  Saturate: '<S71>/Saturation1'
   */
  rtb_Gain3 = 0.0F * rtb_Sum1_j_idx_0;

  /* Logic: '<S76>/Logical Operator' incorporates:
   *  Delay: '<S76>/Delay'
   *  Inport: '<Root>/engage'
   *  RelationalOperator: '<S76>/Relational Operator'
   */
  rtb_LogicalOperator = ((QS_InnerRateLoop_U.engage !=
    QS_InnerRateLoop_DW.Delay_DSTATE) || QS_InnerRateLoop_U.engage);

  /* DiscreteIntegrator: '<S89>/Discrete-Time Integrator' */
  if (rtb_LogicalOperator &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_b <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_ca = 0.0F;
  }

  /* Lookup_n-D: '<S88>/1-D Lookup Table2' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* DiscreteIntegrator: '<S70>/Discrete-Time Integrator2' incorporates:
   *  Inport: '<Root>/engage'
   *  Inport: '<Root>/pos Down (KF)'
   */
  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_IC_LOAD != 0) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_DSTATE =
      QS_InnerRateLoop_U.posDownKF;
  }

  if ((QS_InnerRateLoop_U.engage &&
       (QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_PrevRes <= 0)) || (tmp_0 &&
       (QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_PrevRes == 1))) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_DSTATE =
      QS_InnerRateLoop_U.posDownKF;
  }

  /* Product: '<S88>/Product' incorporates:
   *  DiscreteIntegrator: '<S70>/Discrete-Time Integrator2'
   *  Inport: '<Root>/pos Down (KF)'
   *  Lookup_n-D: '<S88>/1-D Lookup Table2'
   *  Sum: '<S76>/Sum'
   */
  rtb_Product_ee = (QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_DSTATE -
                    QS_InnerRateLoop_U.posDownKF) * intrp1d_fu32fl_pw(bpIdx,
    rtb_Abs_i, QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_k);

  /* Sum: '<S76>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S89>/Discrete-Time Integrator'
   */
  rtb_Sum1_j_idx_0 = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_ca +
    rtb_Product_ee;

  /* Saturate: '<S71>/Saturation2' */
  if (rtb_Sum1_j_idx_0 > 3.0F) {
    rtb_Sum1_j_idx_0 = 3.0F;
  } else if (rtb_Sum1_j_idx_0 < -3.0F) {
    rtb_Sum1_j_idx_0 = -3.0F;
  }

  /* Gain: '<S71>/Gain4' incorporates:
   *  Saturate: '<S71>/Saturation2'
   */
  rtb_Gain4 = 0.0F * rtb_Sum1_j_idx_0;

  /* Outputs for Enabled SubSystem: '<Root>/Enabled Subsystem1' incorporates:
   *  EnablePort: '<S7>/Enable'
   */
  if (tmp_0) {
    /* SignalConversion generated from: '<S7>/col' incorporates:
     *  Inport: '<Root>/input_col'
     */
    QS_InnerRateLoop_B.col = QS_InnerRateLoop_U.input_col;
  }

  /* End of Outputs for SubSystem: '<Root>/Enabled Subsystem1' */

  /* Sum: '<Root>/Sum3' incorporates:
   *  Inport: '<Root>/input_col'
   */
  rtb_wcmd = QS_InnerRateLoop_U.input_col - QS_InnerRateLoop_B.col;

  /* DeadZone: '<Root>/Dead Zone' */
  if (rtb_wcmd > 0.1F) {
    rtb_Sum1_j_idx_0 = rtb_wcmd - 0.1F;
  } else if (rtb_wcmd >= -0.1F) {
    rtb_Sum1_j_idx_0 = 0.0F;
  } else {
    rtb_Sum1_j_idx_0 = rtb_wcmd - -0.1F;
  }

  /* Gain: '<Root>/wcmd' incorporates:
   *  DeadZone: '<Root>/Dead Zone'
   */
  rtb_wcmd = -6.0F * rtb_Sum1_j_idx_0;

  /* Lookup_n-D: '<S43>/1-D Lookup Table2' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Product: '<S73>/Divide4' incorporates:
   *  Product: '<S73>/Divide9'
   */
  rtb_Product_b_tmp = QS_InnerRateLoop_ConstB.TrigonometricFunction1_g *
    QS_InnerRateLoop_ConstB.TrigonometricFunction5_j;

  /* Product: '<S43>/Product' incorporates:
   *  Lookup_n-D: '<S43>/1-D Lookup Table2'
   *  Product: '<S73>/Divide10'
   *  Product: '<S73>/Divide12'
   *  Product: '<S73>/Divide14'
   *  Product: '<S73>/Divide16'
   *  Product: '<S73>/Divide4'
   *  Product: '<S73>/Divide5'
   *  Product: '<S73>/Divide9'
   *  Sum: '<S40>/Sum'
   *  Sum: '<S71>/Sum2'
   *  Sum: '<S73>/Add2'
   *  Sum: '<S73>/Add4'
   *  Sum: '<S73>/Add7'
   *  UnitDelay: '<S40>/Unit Delay'
   */
  rtb_Product_of = (((((rtb_Product_b_tmp * rtb_derivativecutofffrequency_a +
                        QS_InnerRateLoop_ConstB.TrigonometricFunction4_km *
                        rtb_TrigonometricFunction6) * rtb_Gain2 +
                       (rtb_Product_b_tmp * rtb_TrigonometricFunction6 -
                        QS_InnerRateLoop_ConstB.TrigonometricFunction4_km *
                        rtb_derivativecutofffrequency_a) * rtb_Gain3) +
                      QS_InnerRateLoop_ConstB.TrigonometricFunction1_g *
                      QS_InnerRateLoop_ConstB.TrigonometricFunction2_ak *
                      rtb_Gain4) + rtb_wcmd) -
                    QS_InnerRateLoop_DW.UnitDelay_DSTATE_aw) / intrp1d_fu32fl_pw
    (bpIdx, rtb_Abs_i, QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_j);

  /* DiscreteIntegrator: '<S9>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if ((QS_InnerRateLoop_U.engage &&
       (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_ot <= 0)) || (tmp_0 &&
       (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_ot == 1))) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_p = 0.0F;
  }

  /* DiscreteIntegrator: '<S33>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevR_g <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_o = 0.0F;
  }

  rtb_DiscreteTimeIntegrator1_g =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_o;

  /* Switch: '<S24>/Switch' incorporates:
   *  Constant: '<S24>/Constant'
   *  DiscreteIntegrator: '<S9>/Discrete-Time Integrator'
   *  Gain: '<S71>/Gain1'
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage) {
    rtb_Sum1_j_idx_0 = 0.0F *
      QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_p;
  } else {
    rtb_Sum1_j_idx_0 = 0.0F;
  }

  /* Sum: '<S24>/Sum3' incorporates:
   *  DiscreteIntegrator: '<S33>/Discrete-Time Integrator1'
   *  Switch: '<S24>/Switch'
   */
  rtb_Sum3_f = rtb_Sum1_j_idx_0 +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_o;

  /* Gain: '<S31>/derivative cutoff frequency 1' incorporates:
   *  DiscreteIntegrator: '<S31>/Discrete-Time Integrator'
   *  Gain: '<S31>/derivative cutoff frequency '
   *  Sum: '<S31>/Sum1'
   */
  rtb_derivativecutofffrequency_f = (rtb_Sum3_f -
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_b) * 100.0F;

  /* DiscreteIntegrator: '<S8>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if ((QS_InnerRateLoop_U.engage &&
       (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_ek <= 0)) || (tmp_0 &&
       (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_ek == 1))) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_b3 = 0.0F;
  }

  /* DiscreteIntegrator: '<S49>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevR_j <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_m = 0.0F;
  }

  rtb_DiscreteTimeIntegrator1_d =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_m;

  /* Switch: '<S26>/Switch' incorporates:
   *  Constant: '<S26>/Constant'
   *  DiscreteIntegrator: '<S8>/Discrete-Time Integrator'
   *  Gain: '<S71>/Gain'
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage) {
    rtb_Sum1_j_idx_0 = 0.0F *
      QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_b3;
  } else {
    rtb_Sum1_j_idx_0 = 0.0F;
  }

  /* Sum: '<S26>/Sum4' incorporates:
   *  DiscreteIntegrator: '<S49>/Discrete-Time Integrator1'
   *  Switch: '<S26>/Switch'
   */
  rtb_Sum4_f = rtb_Sum1_j_idx_0 +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_m;

  /* Gain: '<S47>/derivative cutoff frequency 1' incorporates:
   *  DiscreteIntegrator: '<S47>/Discrete-Time Integrator'
   *  Gain: '<S47>/derivative cutoff frequency '
   *  Sum: '<S47>/Sum1'
   */
  rtb_derivativecutofffrequency_0 = (rtb_Sum4_f -
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_ou) * 100.0F;

  /* DiscreteIntegrator: '<S60>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_bp <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE = 0.0;
  }

  /* Gain: '<S60>/Gain' incorporates:
   *  DiscreteIntegrator: '<S60>/Discrete-Time Integrator'
   */
  rtb_Gain_on = 400.0 * QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE;

  /* Switch: '<S27>/Switch2' incorporates:
   *  Constant: '<S27>/Constant1'
   */
  if (rtb_Gain_on >= 100.0) {
    /* Switch: '<S27>/Switch' incorporates:
     *  DiscreteIntegrator: '<S14>/Discrete-Time Integrator'
     *  DiscreteIntegrator: '<S59>/Discrete-Time Integrator'
     *  Gain: '<S59>/derivative cutoff frequency 1'
     *  Sum: '<S59>/Sum1'
     */
    rtb_Switch2 = (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_m -
                   QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_mr) * 100.0F;
  } else {
    rtb_Switch2 = 0.0F;
  }

  /* End of Switch: '<S27>/Switch2' */

  /* DiscreteIntegrator: '<S41>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_m <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_f = 0.0F;
  }

  /* Lookup_n-D: '<S38>/1-D Lookup Table2' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* DiscreteIntegrator: '<S40>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_nq <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_fs = 0.0F;
  }

  /* Product: '<S38>/Product' incorporates:
   *  DiscreteIntegrator: '<S40>/Discrete-Time Integrator'
   *  Lookup_n-D: '<S38>/1-D Lookup Table2'
   *  Sum: '<S25>/Sum2'
   */
  rtb_Product_ib = (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_fs -
                    rtb_uDLookupTable_cs) * intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_o);

  /* Sum: '<S25>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S41>/Discrete-Time Integrator'
   */
  rtb_Sum1_ky = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_f +
    rtb_Product_ib;

  /* Lookup_n-D: '<S25>/1-D Lookup Table' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_uDLookupTable_cs = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable_tableData);

  /* Lookup_n-D: '<S25>/1-D Lookup Table1' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_uDLookupTable1 = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable1_tableData);

  /* DiscreteIntegrator: '<S42>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_by <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_c4 = 0.0F;
  }

  rtb_DiscreteTimeIntegrator_bs =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_c4;

  /* DiscreteIntegrator: '<S34>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_es <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_o2 = 0.0F;
  }

  /* Lookup_n-D: '<S29>/1-D Lookup Table2' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Product: '<S29>/Product' incorporates:
   *  Inport: '<Root>/phi_rad'
   *  Lookup_n-D: '<S29>/1-D Lookup Table2'
   *  Sum: '<S24>/Sum'
   */
  rtb_Product_jr = (rtb_Sum3_f - QS_InnerRateLoop_U.phi_rad) * intrp1d_fu32fl_pw
    (bpIdx, rtb_Abs_i, QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_a);

  /* Lookup_n-D: '<S28>/1-D Lookup Table2' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Sum: '<S24>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S34>/Discrete-Time Integrator'
   *  Gain: '<S31>/derivative cutoff frequency 1'
   *  Inport: '<Root>/p_rps'
   *  Lookup_n-D: '<S28>/1-D Lookup Table2'
   *  Product: '<S28>/Product'
   *  Sum: '<S24>/Sum2'
   */
  rtb_Sum1_cq = (rtb_derivativecutofffrequency_f - QS_InnerRateLoop_U.p_rps) *
    intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
                      QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_fu) +
    (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_o2 + rtb_Product_jr);

  /* Lookup_n-D: '<S24>/1-D Lookup Table' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_uDLookupTable = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable_tableData_l);

  /* Lookup_n-D: '<S24>/1-D Lookup Table1' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_uDLookupTable1_p = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable1_tableData_n);

  /* DiscreteIntegrator: '<S35>/Discrete-Time Integrator' */
  rtb_DiscreteTimeIntegrator_g =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_bw;

  /* DiscreteIntegrator: '<S50>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_d <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_j = 0.0F;
  }

  /* Lookup_n-D: '<S45>/1-D Lookup Table2' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Product: '<S45>/Product' incorporates:
   *  Inport: '<Root>/theta_rad'
   *  Lookup_n-D: '<S45>/1-D Lookup Table2'
   *  Sum: '<S26>/Sum'
   */
  rtb_Product_n = (rtb_Sum4_f - QS_InnerRateLoop_U.theta_rad) *
    intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
                      QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_ex);

  /* Lookup_n-D: '<S44>/1-D Lookup Table2' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Sum: '<S26>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S50>/Discrete-Time Integrator'
   *  Gain: '<S47>/derivative cutoff frequency 1'
   *  Inport: '<Root>/q_rps'
   *  Lookup_n-D: '<S44>/1-D Lookup Table2'
   *  Product: '<S44>/Product'
   *  Sum: '<S26>/Sum2'
   */
  rtb_Sum1_c0 = (rtb_derivativecutofffrequency_0 - QS_InnerRateLoop_U.q_rps) *
    intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
                      QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_c) +
    (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_j + rtb_Product_n);

  /* Lookup_n-D: '<S26>/1-D Lookup Table' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_uDLookupTable_e = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable_tableData_m);

  /* Lookup_n-D: '<S26>/1-D Lookup Table1' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_uDLookupTable1_g0 = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable1_tableData_d);

  /* DiscreteIntegrator: '<S51>/Discrete-Time Integrator' */
  rtb_DiscreteTimeIntegrator_fi =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_di;

  /* DiscreteIntegrator: '<S61>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_j <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_l = 0.0F;
  }

  /* Lookup_n-D: '<S55>/1-D Lookup Table2' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Switch: '<S27>/Switch1' incorporates:
   *  Constant: '<S27>/Constant'
   */
  if (rtb_Gain_on >= 100.0) {
    /* Sum: '<S27>/Sum1' incorporates:
     *  DiscreteIntegrator: '<S14>/Discrete-Time Integrator'
     *  Inport: '<Root>/psi_rad'
     */
    rtb_Sum1_e = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_m -
      QS_InnerRateLoop_U.psi_rad;

    /* Switch: '<S63>/Switch' incorporates:
     *  Constant: '<S63>/Constant1'
     *  Constant: '<S63>/Constant2'
     *  Gain: '<S63>/Gain1'
     *  Switch: '<S63>/Switch1'
     */
    if (rtb_Sum1_e >= 0.0F) {
      rtb_Abs = rtb_Sum1_e;
      iU = 1;
    } else {
      rtb_Abs = -rtb_Sum1_e;
      iU = -1;
    }

    /* End of Switch: '<S63>/Switch' */

    /* Product: '<S63>/Product' incorporates:
     *  Constant: '<S63>/Constant'
     *  Gain: '<S63>/Gain'
     *  Product: '<S63>/Divide'
     *  Rounding: '<S63>/Rounding Function'
     *  Sum: '<S63>/Subtract'
     *  Switch: '<S63>/Switch1'
     */
    rtb_Abs = (rtb_Abs - static_cast<real32_T>(floor(static_cast<real_T>(rtb_Abs
      * 0.159154937F))) * 6.28318548F) * static_cast<real32_T>(iU);

    /* Switch: '<S54>/Switch' incorporates:
     *  Abs: '<S54>/Abs'
     */
    if (static_cast<real32_T>(fabs(static_cast<real_T>(rtb_Abs))) > 3.14159274F)
    {
      /* Switch: '<S54>/Switch1' incorporates:
       *  Constant: '<S54>/Constant1'
       *  Constant: '<S54>/Constant2'
       *  Sum: '<S54>/Add'
       *  Sum: '<S54>/Subtract'
       */
      if (rtb_Abs >= 0.0F) {
        rtb_Abs -= 6.28318548F;
      } else {
        rtb_Abs += 6.28318548F;
      }

      /* End of Switch: '<S54>/Switch1' */
    }

    /* End of Switch: '<S54>/Switch' */
  } else {
    rtb_Abs = 0.0F;
  }

  /* End of Switch: '<S27>/Switch1' */

  /* Product: '<S55>/Product' incorporates:
   *  Lookup_n-D: '<S55>/1-D Lookup Table2'
   */
  rtb_Product_ear = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_au) * rtb_Abs;

  /* Lookup_n-D: '<S57>/1-D Lookup Table2' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Sum: '<S27>/Sum5' incorporates:
   *  DiscreteIntegrator: '<S61>/Discrete-Time Integrator'
   *  Inport: '<Root>/r_rps'
   *  Lookup_n-D: '<S57>/1-D Lookup Table2'
   *  Product: '<S57>/Product'
   *  Sum: '<S27>/Sum3'
   */
  rtb_Abs = (rtb_Switch2 - QS_InnerRateLoop_U.r_rps) * intrp1d_fu32fl_pw(bpIdx,
    rtb_Abs_i, QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_c0) +
    (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_l + rtb_Product_ear);

  /* Lookup_n-D: '<S27>/1-D Lookup Table' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_uDLookupTable2_n = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable_tableData_h);

  /* Lookup_n-D: '<S27>/1-D Lookup Table1' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_uDLookupTable_d = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable1_tableData_f);

  /* DiscreteIntegrator: '<S62>/Discrete-Time Integrator' */
  rtb_Sum1_e = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_hv;

  /* Sum: '<S5>/Sum' */
  rtb_Product_hg[0] = rtb_Product_of;

  /* Gain: '<S32>/derivative cutoff frequency 1' incorporates:
   *  DiscreteIntegrator: '<S32>/Discrete-Time Integrator'
   *  Gain: '<S31>/derivative cutoff frequency 1'
   *  Gain: '<S32>/derivative cutoff frequency '
   *  Sum: '<S32>/Sum1'
   */
  rtb_Product_b_tmp = (rtb_derivativecutofffrequency_f -
                       QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_hq) *
    100.0F;

  /* Sum: '<S5>/Sum' incorporates:
   *  Gain: '<S32>/derivative cutoff frequency 1'
   */
  rtb_Product_hg[1] = rtb_Product_b_tmp;

  /* Sum: '<S26>/Sum3' incorporates:
   *  DiscreteIntegrator: '<S48>/Discrete-Time Integrator'
   *  Gain: '<S47>/derivative cutoff frequency 1'
   *  Gain: '<S48>/derivative cutoff frequency '
   *  Gain: '<S48>/derivative cutoff frequency 1'
   *  Sum: '<S48>/Sum1'
   */
  rtb_Product_b_tmp_0 = (rtb_derivativecutofffrequency_0 -
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_mk) * 100.0F;

  /* Sum: '<S5>/Sum' incorporates:
   *  Gain: '<S48>/derivative cutoff frequency 1'
   *  Sum: '<S26>/Sum3'
   */
  rtb_Product_hg[2] = rtb_Product_b_tmp_0;

  /* Gain: '<S58>/derivative cutoff frequency 1' incorporates:
   *  DiscreteIntegrator: '<S58>/Discrete-Time Integrator'
   *  Gain: '<S58>/derivative cutoff frequency '
   *  Sum: '<S58>/Sum1'
   */
  rtb_Switch2 = (rtb_Switch2 -
                 QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_d) * 100.0F;

  /* Sum: '<S5>/Sum' incorporates:
   *  DiscreteIntegrator: '<S35>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S42>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S51>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S62>/Discrete-Time Integrator'
   *  Gain: '<S58>/derivative cutoff frequency 1'
   *  Product: '<S35>/Product1'
   *  Product: '<S42>/Product1'
   *  Product: '<S51>/Product1'
   *  Product: '<S62>/Product1'
   *  Sum: '<S35>/Sum1'
   *  Sum: '<S42>/Sum1'
   *  Sum: '<S51>/Sum1'
   *  Sum: '<S62>/Sum1'
   */
  rtb_Product_hg[3] = rtb_Switch2;
  rtb_Sum1_f[0] = rtb_Sum1_ky * rtb_uDLookupTable_cs / rtb_uDLookupTable1 +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_c4;
  rtb_Sum1_f[1] = rtb_Sum1_cq * rtb_uDLookupTable / rtb_uDLookupTable1_p +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_bw;
  rtb_Sum1_f[2] = rtb_Sum1_c0 * rtb_uDLookupTable_e / rtb_uDLookupTable1_g0 +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_di;
  rtb_Sum1_f[3] = rtb_Abs * rtb_uDLookupTable2_n / rtb_uDLookupTable_d +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_hv;

  /* SignalConversion generated from: '<S65>/Product' incorporates:
   *  Inport: '<Root>/p_rps'
   *  Inport: '<Root>/phi_rad'
   *  Inport: '<Root>/psi_rad'
   *  Inport: '<Root>/q_rps'
   *  Inport: '<Root>/r_rps'
   *  Inport: '<Root>/theta_rad'
   */
  rtb_Add5_0[3] = QS_InnerRateLoop_U.p_rps;
  rtb_Add5_0[4] = QS_InnerRateLoop_U.q_rps;
  rtb_Add5_0[5] = QS_InnerRateLoop_U.r_rps;
  rtb_Add5_0[6] = QS_InnerRateLoop_U.phi_rad;
  rtb_Add5_0[7] = QS_InnerRateLoop_U.theta_rad;
  rtb_Add5_0[8] = QS_InnerRateLoop_U.psi_rad;

  /* Sum: '<S5>/Sum' incorporates:
   *  Lookup_n-D: '<S65>/1-D Lookup Table2'
   *  Product: '<S65>/Product'
   */
  for (iU = 0; iU < 4; iU++) {
    rtb_Sum1_j_idx_0 = 0.0F;
    tmp = 0;
    for (i = 0; i < 9; i++) {
      rtb_Sum1_j_idx_0 += rtb_uDLookupTable2[tmp + iU] * rtb_Add5_0[i];
      tmp += 4;
    }

    rtb_Product_b[iU] = (rtb_Product_hg[iU] + rtb_Sum1_f[iU]) - rtb_Sum1_j_idx_0;
  }

  /* Product: '<S64>/Product' incorporates:
   *  Lookup_n-D: '<S64>/1-D Lookup Table2'
   */
  rtb_Abs_i = rtb_Product_b[1];
  rtb_Add5 = rtb_Product_b[0];
  rtb_Add6 = rtb_Product_b[2];
  rtb_Sum1_eo = rtb_Product_b[3];
  for (iU = 0; iU < 4; iU++) {
    rtb_Product_hg[iU] = ((rtb_uDLookupTable2_k5[iU + 4] * rtb_Abs_i +
      rtb_uDLookupTable2_k5[iU] * rtb_Add5) + rtb_uDLookupTable2_k5[iU + 8] *
                          rtb_Add6) + rtb_uDLookupTable2_k5[iU + 12] *
      rtb_Sum1_eo;
  }

  /* End of Product: '<S64>/Product' */

  /* Outputs for Enabled SubSystem: '<Root>/Enabled Subsystem Grab and Freeze Value Upon Engagement' incorporates:
   *  EnablePort: '<S6>/Enable'
   */
  if (tmp_0) {
    /* Saturate: '<Root>/Saturation' incorporates:
     *  Inport: '<Root>/mixer_in_throttle'
     */
    if (QS_InnerRateLoop_U.mixer_in_throttle > 2.0F) {
      /* SignalConversion generated from: '<S6>/In1' */
      QS_InnerRateLoop_B.In1[0] = 2.0F;
    } else if (QS_InnerRateLoop_U.mixer_in_throttle < -1.0F) {
      /* SignalConversion generated from: '<S6>/In1' */
      QS_InnerRateLoop_B.In1[0] = -1.0F;
    } else {
      /* SignalConversion generated from: '<S6>/In1' */
      QS_InnerRateLoop_B.In1[0] = QS_InnerRateLoop_U.mixer_in_throttle;
    }

    /* End of Saturate: '<Root>/Saturation' */

    /* Saturate: '<Root>/Saturation1' incorporates:
     *  Inport: '<Root>/mixer_in_y'
     */
    if (QS_InnerRateLoop_U.mixer_in_y > 2.0F) {
      /* SignalConversion generated from: '<S6>/In1' */
      QS_InnerRateLoop_B.In1[1] = 2.0F;
    } else if (QS_InnerRateLoop_U.mixer_in_y < -2.0F) {
      /* SignalConversion generated from: '<S6>/In1' */
      QS_InnerRateLoop_B.In1[1] = -2.0F;
    } else {
      /* SignalConversion generated from: '<S6>/In1' */
      QS_InnerRateLoop_B.In1[1] = QS_InnerRateLoop_U.mixer_in_y;
    }

    /* End of Saturate: '<Root>/Saturation1' */

    /* Saturate: '<Root>/Saturation2' incorporates:
     *  Inport: '<Root>/mixer_in_x'
     */
    if (QS_InnerRateLoop_U.mixer_in_x > 2.0F) {
      /* SignalConversion generated from: '<S6>/In1' */
      QS_InnerRateLoop_B.In1[2] = 2.0F;
    } else if (QS_InnerRateLoop_U.mixer_in_x < -2.0F) {
      /* SignalConversion generated from: '<S6>/In1' */
      QS_InnerRateLoop_B.In1[2] = -2.0F;
    } else {
      /* SignalConversion generated from: '<S6>/In1' */
      QS_InnerRateLoop_B.In1[2] = QS_InnerRateLoop_U.mixer_in_x;
    }

    /* End of Saturate: '<Root>/Saturation2' */

    /* Saturate: '<Root>/Saturation3' incorporates:
     *  Inport: '<Root>/mixer_in_z'
     */
    if (QS_InnerRateLoop_U.mixer_in_z > 2.0F) {
      /* SignalConversion generated from: '<S6>/In1' */
      QS_InnerRateLoop_B.In1[3] = 2.0F;
    } else if (QS_InnerRateLoop_U.mixer_in_z < -2.0F) {
      /* SignalConversion generated from: '<S6>/In1' */
      QS_InnerRateLoop_B.In1[3] = -2.0F;
    } else {
      /* SignalConversion generated from: '<S6>/In1' */
      QS_InnerRateLoop_B.In1[3] = QS_InnerRateLoop_U.mixer_in_z;
    }

    /* End of Saturate: '<Root>/Saturation3' */
  }

  /* End of Outputs for SubSystem: '<Root>/Enabled Subsystem Grab and Freeze Value Upon Engagement' */

  /* DiscreteIntegrator: '<S66>/Discrete-Time Integrator' */
  rtb_Add5 = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_co;

  /* Sum: '<S66>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S66>/Discrete-Time Integrator'
   *  Gain: '<S66>/Gain'
   */
  rtb_DeadZone3 = 6.66666651F * rtb_Product_hg[0] +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_co;

  /* DiscreteIntegrator: '<S67>/Discrete-Time Integrator' */
  rtb_Add6 = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_i;

  /* Sum: '<S67>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S67>/Discrete-Time Integrator'
   *  Gain: '<S67>/Gain'
   */
  rtb_Sum1_eo = 6.66666651F * rtb_Product_hg[1] +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_i;

  /* DiscreteIntegrator: '<S68>/Discrete-Time Integrator' */
  rtb_Add7_n = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_ch;

  /* Sum: '<S68>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S68>/Discrete-Time Integrator'
   *  Gain: '<S68>/Gain'
   */
  rtb_Sum1_b3 = 6.66666651F * rtb_Product_hg[2] +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_ch;

  /* DiscreteIntegrator: '<S69>/Discrete-Time Integrator' */
  rtb_DiscreteTimeIntegrator_dh =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_a5;

  /* Sum: '<S69>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S69>/Discrete-Time Integrator'
   *  Gain: '<S69>/Gain'
   */
  rtb_Sum1_c = 0.364F * rtb_Product_hg[3] +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_a5;

  /* Sum: '<Root>/Sum1' */
  rtb_Sum1_j_idx_0 = QS_InnerRateLoop_B.In1[0] + rtb_DeadZone3;
  rtb_Sum1_j_idx_1 = QS_InnerRateLoop_B.In1[1] + rtb_Sum1_eo;
  rtb_Sum1_j_idx_2 = QS_InnerRateLoop_B.In1[2] + rtb_Sum1_b3;
  rtb_Sum1_j_idx_3 = QS_InnerRateLoop_B.In1[3] + rtb_Sum1_c;

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

  /* Abs: '<S15>/Abs' incorporates:
   *  Sum: '<S15>/Sum'
   */
  rtb_Abs_i = static_cast<real32_T>(fabs(static_cast<real_T>(rtb_Sum1_j_idx_0 -
    rtb_Abs_i)));

  /* Switch: '<S41>/Switch' incorporates:
   *  Constant: '<S41>/Constant'
   *  Constant: '<S97>/Constant'
   *  Lookup_n-D: '<S39>/1-D Lookup Table2'
   *  Product: '<S39>/Product'
   *  RelationalOperator: '<S97>/Compare'
   */
  if (rtb_Abs_i > 0.0F) {
    rtb_Product_ib = 0.0F;
  } else {
    /* Lookup_n-D: '<S39>/1-D Lookup Table2' incorporates:
     *  Saturate: '<Root>/Saturation8'
     */
    bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
    rtb_Product_ib *= intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
      QS_InnerRateLoop_ConstP.pooled9);
  }

  /* End of Switch: '<S41>/Switch' */

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

  /* Abs: '<S16>/Abs' incorporates:
   *  Sum: '<S16>/Sum'
   */
  rtb_Abs_i = static_cast<real32_T>(fabs(static_cast<real_T>(rtb_Sum1_j_idx_1 -
    rtb_Abs_i)));

  /* Switch: '<S34>/Switch' incorporates:
   *  Constant: '<S34>/Constant'
   *  Constant: '<S98>/Constant'
   *  Lookup_n-D: '<S30>/1-D Lookup Table2'
   *  Product: '<S30>/Product'
   *  RelationalOperator: '<S98>/Compare'
   */
  if (rtb_Abs_i > 0.0F) {
    rtb_Product_jr = 0.0F;
  } else {
    /* Lookup_n-D: '<S30>/1-D Lookup Table2' incorporates:
     *  Saturate: '<Root>/Saturation8'
     */
    bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
    rtb_Product_jr *= intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
      QS_InnerRateLoop_ConstP.pooled7);
  }

  /* End of Switch: '<S34>/Switch' */

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

  /* Abs: '<S17>/Abs' incorporates:
   *  Sum: '<S17>/Sum'
   */
  rtb_Abs_i = static_cast<real32_T>(fabs(static_cast<real_T>(rtb_Sum1_j_idx_2 -
    rtb_Abs_i)));

  /* Switch: '<S50>/Switch' incorporates:
   *  Constant: '<S50>/Constant'
   *  Constant: '<S99>/Constant'
   *  Lookup_n-D: '<S46>/1-D Lookup Table2'
   *  Product: '<S46>/Product'
   *  RelationalOperator: '<S99>/Compare'
   */
  if (rtb_Abs_i > 0.0F) {
    rtb_Product_n = 0.0F;
  } else {
    /* Lookup_n-D: '<S46>/1-D Lookup Table2' incorporates:
     *  Saturate: '<Root>/Saturation8'
     */
    bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
    rtb_Product_n *= intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
      QS_InnerRateLoop_ConstP.uDLookupTable2_tableData);
  }

  /* End of Switch: '<S50>/Switch' */

  /* Saturate: '<Root>/Saturation5' */
  if (rtb_Sum1_j_idx_3 > 1.0F) {
    rtb_Abs_i = 1.0F;
  } else if (rtb_Sum1_j_idx_3 < -1.0F) {
    rtb_Abs_i = -1.0F;
  } else {
    rtb_Abs_i = rtb_Sum1_j_idx_3;
  }

  /* End of Saturate: '<Root>/Saturation5' */

  /* Outport: '<Root>/mixer_z' */
  QS_InnerRateLoop_Y.mixer_z = rtb_Abs_i;

  /* Abs: '<S18>/Abs' incorporates:
   *  Sum: '<S18>/Sum'
   */
  rtb_Abs_i = static_cast<real32_T>(fabs(static_cast<real_T>(rtb_Abs_i -
    rtb_Sum1_j_idx_3)));

  /* Switch: '<S61>/Switch' incorporates:
   *  Constant: '<S100>/Constant'
   *  Constant: '<S61>/Constant'
   *  Lookup_n-D: '<S56>/1-D Lookup Table2'
   *  Product: '<S56>/Product'
   *  RelationalOperator: '<S100>/Compare'
   */
  if (rtb_Abs_i > 0.0F) {
    rtb_Product_ear = 0.0F;
  } else {
    /* Lookup_n-D: '<S56>/1-D Lookup Table2' incorporates:
     *  Saturate: '<Root>/Saturation8'
     */
    bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
    rtb_Product_ear *= intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
      QS_InnerRateLoop_ConstP.pooled7);
  }

  /* End of Switch: '<S61>/Switch' */

  /* Outport: '<Root>/yaw_sweep' */
  QS_InnerRateLoop_Y.yaw_sweep = rtb_Sum1_c;

  /* Outport: '<Root>/pitch_sweep' */
  QS_InnerRateLoop_Y.pitch_sweep = rtb_Sum1_b3;

  /* Outport: '<Root>/roll_sweep' */
  QS_InnerRateLoop_Y.roll_sweep = rtb_Sum1_eo;

  /* Outport: '<Root>/coll_sweep' */
  QS_InnerRateLoop_Y.coll_sweep = rtb_DeadZone3;

  /* Product: '<S62>/Product' incorporates:
   *  Product: '<S62>/Product2'
   *  Sum: '<S62>/Sum'
   *  Sum: '<S62>/Sum2'
   *  UnitDelay: '<S62>/Unit Delay'
   */
  rtb_uDLookupTable2_n *= (rtb_uDLookupTable_d - rtb_uDLookupTable2_n) * rtb_Abs
    / rtb_uDLookupTable_d - QS_InnerRateLoop_DW.UnitDelay_DSTATE_of;

  /* Outport: '<Root>/vz_cmd' incorporates:
   *  DiscreteIntegrator: '<S40>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_Y.vz_cmd =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_fs;

  /* Switch: '<S60>/Switch' incorporates:
   *  Constant: '<S60>/Constant'
   *  Constant: '<S60>/Constant1'
   *  Inport: '<Root>/engage'
   *  RelationalOperator: '<S60>/Relational Operator'
   */
  if (rtb_Gain_on >= 100.0) {
    rtb_Gain_on = 0.0;
  } else {
    rtb_Gain_on = QS_InnerRateLoop_U.engage;
  }

  /* End of Switch: '<S60>/Switch' */

  /* Sum: '<S59>/Sum' incorporates:
   *  DiscreteIntegrator: '<S14>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S59>/Discrete-Time Integrator'
   */
  rtb_Sum1_eo = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_m -
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_mr;

  /* Outport: '<Root>/theta_cmd' */
  QS_InnerRateLoop_Y.theta_cmd = rtb_Sum4_f;

  /* Sum: '<S72>/Add7' incorporates:
   *  DiscreteIntegrator: '<S8>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S9>/Discrete-Time Integrator'
   *  Product: '<S72>/Divide11'
   *  Product: '<S72>/Divide12'
   *  Product: '<S72>/Divide2'
   */
  rtb_Sum1_b3 = (QS_InnerRateLoop_ConstB.TrigonometricFunction1_m *
                 QS_InnerRateLoop_ConstB.TrigonometricFunction2_a * rtb_wcmd +
                 QS_InnerRateLoop_ConstB.TrigonometricFunction4_k *
                 QS_InnerRateLoop_ConstB.TrigonometricFunction2_a *
                 QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_p) +
    -QS_InnerRateLoop_ConstB.TrigonometricFunction5_f *
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_b3;

  /* Outport: '<Root>/Vdown_cmd' */
  QS_InnerRateLoop_Y.Vdown_cmd = rtb_Sum1_b3;

  /* Gain: '<S70>/Gain' incorporates:
   *  Inport: '<Root>/psi_rad'
   */
  rtb_DeadZone3 = 0.0174532924F * QS_InnerRateLoop_U.psi_rad;

  /* Trigonometry: '<S72>/Trigonometric Function3' */
  rtb_uDLookupTable_d = static_cast<real32_T>(cos(static_cast<real_T>
    (rtb_DeadZone3)));

  /* Trigonometry: '<S72>/Trigonometric Function6' */
  rtb_DeadZone3 = static_cast<real32_T>(sin(static_cast<real_T>(rtb_DeadZone3)));

  /* Product: '<S72>/Divide4' incorporates:
   *  Product: '<S72>/Divide9'
   */
  rtb_Abs_i = QS_InnerRateLoop_ConstB.TrigonometricFunction1_m *
    QS_InnerRateLoop_ConstB.TrigonometricFunction5_f;

  /* Product: '<S72>/Divide1' incorporates:
   *  Product: '<S72>/Divide7'
   */
  rtb_Abs = QS_InnerRateLoop_ConstB.TrigonometricFunction4_k *
    QS_InnerRateLoop_ConstB.TrigonometricFunction5_f;

  /* Sum: '<S72>/Add5' incorporates:
   *  DiscreteIntegrator: '<S8>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S9>/Discrete-Time Integrator'
   *  Product: '<S72>/Divide'
   *  Product: '<S72>/Divide1'
   *  Product: '<S72>/Divide13'
   *  Product: '<S72>/Divide14'
   *  Product: '<S72>/Divide3'
   *  Product: '<S72>/Divide4'
   *  Product: '<S72>/Divide5'
   *  Sum: '<S72>/Add1'
   *  Sum: '<S72>/Add2'
   */
  rtb_Sum1_c = ((rtb_Abs_i * rtb_uDLookupTable_d +
                 QS_InnerRateLoop_ConstB.TrigonometricFunction4_k *
                 rtb_DeadZone3) * rtb_wcmd + (rtb_Abs * rtb_uDLookupTable_d -
    QS_InnerRateLoop_ConstB.TrigonometricFunction1_m * rtb_DeadZone3) *
                QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_p) +
    QS_InnerRateLoop_ConstB.TrigonometricFunction2_a * rtb_uDLookupTable_d *
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_b3;

  /* Outport: '<Root>/Vnorth_cmd' */
  QS_InnerRateLoop_Y.Vnorth_cmd = rtb_Sum1_c;

  /* Sum: '<S72>/Add6' incorporates:
   *  DiscreteIntegrator: '<S8>/Discrete-Time Integrator'
   *  DiscreteIntegrator: '<S9>/Discrete-Time Integrator'
   *  Product: '<S72>/Divide10'
   *  Product: '<S72>/Divide15'
   *  Product: '<S72>/Divide16'
   *  Product: '<S72>/Divide6'
   *  Product: '<S72>/Divide7'
   *  Product: '<S72>/Divide8'
   *  Product: '<S72>/Divide9'
   *  Sum: '<S72>/Add3'
   *  Sum: '<S72>/Add4'
   */
  rtb_wcmd = ((rtb_Abs_i * rtb_DeadZone3 -
               QS_InnerRateLoop_ConstB.TrigonometricFunction4_k *
               rtb_uDLookupTable_d) * rtb_wcmd + (rtb_Abs * rtb_DeadZone3 +
    QS_InnerRateLoop_ConstB.TrigonometricFunction1_m * rtb_uDLookupTable_d) *
              QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_p) +
    QS_InnerRateLoop_ConstB.TrigonometricFunction2_a * rtb_DeadZone3 *
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_b3;

  /* Outport: '<Root>/Veast_cmd' */
  QS_InnerRateLoop_Y.Veast_cmd = rtb_wcmd;

  /* Lookup_n-D: '<S83>/1-D Lookup Table2' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Product: '<S83>/Product' incorporates:
   *  DiscreteIntegrator: '<S8>/Discrete-Time Integrator'
   *  Lookup_n-D: '<S83>/1-D Lookup Table2'
   *  Product: '<S73>/Divide'
   *  Product: '<S73>/Divide2'
   *  Product: '<S73>/Divide6'
   *  Sum: '<S73>/Add5'
   *  Sum: '<S75>/Sum'
   *  Sum: '<S75>/Sum4'
   */
  rtb_Abs = ((((QS_InnerRateLoop_ConstB.TrigonometricFunction2_ak *
                rtb_derivativecutofffrequency_a * rtb_Gain2 +
                QS_InnerRateLoop_ConstB.TrigonometricFunction2_ak *
                rtb_TrigonometricFunction6 * rtb_Gain3) +
               -QS_InnerRateLoop_ConstB.TrigonometricFunction5_j * rtb_Gain4) +
              QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_b3) - rtb_Add5_e)
    * intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
                        QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_kg);

  /* Lookup_n-D: '<S84>/1-D Lookup Table2' incorporates:
   *  Saturate: '<Root>/Saturation8'
   *  Switch: '<S85>/Switch'
   */
  bpIdx = plook_u32ff_evenxg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Switch: '<S85>/Switch' incorporates:
   *  Lookup_n-D: '<S84>/1-D Lookup Table2'
   *  Product: '<S84>/Product'
   */
  rtb_Add5_e = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_f) * rtb_Abs;

  /* DiscreteIntegrator: '<S85>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_h2 <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_ny = 0.0F;
  }

  /* Sum: '<S75>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S85>/Discrete-Time Integrator'
   */
  rtb_DeadZone3 = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_ny + rtb_Abs;

  /* Lookup_n-D: '<S75>/1-D Lookup Table' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_uDLookupTable_d = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable_tableData_d);

  /* Lookup_n-D: '<S75>/1-D Lookup Table1' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_Abs = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable1_tableData_m);

  /* DiscreteIntegrator: '<S86>/Discrete-Time Integrator' */
  rtb_Sum4_f = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_g;

  /* Lookup_n-D: '<S52>/1-D Lookup Table2' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Lookup_n-D: '<S53>/1-D Lookup Table2' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx_0 = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U,
    &rtb_Sum1_j_idx_3);

  /* Gain: '<S75>/Gain5' incorporates:
   *  DiscreteIntegrator: '<S86>/Discrete-Time Integrator'
   *  Product: '<S86>/Product1'
   *  Sum: '<S86>/Sum1'
   */
  rtb_Sum1_j_idx_0 = -(rtb_DeadZone3 * rtb_uDLookupTable_d / rtb_Abs +
                       QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_g);

  /* Saturate: '<Root>/Saturation9' */
  if (rtb_Sum1_j_idx_0 > 0.349040151F) {
    rtb_Sum1_j_idx_0 = 0.349040151F;
  } else if (rtb_Sum1_j_idx_0 < -0.349040151F) {
    rtb_Sum1_j_idx_0 = -0.349040151F;
  }

  /* Product: '<S53>/Product' incorporates:
   *  Lookup_n-D: '<S52>/1-D Lookup Table2'
   *  Lookup_n-D: '<S53>/1-D Lookup Table2'
   *  Product: '<S52>/Product'
   *  Saturate: '<Root>/Saturation9'
   *  Sum: '<S49>/Sum'
   *  Sum: '<S49>/Sum1'
   *  UnitDelay: '<S49>/Unit Delay'
   *  UnitDelay: '<S49>/Unit Delay1'
   */
  rtb_Sum1_j_idx_1 = ((rtb_Sum1_j_idx_0 -
                       QS_InnerRateLoop_DW.UnitDelay1_DSTATE_c) *
                      intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.pooled36) - QS_InnerRateLoop_DW.UnitDelay_DSTATE_j) *
    intrp1d_fu32fl_pw(bpIdx_0, rtb_Sum1_j_idx_3,
                      QS_InnerRateLoop_ConstP.pooled37);

  /* Product: '<S86>/Product' incorporates:
   *  Product: '<S86>/Product2'
   *  Sum: '<S86>/Sum'
   *  Sum: '<S86>/Sum2'
   *  UnitDelay: '<S86>/Unit Delay'
   */
  rtb_Sum1_j_idx_2 = ((rtb_Abs - rtb_uDLookupTable_d) * rtb_DeadZone3 / rtb_Abs
                      - QS_InnerRateLoop_DW.UnitDelay_DSTATE_oc) *
    rtb_uDLookupTable_d;

  /* Outport: '<Root>/phi_cmd' */
  QS_InnerRateLoop_Y.phi_cmd = rtb_Sum3_f;

  /* Lookup_n-D: '<S79>/1-D Lookup Table2' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Product: '<S73>/Divide1' incorporates:
   *  Product: '<S73>/Divide7'
   */
  rtb_Sum3_f = QS_InnerRateLoop_ConstB.TrigonometricFunction4_km *
    QS_InnerRateLoop_ConstB.TrigonometricFunction5_j;

  /* Product: '<S79>/Product' incorporates:
   *  DiscreteIntegrator: '<S9>/Discrete-Time Integrator'
   *  Lookup_n-D: '<S79>/1-D Lookup Table2'
   *  Product: '<S73>/Divide1'
   *  Product: '<S73>/Divide11'
   *  Product: '<S73>/Divide13'
   *  Product: '<S73>/Divide15'
   *  Product: '<S73>/Divide3'
   *  Product: '<S73>/Divide7'
   *  Product: '<S73>/Divide8'
   *  Sum: '<S73>/Add1'
   *  Sum: '<S73>/Add3'
   *  Sum: '<S73>/Add6'
   *  Sum: '<S74>/Sum'
   *  Sum: '<S74>/Sum4'
   */
  rtb_Abs = (((((rtb_Sum3_f * rtb_derivativecutofffrequency_a -
                 QS_InnerRateLoop_ConstB.TrigonometricFunction1_g *
                 rtb_TrigonometricFunction6) * rtb_Gain2 + (rtb_Sum3_f *
    rtb_TrigonometricFunction6 +
    QS_InnerRateLoop_ConstB.TrigonometricFunction1_g *
    rtb_derivativecutofffrequency_a) * rtb_Gain3) +
               QS_InnerRateLoop_ConstB.TrigonometricFunction4_km *
               QS_InnerRateLoop_ConstB.TrigonometricFunction2_ak * rtb_Gain4) +
              QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_p) - rtb_Add6_i)
    * intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
                        QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_n);

  /* Lookup_n-D: '<S80>/1-D Lookup Table2' incorporates:
   *  Saturate: '<Root>/Saturation8'
   *  Switch: '<S81>/Switch'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Switch: '<S81>/Switch' incorporates:
   *  Lookup_n-D: '<S80>/1-D Lookup Table2'
   *  Product: '<S80>/Product'
   */
  rtb_Add6_i = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.pooled9) * rtb_Abs;

  /* DiscreteIntegrator: '<S81>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_dc <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cf = 0.0F;
  }

  /* Sum: '<S74>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S81>/Discrete-Time Integrator'
   */
  rtb_DeadZone3 = QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cf + rtb_Abs;

  /* Lookup_n-D: '<S74>/1-D Lookup Table' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_uDLookupTable_d = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable_tableData_hs);

  /* Lookup_n-D: '<S74>/1-D Lookup Table1' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);
  rtb_Abs = intrp1d_fu32fl_pw(bpIdx, rtb_Abs_i,
    QS_InnerRateLoop_ConstP.uDLookupTable1_tableData_p);

  /* DiscreteIntegrator: '<S82>/Discrete-Time Integrator' */
  rtb_derivativecutofffrequency_a =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cc;

  /* Lookup_n-D: '<S36>/1-D Lookup Table2' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evenxg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Sum: '<S82>/Sum1' incorporates:
   *  DiscreteIntegrator: '<S82>/Discrete-Time Integrator'
   *  Product: '<S82>/Product1'
   */
  rtb_Sum1_j_idx_0 = rtb_DeadZone3 * rtb_uDLookupTable_d / rtb_Abs +
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cc;

  /* Saturate: '<Root>/Saturation10' */
  if (rtb_Sum1_j_idx_0 > 0.349040151F) {
    rtb_Sum1_j_idx_0 = 0.349040151F;
  } else if (rtb_Sum1_j_idx_0 < -0.349040151F) {
    rtb_Sum1_j_idx_0 = -0.349040151F;
  }

  /* Sum: '<S33>/Sum' incorporates:
   *  Lookup_n-D: '<S36>/1-D Lookup Table2'
   *  Product: '<S36>/Product'
   *  Saturate: '<Root>/Saturation10'
   *  Sum: '<S33>/Sum1'
   *  UnitDelay: '<S33>/Unit Delay'
   *  UnitDelay: '<S33>/Unit Delay1'
   */
  rtb_TrigonometricFunction6 = (rtb_Sum1_j_idx_0 -
    QS_InnerRateLoop_DW.UnitDelay1_DSTATE_a) * intrp1d_fu32fl_pw(bpIdx,
    rtb_Abs_i, QS_InnerRateLoop_ConstP.pooled36) -
    QS_InnerRateLoop_DW.UnitDelay_DSTATE_g;

  /* Lookup_n-D: '<S37>/1-D Lookup Table2' incorporates:
   *  Saturate: '<Root>/Saturation8'
   */
  bpIdx = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U, &rtb_Abs_i);

  /* Outport: '<Root>/Vdown_corr' */
  QS_InnerRateLoop_Y.Vdown_corr = rtb_Gain4;

  /* Lookup_n-D: '<S87>/1-D Lookup Table2' incorporates:
   *  Saturate: '<Root>/Saturation8'
   *  Switch: '<S89>/Switch'
   */
  bpIdx_0 = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U,
    &rtb_Sum1_j_idx_3);

  /* Switch: '<S89>/Switch' incorporates:
   *  Lookup_n-D: '<S87>/1-D Lookup Table2'
   *  Product: '<S87>/Product'
   */
  rtb_Product_ee *= intrp1d_fu32fl_pw(bpIdx_0, rtb_Sum1_j_idx_3,
    QS_InnerRateLoop_ConstP.uDLookupTable2_tableData_e);

  /* Outport: '<Root>/Veast_corr' */
  QS_InnerRateLoop_Y.Veast_corr = rtb_Gain3;

  /* Switch: '<S92>/Switch' incorporates:
   *  Constant: '<S92>/Constant'
   *  Lookup_n-D: '<S91>/1-D Lookup Table2'
   *  Product: '<S91>/Product'
   */
  if (QS_InnerRateLoop_ConstB.LogicalOperator) {
    rtb_Gain3 = 0.0F;
  } else {
    /* Lookup_n-D: '<S91>/1-D Lookup Table2' incorporates:
     *  Saturate: '<Root>/Saturation8'
     */
    bpIdx_0 = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U,
      &rtb_Sum1_j_idx_3);
    rtb_Gain3 = intrp1d_fu32fl_pw(bpIdx_0, rtb_Sum1_j_idx_3,
      QS_InnerRateLoop_ConstP.pooled13) * rtb_Product_ng;
  }

  /* End of Switch: '<S92>/Switch' */

  /* Outport: '<Root>/Vnorth_corr' */
  QS_InnerRateLoop_Y.Vnorth_corr = rtb_Gain2;

  /* Switch: '<S95>/Switch' incorporates:
   *  Constant: '<S95>/Constant'
   *  Lookup_n-D: '<S94>/1-D Lookup Table2'
   *  Product: '<S94>/Product'
   */
  if (QS_InnerRateLoop_ConstB.LogicalOperator_a) {
    rtb_Saturation8 = 0.0F;
  } else {
    /* Lookup_n-D: '<S94>/1-D Lookup Table2' incorporates:
     *  Saturate: '<Root>/Saturation8'
     */
    bpIdx_0 = plook_u32ff_evencg(rtb_Saturation8, 0.0F, 1.75F, 4U,
      &rtb_Sum1_j_idx_3);
    rtb_Saturation8 = intrp1d_fu32fl_pw(bpIdx_0, rtb_Sum1_j_idx_3,
      QS_InnerRateLoop_ConstP.pooled13) * rtb_Product;
  }

  /* End of Switch: '<S95>/Switch' */

  /* Outport: '<Root>/psi_cmd' incorporates:
   *  DiscreteIntegrator: '<S14>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_Y.psi_cmd =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_m;

  /* DiscreteIntegrator: '<S19>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_l <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_gh = 0.0F;
  }

  /* DiscreteIntegrator: '<S33>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_f <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_dn = 0.0F;
  }

  /* DiscreteIntegrator: '<S49>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if (QS_InnerRateLoop_U.engage &&
      (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_f0 <= 0)) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_nq = 0.0F;
  }

  /* DeadZone: '<S14>/Dead Zone3' incorporates:
   *  Inport: '<Root>/input_ped'
   */
  if (QS_InnerRateLoop_U.input_ped > 0.05F) {
    rtb_Sum1_j_idx_0 = QS_InnerRateLoop_U.input_ped - 0.05F;
  } else if (QS_InnerRateLoop_U.input_ped >= -0.05F) {
    rtb_Sum1_j_idx_0 = 0.0F;
  } else {
    rtb_Sum1_j_idx_0 = QS_InnerRateLoop_U.input_ped - -0.05F;
  }

  /* Sum: '<S96>/Sum' incorporates:
   *  DeadZone: '<S14>/Dead Zone3'
   *  Gain: '<S14>/rcmd'
   *  UnitDelay: '<S96>/Unit Delay'
   */
  rtb_Product = 1.91972077F * rtb_Sum1_j_idx_0 -
    QS_InnerRateLoop_DW.UnitDelay_DSTATE_k;

  /* DiscreteIntegrator: '<S96>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  if ((QS_InnerRateLoop_U.engage &&
       (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_bv <= 0)) || (tmp_0 &&
       (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_bv == 1))) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_gy = 0.0F;
  }

  /* Outport: '<Root>/TrajectoryON' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_Y.TrajectoryON = QS_InnerRateLoop_U.engage;

  /* Update for UnitDelay: '<S20>/Unit Delay' incorporates:
   *  DiscreteIntegrator: '<S20>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_n;

  /* Update for UnitDelay: '<S21>/Unit Delay' incorporates:
   *  DiscreteIntegrator: '<S21>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_d =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_o;

  /* Update for UnitDelay: '<S22>/Unit Delay' incorporates:
   *  DiscreteIntegrator: '<S22>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_a =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_e;

  /* Update for DiscreteIntegrator: '<S22>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S22>/Gain'
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOADI = 0U;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_e += 0.5F * rtb_Sum_nx *
    0.0025F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRese = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S23>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S23>/Gain'
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_c += 0.5F * rtb_Sum1_mn *
    0.0025F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_e = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S21>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S21>/Gain'
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOA_m = 0U;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_o += 0.5F * rtb_Sum_j *
    0.0025F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_h = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for UnitDelay: '<S19>/Unit Delay1' incorporates:
   *  DiscreteIntegrator: '<S19>/Discrete-Time Integrator1'
   */
  QS_InnerRateLoop_DW.UnitDelay1_DSTATE =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTATE;

  /* Update for UnitDelay: '<S19>/Unit Delay' incorporates:
   *  DiscreteIntegrator: '<S19>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_ak =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_gh;

  /* Update for DiscreteIntegrator: '<S19>/Discrete-Time Integrator1' incorporates:
   *  DiscreteIntegrator: '<S19>/Discrete-Time Integrator'
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_IC_LOAD = 0U;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTATE += 0.0025F *
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_gh;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevRes = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S20>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S20>/Gain'
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_n += static_cast<real32_T>
    (0.5 * rtb_Sum * 0.0025);
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_p = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for UnitDelay: '<S23>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_o = rtb_DiscreteTimeIntegrator_ff;

  /* Update for UnitDelay: '<S9>/Unit Delay' incorporates:
   *  DiscreteIntegrator: '<S9>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_c =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_p;

  /* Update for UnitDelay: '<S8>/Unit Delay' incorporates:
   *  DiscreteIntegrator: '<S8>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_dr =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_b3;

  /* Update for DiscreteIntegrator: '<S14>/Discrete-Time Integrator' incorporates:
   *  DiscreteIntegrator: '<S96>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOA_g = 0U;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_m += 0.0025F *
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_gy;

  /* Update for DiscreteIntegrator: '<S95>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_a += 0.0025F *
    rtb_Saturation8;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_n = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOA_d = 0U;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_mo += 0.0025F * rtb_Sum1_c;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_a = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S92>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_h += 0.0025F * rtb_Gain3;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_o = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator1' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_IC_LO_k = 0U;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_l += 0.0025F * rtb_wcmd;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevR_b = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for Delay: '<S76>/Delay' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.Delay_DSTATE = QS_InnerRateLoop_U.engage;

  /* Update for DiscreteIntegrator: '<S89>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_ca += 0.0025F *
    rtb_Product_ee;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_b = static_cast<int8_T>
    (rtb_LogicalOperator);

  /* Update for DiscreteIntegrator: '<S70>/Discrete-Time Integrator2' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_IC_LOAD = 0U;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_DSTATE += 0.0025F * rtb_Sum1_b3;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_PrevRes = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for UnitDelay: '<S40>/Unit Delay' incorporates:
   *  DiscreteIntegrator: '<S40>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_aw =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_fs;

  /* Update for DiscreteIntegrator: '<S32>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_hq += rtb_Product_b_tmp *
    0.0025F;

  /* Update for DiscreteIntegrator: '<S31>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_b +=
    rtb_derivativecutofffrequency_f * 0.0025F;

  /* Update for DiscreteIntegrator: '<S9>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S9>/Gain'
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_p += 10.0F * rtb_Sum_lu *
    0.0025F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_ot = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S33>/Discrete-Time Integrator1' incorporates:
   *  DiscreteIntegrator: '<S33>/Discrete-Time Integrator'
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_o += 0.0025F *
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_dn;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevR_g = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S48>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_mk += rtb_Product_b_tmp_0 *
    0.0025F;

  /* Update for DiscreteIntegrator: '<S47>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_ou +=
    rtb_derivativecutofffrequency_0 * 0.0025F;

  /* Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S8>/Gain'
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_b3 += 10.0F * rtb_Sum_hb *
    0.0025F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_ek = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator1' incorporates:
   *  DiscreteIntegrator: '<S49>/Discrete-Time Integrator'
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_DSTAT_m += 0.0025F *
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_nq;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevR_j = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S58>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_d += rtb_Switch2 * 0.0025F;

  /* Update for DiscreteIntegrator: '<S59>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S59>/derivative cutoff frequency '
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_mr += 100.0F * rtb_Sum1_eo *
    0.0025F;

  /* Update for DiscreteIntegrator: '<S60>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE += 0.0025 * rtb_Gain_on;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_bp = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S41>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_f += 0.0025F *
    rtb_Product_ib;
  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_f > 0.1F) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_f = 0.1F;
  } else if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_f < -0.1F) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_f = -0.1F;
  }

  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_m = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* End of Update for DiscreteIntegrator: '<S41>/Discrete-Time Integrator' */

  /* Update for DiscreteIntegrator: '<S40>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_fs += 0.0025F *
    rtb_Product_of;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_nq = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S42>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   *  Product: '<S42>/Product'
   *  Product: '<S42>/Product2'
   *  Sum: '<S42>/Sum'
   *  Sum: '<S42>/Sum2'
   *  UnitDelay: '<S42>/Unit Delay'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_c4 += ((rtb_uDLookupTable1 -
    rtb_uDLookupTable_cs) * rtb_Sum1_ky / rtb_uDLookupTable1 -
    QS_InnerRateLoop_DW.UnitDelay_DSTATE_h) * rtb_uDLookupTable_cs * 0.0025F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_by = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S34>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_o2 += 0.0025F *
    rtb_Product_jr;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_es = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S35>/Discrete-Time Integrator' incorporates:
   *  Product: '<S35>/Product'
   *  Product: '<S35>/Product2'
   *  Sum: '<S35>/Sum'
   *  Sum: '<S35>/Sum2'
   *  UnitDelay: '<S35>/Unit Delay'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_bw += ((rtb_uDLookupTable1_p
    - rtb_uDLookupTable) * rtb_Sum1_cq / rtb_uDLookupTable1_p -
    QS_InnerRateLoop_DW.UnitDelay_DSTATE_b) * rtb_uDLookupTable * 0.0025F;

  /* Update for DiscreteIntegrator: '<S50>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_j += 0.0025F * rtb_Product_n;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_d = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S51>/Discrete-Time Integrator' incorporates:
   *  Product: '<S51>/Product'
   *  Product: '<S51>/Product2'
   *  Sum: '<S51>/Sum'
   *  Sum: '<S51>/Sum2'
   *  UnitDelay: '<S51>/Unit Delay'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_di += ((rtb_uDLookupTable1_g0
    - rtb_uDLookupTable_e) * rtb_Sum1_c0 / rtb_uDLookupTable1_g0 -
    QS_InnerRateLoop_DW.UnitDelay_DSTATE_eo) * rtb_uDLookupTable_e * 0.0025F;

  /* Update for DiscreteIntegrator: '<S61>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_l += 0.0025F *
    rtb_Product_ear;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_j = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S62>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_hv += 0.0025F *
    rtb_uDLookupTable2_n;

  /* Update for DiscreteIntegrator: '<S66>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S66>/Gain1'
   *  Gain: '<S66>/Gain2'
   *  Sum: '<S66>/Sum'
   *  UnitDelay: '<S66>/Unit Delay'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_co += (-5.66666651F *
    rtb_Product_hg[0] - QS_InnerRateLoop_DW.UnitDelay_DSTATE_f) * 100.0F *
    0.0025F;

  /* Update for DiscreteIntegrator: '<S67>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S67>/Gain1'
   *  Gain: '<S67>/Gain2'
   *  Sum: '<S67>/Sum'
   *  UnitDelay: '<S67>/Unit Delay'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_i += (-5.66666651F *
    rtb_Product_hg[1] - QS_InnerRateLoop_DW.UnitDelay_DSTATE_av) * 100.0F *
    0.0025F;

  /* Update for DiscreteIntegrator: '<S68>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S68>/Gain1'
   *  Gain: '<S68>/Gain2'
   *  Sum: '<S68>/Sum'
   *  UnitDelay: '<S68>/Unit Delay'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_ch += (-5.66666651F *
    rtb_Product_hg[2] - QS_InnerRateLoop_DW.UnitDelay_DSTATE_dj) * 100.0F *
    0.0025F;

  /* Update for DiscreteIntegrator: '<S69>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S69>/Gain1'
   *  Gain: '<S69>/Gain2'
   *  Sum: '<S69>/Sum'
   *  UnitDelay: '<S69>/Unit Delay'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_a5 += (0.636F *
    rtb_Product_hg[3] - QS_InnerRateLoop_DW.UnitDelay_DSTATE_e) * 5.46F *
    0.0025F;

  /* Update for UnitDelay: '<S66>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_f = rtb_Add5;

  /* Update for UnitDelay: '<S67>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_av = rtb_Add6;

  /* Update for UnitDelay: '<S68>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_dj = rtb_Add7_n;

  /* Update for UnitDelay: '<S69>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_e = rtb_DiscreteTimeIntegrator_dh;

  /* Update for UnitDelay: '<S62>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_of = rtb_Sum1_e;

  /* Update for UnitDelay: '<S51>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_eo = rtb_DiscreteTimeIntegrator_fi;

  /* Update for UnitDelay: '<S35>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_b = rtb_DiscreteTimeIntegrator_g;

  /* Update for UnitDelay: '<S42>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_h = rtb_DiscreteTimeIntegrator_bs;

  /* Update for DiscreteIntegrator: '<S85>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_ny += 0.0025F * rtb_Add5_e;
  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_ny > 0.0174520072F) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_ny = 0.0174520072F;
  } else if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_ny <
             -0.0174520072F) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_ny = -0.0174520072F;
  }

  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_h2 = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* End of Update for DiscreteIntegrator: '<S85>/Discrete-Time Integrator' */

  /* Update for DiscreteIntegrator: '<S86>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTATE_g += 0.0025F *
    rtb_Sum1_j_idx_2;

  /* Update for UnitDelay: '<S49>/Unit Delay1' */
  QS_InnerRateLoop_DW.UnitDelay1_DSTATE_c = rtb_DiscreteTimeIntegrator1_d;

  /* Update for UnitDelay: '<S49>/Unit Delay' incorporates:
   *  DiscreteIntegrator: '<S49>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_j =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_nq;

  /* Update for UnitDelay: '<S86>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_oc = rtb_Sum4_f;

  /* Update for DiscreteIntegrator: '<S81>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cf += 0.0025F * rtb_Add6_i;
  if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cf > 0.0174520072F) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cf = 0.0174520072F;
  } else if (QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cf <
             -0.0174520072F) {
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cf = -0.0174520072F;
  }

  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_dc = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* End of Update for DiscreteIntegrator: '<S81>/Discrete-Time Integrator' */

  /* Update for DiscreteIntegrator: '<S82>/Discrete-Time Integrator' incorporates:
   *  Product: '<S82>/Product'
   *  Product: '<S82>/Product2'
   *  Sum: '<S82>/Sum'
   *  Sum: '<S82>/Sum2'
   *  UnitDelay: '<S82>/Unit Delay'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_cc += ((rtb_Abs -
    rtb_uDLookupTable_d) * rtb_DeadZone3 / rtb_Abs -
    QS_InnerRateLoop_DW.UnitDelay_DSTATE_gi) * rtb_uDLookupTable_d * 0.0025F;

  /* Update for UnitDelay: '<S33>/Unit Delay1' */
  QS_InnerRateLoop_DW.UnitDelay1_DSTATE_a = rtb_DiscreteTimeIntegrator1_g;

  /* Update for UnitDelay: '<S33>/Unit Delay' incorporates:
   *  DiscreteIntegrator: '<S33>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_g =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_dn;

  /* Update for UnitDelay: '<S82>/Unit Delay' */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_gi = rtb_derivativecutofffrequency_a;

  /* Update for DiscreteIntegrator: '<S19>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S19>/Gain'
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_gh += 2.828F * rtb_Gain_ir *
    0.0025F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_l = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S33>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   *  Lookup_n-D: '<S37>/1-D Lookup Table2'
   *  Product: '<S37>/Product'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_dn += intrp1d_fu32fl_pw(bpIdx,
    rtb_Abs_i, QS_InnerRateLoop_ConstP.pooled37) * rtb_TrigonometricFunction6 *
    0.0025F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_f = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for DiscreteIntegrator: '<S49>/Discrete-Time Integrator' incorporates:
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_nq += 0.0025F *
    rtb_Sum1_j_idx_1;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_f0 = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);

  /* Update for UnitDelay: '<S96>/Unit Delay' incorporates:
   *  DiscreteIntegrator: '<S96>/Discrete-Time Integrator'
   */
  QS_InnerRateLoop_DW.UnitDelay_DSTATE_k =
    QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_gy;

  /* Update for DiscreteIntegrator: '<S96>/Discrete-Time Integrator' incorporates:
   *  Gain: '<S96>/Gain'
   *  Inport: '<Root>/engage'
   */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_DSTAT_gy += 10.0F * rtb_Product *
    0.0025F;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_bv = static_cast<int8_T>
    (QS_InnerRateLoop_U.engage);
}

/* Model initialize function */
void QS_InnerRateLoop::initialize()
{
  /* InitializeConditions for DiscreteIntegrator: '<S22>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRese = 2;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOADI = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S23>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_e = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S21>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_h = 2;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOA_m = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S19>/Discrete-Time Integrator1' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevRes = 2;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_IC_LOAD = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S20>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_p = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S14>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOA_g = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S95>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_n = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S70>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_a = 2;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_IC_LOA_d = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S92>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_o = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S70>/Discrete-Time Integrator1' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevR_b = 2;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_IC_LO_k = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S89>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_b = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S70>/Discrete-Time Integrator2' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_PrevRes = 2;
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator2_IC_LOAD = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S9>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_ot = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S33>/Discrete-Time Integrator1' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevR_g = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S8>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_ek = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S49>/Discrete-Time Integrator1' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator1_PrevR_j = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S60>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_bp = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S41>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_m = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S40>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_nq = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S42>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_by = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S34>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_es = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S50>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_d = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S61>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_j = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S85>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_h2 = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S81>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_dc = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S19>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_l = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S33>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevRe_f = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S49>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_f0 = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S96>/Discrete-Time Integrator' */
  QS_InnerRateLoop_DW.DiscreteTimeIntegrator_PrevR_bv = 2;

  /* ConstCode for Outport: '<Root>/ScoreDisplay' */
  QS_InnerRateLoop_Y.ScoreDisplay = false;

  /* ConstCode for Outport: '<Root>/ScoreOn' */
  QS_InnerRateLoop_Y.ScoreOn = false;
}

/* Model terminate function */
void QS_InnerRateLoop::terminate()
{
  /* (no terminate code required) */
}

/* Constructor */
QS_InnerRateLoop::QS_InnerRateLoop() :
  QS_InnerRateLoop_U(),
  QS_InnerRateLoop_Y(),
  QS_InnerRateLoop_B(),
  QS_InnerRateLoop_DW(),
  QS_InnerRateLoop_M()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
QS_InnerRateLoop::~QS_InnerRateLoop()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_QS_InnerRateLoop_T * QS_InnerRateLoop::getRTM()
{
  return (&QS_InnerRateLoop_M);
}
