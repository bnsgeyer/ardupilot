/*
 * QS_InnerRateLoop_data.cpp
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

/* Invariant block signals (default storage) */
const ConstB_QS_InnerRateLoop_T QS_InnerRateLoop_ConstB = {
  1.0F
  ,                                    /* '<S11>/Trigonometric Function3' */
  0.0F
  ,                                    /* '<S11>/Trigonometric Function6' */
  1.0F
  ,                                    /* '<S12>/Trigonometric Function1' */
  1.0F
  ,                                    /* '<S12>/Trigonometric Function2' */
  0.0F
  ,                                    /* '<S12>/Trigonometric Function4' */
  0.0F
  ,                                    /* '<S12>/Trigonometric Function5' */
  1.0F
  ,                                    /* '<S72>/Trigonometric Function1' */
  1.0F
  ,                                    /* '<S72>/Trigonometric Function2' */
  0.0F
  ,                                    /* '<S72>/Trigonometric Function4' */
  0.0F
  ,                                    /* '<S72>/Trigonometric Function5' */
  1.0F
  ,                                    /* '<S73>/Trigonometric Function1' */
  1.0F
  ,                                    /* '<S73>/Trigonometric Function2' */
  0.0F
  ,                                    /* '<S73>/Trigonometric Function4' */
  0.0F
  ,                                    /* '<S73>/Trigonometric Function5' */
  0
  ,                                    /* '<S77>/Logical Operator' */
  0
  /* '<S78>/Logical Operator' */
};

/* Constant parameters (default storage) */
const ConstP_QS_InnerRateLoop_T QS_InnerRateLoop_ConstP = {
  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S30>/1-D Lookup Table2'
   *   '<S56>/1-D Lookup Table2'
   */
  { 1.395F, 1.395F, 1.395F, 1.395F, 1.395F },

  /* Pooled Parameter (Expression: bpV)
   * Referenced by:
   *   '<S24>/1-D Lookup Table'
   *   '<S24>/1-D Lookup Table1'
   *   '<S25>/1-D Lookup Table'
   *   '<S25>/1-D Lookup Table1'
   *   '<S26>/1-D Lookup Table'
   *   '<S26>/1-D Lookup Table1'
   *   '<S27>/1-D Lookup Table'
   *   '<S27>/1-D Lookup Table1'
   *   '<S64>/1-D Lookup Table2'
   *   '<S65>/1-D Lookup Table2'
   *   '<S28>/1-D Lookup Table2'
   *   '<S29>/1-D Lookup Table2'
   *   '<S30>/1-D Lookup Table2'
   *   '<S38>/1-D Lookup Table2'
   *   '<S39>/1-D Lookup Table2'
   *   '<S44>/1-D Lookup Table2'
   *   '<S45>/1-D Lookup Table2'
   *   '<S46>/1-D Lookup Table2'
   *   '<S55>/1-D Lookup Table2'
   *   '<S56>/1-D Lookup Table2'
   *   '<S57>/1-D Lookup Table2'
   *   '<S74>/1-D Lookup Table'
   *   '<S74>/1-D Lookup Table1'
   *   '<S75>/1-D Lookup Table'
   *   '<S75>/1-D Lookup Table1'
   *   '<S36>/1-D Lookup Table2'
   *   '<S37>/1-D Lookup Table2'
   *   '<S43>/1-D Lookup Table2'
   *   '<S52>/1-D Lookup Table2'
   *   '<S53>/1-D Lookup Table2'
   *   '<S79>/1-D Lookup Table2'
   *   '<S80>/1-D Lookup Table2'
   *   '<S83>/1-D Lookup Table2'
   *   '<S84>/1-D Lookup Table2'
   *   '<S87>/1-D Lookup Table2'
   *   '<S88>/1-D Lookup Table2'
   *   '<S90>/1-D Lookup Table2'
   *   '<S91>/1-D Lookup Table2'
   *   '<S93>/1-D Lookup Table2'
   *   '<S94>/1-D Lookup Table2'
   */
  { 0.0F, 1.75F, 3.5F, 5.25F, 7.0F },

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S39>/1-D Lookup Table2'
   *   '<S80>/1-D Lookup Table2'
   */
  { 0.93F, 0.93F, 0.93F, 0.93F, 0.93F },

  /* Computed Parameter: uDLookupTable2_tableData
   * Referenced by: '<S46>/1-D Lookup Table2'
   */
  { 1.3671F, 1.3671F, 1.3671F, 1.3671F, 1.3671F },

  /* Computed Parameter: uDLookupTable2_tableData_f
   * Referenced by: '<S84>/1-D Lookup Table2'
   */
  { 0.9114F, 0.9114F, 0.9114F, 0.9114F, 0.9114F },

  /* Computed Parameter: uDLookupTable2_tableData_e
   * Referenced by: '<S87>/1-D Lookup Table2'
   */
  { 0.372F, 0.372F, 0.372F, 0.372F, 0.372F },

  /* Pooled Parameter (Expression: KlatlonI)
   * Referenced by:
   *   '<S91>/1-D Lookup Table2'
   *   '<S94>/1-D Lookup Table2'
   */
  { 0.186F, 0.186F, 0.186F, 0.186F, 0.186F },

  /* Pooled Parameter (Expression: [ -8 -6 -4  -2 -1 0 1 2 4 6 8])
   * Referenced by:
   *   '<Root>/1-D Lookup Table1'
   *   '<Root>/1-D Lookup Table2'
   */
  { -8.0F, -6.0F, -4.0F, -2.0F, -1.0F, 0.0F, 1.0F, 2.0F, 4.0F, 6.0F, 8.0F },

  /* Computed Parameter: uDLookupTable2_tableData_g
   * Referenced by: '<S65>/1-D Lookup Table2'
   */
  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 4.1F, 4.1F,
    2.505F, 2.505F, 2.505F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, -4.1F, -4.1F, -3.18F, -3.18F, -3.18F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -0.33784F, -0.33784F, -0.53738F, -0.53738F,
    -0.53738F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -0.305F, -0.305F,
    -0.305F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, -0.895F, -0.895F, -0.895F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 5.0F, 5.0F, 5.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, -0.357F, -0.357F, -0.357F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, -0.5F, -0.5F, -0.5F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 1.03F, 1.03F, 1.03F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F },

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S65>/Constant'
   *   '<S65>/1-D Lookup Table2'
   */
  { 1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F, 9.0F, 10.0F, 11.0F, 12.0F,
    13.0F, 14.0F, 15.0F, 16.0F, 17.0F, 18.0F, 19.0F, 20.0F, 21.0F, 22.0F, 23.0F,
    24.0F, 25.0F, 26.0F, 27.0F, 28.0F, 29.0F, 30.0F, 31.0F, 32.0F, 33.0F, 34.0F,
    35.0F, 36.0F },

  /* Computed Parameter: uDLookupTable2_tableData_b
   * Referenced by: '<S64>/1-D Lookup Table2'
   */
  { -0.025305314F, -0.025305314F, -0.0254905038F, -0.0254905038F, -0.0254905038F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -0.000900337647F, -0.000900337647F,
    -0.000900337647F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0068965517F, 0.0068965517F, 0.00709219836F, 0.00709219836F, 0.00709219836F,
    0.0F, 0.0F, -0.0F, -0.0F, -0.0F, 0.0F, 0.0F, -0.0F, -0.0F, -0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.00593134388F,
    0.00593134388F, 0.00641025649F, 0.00641025649F, 0.00641025649F, 0.0F, 0.0F,
    -0.0F, -0.0F, -0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0871084109F, 0.0871084109F,
    0.0887369215F, 0.0887369215F, 0.0887369215F },

  /* Pooled Parameter (Expression: [1:16])
   * Referenced by:
   *   '<S64>/Constant'
   *   '<S64>/1-D Lookup Table2'
   */
  { 1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F, 9.0F, 10.0F, 11.0F, 12.0F,
    13.0F, 14.0F, 15.0F, 16.0F },

  /* Pooled Parameter (Expression: Klatlon)
   * Referenced by:
   *   '<S90>/1-D Lookup Table2'
   *   '<S93>/1-D Lookup Table2'
   */
  { 1.09302294F, 1.09302294F, 1.09302294F, 1.09302294F, 1.09302294F },

  /* Computed Parameter: uDLookupTable2_tableData_k
   * Referenced by: '<S88>/1-D Lookup Table2'
   */
  { 1.30631721F, 1.30631721F, 1.30631721F, 1.30631721F, 1.30631721F },

  /* Computed Parameter: uDLookupTable2_tableData_j
   * Referenced by: '<S43>/1-D Lookup Table2'
   */
  { 0.25F, 0.25F, 0.25F, 0.25F, 0.25F },

  /* Computed Parameter: uDLookupTable2_tableData_o
   * Referenced by: '<S38>/1-D Lookup Table2'
   */
  { 15.5315018F, 15.5315018F, 15.5315018F, 15.5315018F, 15.5315018F },

  /* Computed Parameter: uDLookupTable_tableData
   * Referenced by: '<S25>/1-D Lookup Table'
   */
  { 5.77168226F, 5.77168226F, 5.77168226F, 5.77168226F, 5.77168226F },

  /* Computed Parameter: uDLookupTable1_tableData
   * Referenced by: '<S25>/1-D Lookup Table1'
   */
  { 14.9852324F, 14.9852324F, 14.9852324F, 14.9852324F, 14.9852324F },

  /* Computed Parameter: uDLookupTable2_tableData_a
   * Referenced by: '<S29>/1-D Lookup Table2'
   */
  { 76.7134171F, 76.7134171F, 76.7134171F, 76.7134171F, 76.7134171F },

  /* Computed Parameter: uDLookupTable2_tableData_fu
   * Referenced by: '<S28>/1-D Lookup Table2'
   */
  { 10.1607656F, 10.1607656F, 10.1607656F, 10.1607656F, 10.1607656F },

  /* Computed Parameter: uDLookupTable_tableData_l
   * Referenced by: '<S24>/1-D Lookup Table'
   */
  { 17.6654816F, 17.6654816F, 17.6654816F, 17.6654816F, 17.6654816F },

  /* Computed Parameter: uDLookupTable1_tableData_n
   * Referenced by: '<S24>/1-D Lookup Table1'
   */
  { 11.015975F, 11.015975F, 11.015975F, 11.015975F, 11.015975F },

  /* Computed Parameter: uDLookupTable2_tableData_ex
   * Referenced by: '<S45>/1-D Lookup Table2'
   */
  { 74.339119F, 74.339119F, 74.339119F, 74.339119F, 74.339119F },

  /* Computed Parameter: uDLookupTable2_tableData_c
   * Referenced by: '<S44>/1-D Lookup Table2'
   */
  { 9.69888592F, 9.69888592F, 9.69888592F, 9.69888592F, 9.69888592F },

  /* Computed Parameter: uDLookupTable_tableData_m
   * Referenced by: '<S26>/1-D Lookup Table'
   */
  { 17.6197205F, 17.6197205F, 17.6197205F, 17.6197205F, 17.6197205F },

  /* Computed Parameter: uDLookupTable1_tableData_d
   * Referenced by: '<S26>/1-D Lookup Table1'
   */
  { 10.6072197F, 10.6072197F, 10.6072197F, 10.6072197F, 10.6072197F },

  /* Computed Parameter: uDLookupTable2_tableData_au
   * Referenced by: '<S55>/1-D Lookup Table2'
   */
  { 62.2932281F, 62.2932281F, 62.2932281F, 62.2932281F, 62.2932281F },

  /* Computed Parameter: uDLookupTable2_tableData_c0
   * Referenced by: '<S57>/1-D Lookup Table2'
   */
  { 11.2615271F, 11.2615271F, 11.2615271F, 11.2615271F, 11.2615271F },

  /* Computed Parameter: uDLookupTable_tableData_h
   * Referenced by: '<S27>/1-D Lookup Table'
   */
  { 16.4246674F, 16.4246674F, 16.4246674F, 16.4246674F, 16.4246674F },

  /* Computed Parameter: uDLookupTable1_tableData_f
   * Referenced by: '<S27>/1-D Lookup Table1'
   */
  { 11.8481846F, 11.8481846F, 11.8481846F, 11.8481846F, 11.8481846F },

  /* Computed Parameter: uDLookupTable2_tableData_kg
   * Referenced by: '<S83>/1-D Lookup Table2'
   */
  { 0.215953469F, 0.215953469F, 0.215953469F, 0.215953469F, 0.215953469F },

  /* Computed Parameter: uDLookupTable_tableData_d
   * Referenced by: '<S75>/1-D Lookup Table'
   */
  { 19.2063885F, 19.2063885F, 19.2063885F, 19.2063885F, 19.2063885F },

  /* Computed Parameter: uDLookupTable1_tableData_m
   * Referenced by: '<S75>/1-D Lookup Table1'
   */
  { 1.08121574F, 1.08121574F, 1.08121574F, 1.08121574F, 1.08121574F },

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S36>/1-D Lookup Table2'
   *   '<S52>/1-D Lookup Table2'
   */
  { 2.0F, 2.0F, 2.0F, 2.0F, 2.0F },

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S37>/1-D Lookup Table2'
   *   '<S53>/1-D Lookup Table2'
   */
  { 8.0F, 8.0F, 8.0F, 8.0F, 8.0F },

  /* Computed Parameter: uDLookupTable2_tableData_n
   * Referenced by: '<S79>/1-D Lookup Table2'
   */
  { 0.216432646F, 0.216432646F, 0.216432646F, 0.216432646F, 0.216432646F },

  /* Computed Parameter: uDLookupTable_tableData_hs
   * Referenced by: '<S74>/1-D Lookup Table'
   */
  { 19.9177628F, 19.9177628F, 19.9177628F, 19.9177628F, 19.9177628F },

  /* Computed Parameter: uDLookupTable1_tableData_p
   * Referenced by: '<S74>/1-D Lookup Table1'
   */
  { 1.08558869F, 1.08558869F, 1.08558869F, 1.08558869F, 1.08558869F }
};
