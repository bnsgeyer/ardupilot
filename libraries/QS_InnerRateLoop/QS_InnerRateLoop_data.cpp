/*
 * QS_InnerRateLoop_data.cpp
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

/* Invariant block signals (default storage) */
const ConstB_QS_InnerRateLoop_T QS_InnerRateLoop_ConstB = {
  1.0F
  ,                                    /* '<S9>/Trigonometric Function2' */
  0.0F
  ,                                    /* '<S9>/Trigonometric Function5' */
  0.0F
  ,                                    /* '<S9>/Trigonometric Function4' */
  1.0F
  ,                                    /* '<S9>/Trigonometric Function1' */
  1.0F
  ,                                    /* '<S8>/Trigonometric Function3' */
  0.0F
  ,                                    /* '<S8>/Trigonometric Function6' */
  1.0F
  ,                                    /* '<S72>/Trigonometric Function1' */
  0.0F
  ,                                    /* '<S72>/Trigonometric Function5' */
  0.0F
  ,                                    /* '<S72>/Trigonometric Function4' */
  1.0F
  ,                                    /* '<S72>/Trigonometric Function2' */
  1.0F
  ,                                    /* '<S103>/Trigonometric Function1' */
  0.0F
  ,                                    /* '<S103>/Trigonometric Function5' */
  0.0F
  ,                                    /* '<S103>/Trigonometric Function4' */
  1.0F
  ,                                    /* '<S103>/Trigonometric Function2' */
  0.0F
  ,                                    /* '<S71>/Trigonometric Function4' */
  0.0F
  ,                                    /* '<S71>/Trigonometric Function5' */
  1.0F
  ,                                    /* '<S71>/Trigonometric Function1' */
  1.0F
  ,                                    /* '<S71>/Trigonometric Function2' */
  49.6F
  ,                                    /* '<S12>/Sum2' */
  29.6F
  /* '<S12>/Sum1' */
};

/* Constant parameters (default storage) */
const ConstP_QS_InnerRateLoop_T QS_InnerRateLoop_ConstP = {
  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S29>/1-D Lookup Table2'
   *   '<S55>/1-D Lookup Table2'
   */
  { 1.70887494F, 1.70887494F, 1.70887494F, 1.70887494F, 1.70887494F },

  /* Pooled Parameter (Expression: bpV)
   * Referenced by:
   *   '<S23>/1-D Lookup Table'
   *   '<S23>/1-D Lookup Table1'
   *   '<S24>/1-D Lookup Table'
   *   '<S24>/1-D Lookup Table1'
   *   '<S25>/1-D Lookup Table'
   *   '<S25>/1-D Lookup Table1'
   *   '<S26>/1-D Lookup Table'
   *   '<S26>/1-D Lookup Table1'
   *   '<S63>/1-D Lookup Table2'
   *   '<S64>/1-D Lookup Table2'
   *   '<S27>/1-D Lookup Table2'
   *   '<S28>/1-D Lookup Table2'
   *   '<S29>/1-D Lookup Table2'
   *   '<S37>/1-D Lookup Table2'
   *   '<S38>/1-D Lookup Table2'
   *   '<S43>/1-D Lookup Table2'
   *   '<S44>/1-D Lookup Table2'
   *   '<S45>/1-D Lookup Table2'
   *   '<S54>/1-D Lookup Table2'
   *   '<S55>/1-D Lookup Table2'
   *   '<S56>/1-D Lookup Table2'
   *   '<S73>/1-D Lookup Table'
   *   '<S73>/1-D Lookup Table1'
   *   '<S74>/1-D Lookup Table'
   *   '<S74>/1-D Lookup Table1'
   *   '<S35>/1-D Lookup Table2'
   *   '<S36>/1-D Lookup Table2'
   *   '<S42>/1-D Lookup Table2'
   *   '<S51>/1-D Lookup Table2'
   *   '<S52>/1-D Lookup Table2'
   *   '<S78>/1-D Lookup Table2'
   *   '<S79>/1-D Lookup Table2'
   *   '<S82>/1-D Lookup Table2'
   *   '<S83>/1-D Lookup Table2'
   *   '<S86>/1-D Lookup Table2'
   *   '<S87>/1-D Lookup Table2'
   *   '<S89>/1-D Lookup Table2'
   *   '<S90>/1-D Lookup Table2'
   *   '<S92>/1-D Lookup Table2'
   *   '<S93>/1-D Lookup Table2'
   */
  { 0.0F, 1.75F, 3.5F, 5.25F, 7.0F },

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S38>/1-D Lookup Table2'
   *   '<S79>/1-D Lookup Table2'
   */
  { 1.13925F, 1.13925F, 1.13925F, 1.13925F, 1.13925F },

  /* Computed Parameter: uDLookupTable2_tableData
   * Referenced by: '<S45>/1-D Lookup Table2'
   */
  { 1.67469752F, 1.67469752F, 1.67469752F, 1.67469752F, 1.67469752F },

  /* Computed Parameter: uDLookupTable2_tableData_f
   * Referenced by: '<S83>/1-D Lookup Table2'
   */
  { 1.11646497F, 1.11646497F, 1.11646497F, 1.11646497F, 1.11646497F },

  /* Computed Parameter: uDLookupTable2_tableData_e
   * Referenced by: '<S86>/1-D Lookup Table2'
   */
  { 0.4557F, 0.4557F, 0.4557F, 0.4557F, 0.4557F },

  /* Pooled Parameter (Expression: KlatlonI)
   * Referenced by:
   *   '<S90>/1-D Lookup Table2'
   *   '<S93>/1-D Lookup Table2'
   */
  { 0.22785F, 0.22785F, 0.22785F, 0.22785F, 0.22785F },

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S64>/Constant'
   *   '<S64>/1-D Lookup Table2'
   */
  { 1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F, 9.0F, 10.0F, 11.0F, 12.0F,
    13.0F, 14.0F, 15.0F, 16.0F, 17.0F, 18.0F, 19.0F, 20.0F, 21.0F, 22.0F, 23.0F,
    24.0F, 25.0F, 26.0F, 27.0F, 28.0F, 29.0F, 30.0F, 31.0F, 32.0F, 33.0F, 34.0F,
    35.0F, 36.0F },

  /* Computed Parameter: uDLookupTable2_tableData_g
   * Referenced by: '<S64>/1-D Lookup Table2'
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

  /* Pooled Parameter (Expression: [1:16])
   * Referenced by:
   *   '<S63>/Constant'
   *   '<S63>/1-D Lookup Table2'
   */
  { 1.0F, 2.0F, 3.0F, 4.0F, 5.0F, 6.0F, 7.0F, 8.0F, 9.0F, 10.0F, 11.0F, 12.0F,
    13.0F, 14.0F, 15.0F, 16.0F },

  /* Computed Parameter: uDLookupTable2_tableData_b
   * Referenced by: '<S63>/1-D Lookup Table2'
   */
  { -0.025305314F, -0.025305314F, -0.0254905038F, -0.0254905038F, -0.0254905038F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -0.000900337647F, -0.000900337647F,
    -0.000900337647F, -0.0F, -0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0068965517F, 0.0068965517F, 0.00709219836F, 0.00709219836F,
    0.00709219836F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, -0.0F, -0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.00593134388F,
    0.00593134388F, 0.00641025649F, 0.00641025649F, 0.00641025649F, -0.0F, -0.0F,
    0.0F, 0.0F, 0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F,
    -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, -0.0F, 0.0871084109F,
    0.0871084109F, 0.0887369215F, 0.0887369215F, 0.0887369215F },

  /* Pooled Parameter (Expression: Klatlon)
   * Referenced by:
   *   '<S89>/1-D Lookup Table2'
   *   '<S92>/1-D Lookup Table2'
   */
  { 1.00830865F, 1.00830865F, 1.00830865F, 1.00830865F, 1.00830865F },

  /* Computed Parameter: uDLookupTable2_tableData_k
   * Referenced by: '<S87>/1-D Lookup Table2'
   */
  { 1.0F, 1.0F, 1.0F, 1.0F, 1.0F },

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S12>/1-D Lookup Table1'
   *   '<S12>/1-D Lookup Table2'
   *   '<S12>/1-D Lookup Table3'
   */
  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

  /* Expression: VXdata'
   * Referenced by: '<S12>/1-D Lookup Table'
   */
  { 0.0F, 0.24686F, 0.49371F, 0.74057F, 0.98743F, 1.2343F, 1.4811F, 1.728F,
    1.9749F, 2.2217F, 2.4686F, 2.7154F, 2.592F, 2.3451F, 2.0983F, 1.8514F,
    1.6046F, 1.3577F, 1.1109F, 0.864F, 0.61714F, 0.37029F, 0.12343F, 0.0F },

  /* Computed Parameter: uDLookupTable2_tableData_j
   * Referenced by: '<S42>/1-D Lookup Table2'
   */
  { 0.05F, 0.05F, 0.05F, 0.05F, 0.05F },

  /* Computed Parameter: uDLookupTable2_tableData_o
   * Referenced by: '<S37>/1-D Lookup Table2'
   */
  { 14.4735041F, 14.4735041F, 14.4735041F, 14.4735041F, 14.4735041F },

  /* Computed Parameter: uDLookupTable_tableData_k
   * Referenced by: '<S24>/1-D Lookup Table'
   */
  { 9.79965687F, 9.79965687F, 9.79965687F, 9.79965687F, 9.79965687F },

  /* Computed Parameter: uDLookupTable1_tableData
   * Referenced by: '<S24>/1-D Lookup Table1'
   */
  { 13.2442446F, 13.2442446F, 13.2442446F, 13.2442446F, 13.2442446F },

  /* Computed Parameter: uDLookupTable2_tableData_a
   * Referenced by: '<S28>/1-D Lookup Table2'
   */
  { 113.135445F, 113.135445F, 113.135445F, 113.135445F, 113.135445F },

  /* Computed Parameter: uDLookupTable2_tableData_fu
   * Referenced by: '<S27>/1-D Lookup Table2'
   */
  { 10.5738382F, 10.5738382F, 10.5738382F, 10.5738382F, 10.5738382F },

  /* Computed Parameter: uDLookupTable_tableData_l
   * Referenced by: '<S23>/1-D Lookup Table'
   */
  { 26.5284462F, 26.5284462F, 26.5284462F, 26.5284462F, 26.5284462F },

  /* Computed Parameter: uDLookupTable1_tableData_n
   * Referenced by: '<S23>/1-D Lookup Table1'
   */
  { 11.008009F, 11.008009F, 11.008009F, 11.008009F, 11.008009F },

  /* Computed Parameter: uDLookupTable2_tableData_ex
   * Referenced by: '<S44>/1-D Lookup Table2'
   */
  { 136.347137F, 136.347137F, 136.347137F, 136.347137F, 136.347137F },

  /* Computed Parameter: uDLookupTable2_tableData_c
   * Referenced by: '<S43>/1-D Lookup Table2'
   */
  { 10.6858244F, 10.6858244F, 10.6858244F, 10.6858244F, 10.6858244F },

  /* Computed Parameter: uDLookupTable_tableData_m
   * Referenced by: '<S25>/1-D Lookup Table'
   */
  { 27.4979649F, 27.4979649F, 27.4979649F, 27.4979649F, 27.4979649F },

  /* Computed Parameter: uDLookupTable1_tableData_d
   * Referenced by: '<S25>/1-D Lookup Table1'
   */
  { 10.1993437F, 10.1993437F, 10.1993437F, 10.1993437F, 10.1993437F },

  /* Computed Parameter: uDLookupTable2_tableData_au
   * Referenced by: '<S54>/1-D Lookup Table2'
   */
  { 92.1007614F, 92.1007614F, 92.1007614F, 92.1007614F, 92.1007614F },

  /* Computed Parameter: uDLookupTable2_tableData_c0
   * Referenced by: '<S56>/1-D Lookup Table2'
   */
  { 12.9308949F, 12.9308949F, 12.9308949F, 12.9308949F, 12.9308949F },

  /* Computed Parameter: uDLookupTable_tableData_h
   * Referenced by: '<S26>/1-D Lookup Table'
   */
  { 22.6359711F, 22.6359711F, 22.6359711F, 22.6359711F, 22.6359711F },

  /* Computed Parameter: uDLookupTable1_tableData_f
   * Referenced by: '<S26>/1-D Lookup Table1'
   */
  { 12.9009438F, 12.9009438F, 12.9009438F, 12.9009438F, 12.9009438F },

  /* Computed Parameter: uDLookupTable2_tableData_kg
   * Referenced by: '<S82>/1-D Lookup Table2'
   */
  { 0.357682616F, 0.357682616F, 0.357682616F, 0.357682616F, 0.357682616F },

  /* Computed Parameter: uDLookupTable_tableData_d
   * Referenced by: '<S74>/1-D Lookup Table'
   */
  { 13.388237F, 13.388237F, 13.388237F, 13.388237F, 13.388237F },

  /* Computed Parameter: uDLookupTable1_tableData_m
   * Referenced by: '<S74>/1-D Lookup Table1'
   */
  { 2.3275919F, 2.3275919F, 2.3275919F, 2.3275919F, 2.3275919F },

  /* Computed Parameter: uDLookupTable2_tableData_or
   * Referenced by: '<S51>/1-D Lookup Table2'
   */
  { 3.73125648F, 3.73125648F, 3.73125648F, 3.73125648F, 3.73125648F },

  /* Computed Parameter: uDLookupTable2_tableData_gp
   * Referenced by: '<S52>/1-D Lookup Table2'
   */
  { 14.9250259F, 14.9250259F, 14.9250259F, 14.9250259F, 14.9250259F },

  /* Computed Parameter: uDLookupTable2_tableData_n
   * Referenced by: '<S78>/1-D Lookup Table2'
   */
  { 0.234992728F, 0.234992728F, 0.234992728F, 0.234992728F, 0.234992728F },

  /* Computed Parameter: uDLookupTable_tableData_hs
   * Referenced by: '<S73>/1-D Lookup Table'
   */
  { 24.229393F, 24.229393F, 24.229393F, 24.229393F, 24.229393F },

  /* Computed Parameter: uDLookupTable1_tableData_p
   * Referenced by: '<S73>/1-D Lookup Table1'
   */
  { 1.3391695F, 1.3391695F, 1.3391695F, 1.3391695F, 1.3391695F },

  /* Computed Parameter: uDLookupTable2_tableData_ob
   * Referenced by: '<S35>/1-D Lookup Table2'
   */
  { 2.25760984F, 2.25760984F, 2.25760984F, 2.25760984F, 2.25760984F },

  /* Computed Parameter: uDLookupTable2_tableData_l
   * Referenced by: '<S36>/1-D Lookup Table2'
   */
  { 9.03043938F, 9.03043938F, 9.03043938F, 9.03043938F, 9.03043938F }
};
