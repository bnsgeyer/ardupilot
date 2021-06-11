/*
 * QS_InnerRateLoop.h
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

#ifndef RTW_HEADER_QS_InnerRateLoop_h_
#define RTW_HEADER_QS_InnerRateLoop_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef QS_InnerRateLoop_COMMON_INCLUDES_
# define QS_InnerRateLoop_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* QS_InnerRateLoop_COMMON_INCLUDES_ */

#include "QS_InnerRateLoop_types.h"

/* Shared type includes */
#include "multiword_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
typedef struct {
  real32_T col;                        /* '<S104>/col' */
  real32_T lon;                        /* '<S98>/lon' */
  real32_T lat;                        /* '<S98>/lat' */
  real32_T col_h;                      /* '<S98>/col' */
  real32_T ped;                        /* '<S98>/ped' */
  real32_T In1[4];                     /* '<S6>/In1' */
  real32_T Gain;                       /* '<S4>/Gain' */
} B_QS_InnerRateLoop_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T Delay2_DSTATE;                /* '<S96>/Delay2' */
  real_T DiscreteTimeIntegrator_DSTATE;/* '<S59>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_a;/* '<S94>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_m;/* '<S69>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_h;/* '<S91>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTATE;/* '<S69>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator_DSTATE_c;/* '<S88>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator2_DSTATE;/* '<S69>/Discrete-Time Integrator2' */
  real32_T UnitDelay_DSTATE;           /* '<S12>/Unit Delay' */
  real32_T UnitDelay1_DSTATE;          /* '<S108>/Unit Delay1' */
  real32_T UnitDelay2_DSTATE;          /* '<S108>/Unit Delay2' */
  real32_T UnitDelay_DSTATE_a;         /* '<S39>/Unit Delay' */
  real32_T DiscreteTimeIntegrator_DSTAT_hq;/* '<S31>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_b;/* '<S30>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTAT_o;/* '<S32>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator_DSTAT_mk;/* '<S47>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_o;/* '<S46>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTAT_m;/* '<S48>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator_DSTATE_d;/* '<S57>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_mr;/* '<S58>/Discrete-Time Integrator' */
  real32_T Delay_DSTATE;               /* '<S13>/Delay' */
  real32_T DiscreteTimeIntegrator_DSTAT_mz;/* '<S13>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_f;/* '<S40>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_fs;/* '<S39>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_c4;/* '<S41>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_o2;/* '<S33>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_bw;/* '<S34>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_j;/* '<S49>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_di;/* '<S50>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_l;/* '<S60>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_hv;/* '<S61>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_co;/* '<S65>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_i;/* '<S66>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_ch;/* '<S67>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_a5;/* '<S68>/Discrete-Time Integrator' */
  real32_T UnitDelay_DSTATE_f;         /* '<S65>/Unit Delay' */
  real32_T UnitDelay_DSTATE_av;        /* '<S66>/Unit Delay' */
  real32_T UnitDelay_DSTATE_d;         /* '<S67>/Unit Delay' */
  real32_T UnitDelay_DSTATE_e;         /* '<S68>/Unit Delay' */
  real32_T UnitDelay_DSTATE_o;         /* '<S61>/Unit Delay' */
  real32_T UnitDelay_DSTATE_eo;        /* '<S50>/Unit Delay' */
  real32_T UnitDelay_DSTATE_b;         /* '<S34>/Unit Delay' */
  real32_T UnitDelay_DSTATE_h;         /* '<S41>/Unit Delay' */
  real32_T DiscreteTimeIntegrator_DSTATE_n;/* '<S84>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_g;/* '<S85>/Discrete-Time Integrator' */
  real32_T UnitDelay2_DSTATE_k;        /* '<S106>/Unit Delay2' */
  real32_T UnitDelay1_DSTATE_f;        /* '<S106>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_c;        /* '<S48>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_j;         /* '<S48>/Unit Delay' */
  real32_T DiscreteTimeIntegrator_DSTAT_cf;/* '<S80>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_cc;/* '<S81>/Discrete-Time Integrator' */
  real32_T UnitDelay2_DSTATE_a;        /* '<S107>/Unit Delay2' */
  real32_T UnitDelay1_DSTATE_fj;       /* '<S107>/Unit Delay1' */
  real32_T UnitDelay1_DSTATE_a;        /* '<S32>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_g;         /* '<S32>/Unit Delay' */
  real32_T UnitDelay_DSTATE_gi;        /* '<S81>/Unit Delay' */
  real32_T UnitDelay_DSTATE_oc;        /* '<S85>/Unit Delay' */
  real32_T UnitDelay_DSTATE_j2;        /* '<S19>/Unit Delay' */
  real32_T UnitDelay_DSTATE_da;        /* '<S20>/Unit Delay' */
  real32_T UnitDelay_DSTATE_ag;        /* '<S21>/Unit Delay' */
  real32_T DiscreteTimeIntegrator_DSTATE_e;/* '<S21>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_cg;/* '<S22>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_o3;/* '<S20>/Discrete-Time Integrator' */
  real32_T UnitDelay1_DSTATE_a1;       /* '<S18>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_ak;        /* '<S18>/Unit Delay' */
  real32_T DiscreteTimeIntegrator1_DSTAT_g;/* '<S18>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator_DSTAT_nv;/* '<S19>/Discrete-Time Integrator' */
  real32_T UnitDelay_DSTATE_o1;        /* '<S22>/Unit Delay' */
  real32_T DiscreteTimeIntegrator_DSTAT_gh;/* '<S18>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_dn;/* '<S32>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_nq;/* '<S48>/Discrete-Time Integrator' */
  real32_T UnitDelay_DSTATE_k;         /* '<S105>/Unit Delay' */
  real32_T DiscreteTimeIntegrator_DSTAT_gy;/* '<S105>/Discrete-Time Integrator' */
  real32_T PrevY;                      /* '<S12>/Rate Limiter' */
  boolean_T Delay1_DSTATE;             /* '<S96>/Delay1' */
  boolean_T UnitDelay_DSTATE_ax;       /* '<S108>/Unit Delay' */
  boolean_T Delay1_DSTATE_j[2];        /* '<S13>/Delay1' */
  boolean_T UnitDelay_DSTATE_e2;       /* '<S106>/Unit Delay' */
  boolean_T UnitDelay_DSTATE_j1;       /* '<S107>/Unit Delay' */
  int8_T DiscreteTimeIntegrator_PrevRese;/* '<S94>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_a;/* '<S69>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_o;/* '<S91>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_PrevRes;/* '<S69>/Discrete-Time Integrator1' */
  int8_T DiscreteTimeIntegrator_PrevRe_b;/* '<S88>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator2_PrevRes;/* '<S69>/Discrete-Time Integrator2' */
  int8_T DiscreteTimeIntegrator1_PrevR_g;/* '<S32>/Discrete-Time Integrator1' */
  int8_T DiscreteTimeIntegrator1_PrevR_j;/* '<S48>/Discrete-Time Integrator1' */
  int8_T DiscreteTimeIntegrator_PrevRe_p;/* '<S13>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevR_bp;/* '<S59>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_m;/* '<S40>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_n;/* '<S39>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevR_by;/* '<S41>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_e;/* '<S33>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_d;/* '<S49>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_j;/* '<S60>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_h;/* '<S84>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevR_dc;/* '<S80>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_l;/* '<S21>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevR_em;/* '<S22>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevR_hn;/* '<S20>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_Prev_gs;/* '<S18>/Discrete-Time Integrator1' */
  int8_T DiscreteTimeIntegrator_PrevR_pr;/* '<S19>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevR_lm;/* '<S18>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_f;/* '<S32>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevR_f0;/* '<S48>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevR_bv;/* '<S105>/Discrete-Time Integrator' */
  uint8_T DiscreteTimeIntegrator_IC_LOADI;/* '<S69>/Discrete-Time Integrator' */
  uint8_T DiscreteTimeIntegrator1_IC_LOAD;/* '<S69>/Discrete-Time Integrator1' */
  uint8_T DiscreteTimeIntegrator2_IC_LOAD;/* '<S69>/Discrete-Time Integrator2' */
  uint8_T icLoad;                      /* '<S13>/Delay' */
  uint8_T DiscreteTimeIntegrator_IC_LOA_b;/* '<S13>/Discrete-Time Integrator' */
  uint8_T DiscreteTimeIntegrator_IC_LOA_j;/* '<S21>/Discrete-Time Integrator' */
  uint8_T DiscreteTimeIntegrator_IC_LOA_m;/* '<S20>/Discrete-Time Integrator' */
  uint8_T DiscreteTimeIntegrator1_IC_LO_e;/* '<S18>/Discrete-Time Integrator1' */
} DW_QS_InnerRateLoop_T;

/* Invariant block signals (default storage) */
typedef const struct tag_ConstB_QS_InnerRateLoop_T {
  real32_T TrigonometricFunction2;     /* '<S9>/Trigonometric Function2' */
  real32_T TrigonometricFunction5;     /* '<S9>/Trigonometric Function5' */
  real32_T TrigonometricFunction4;     /* '<S9>/Trigonometric Function4' */
  real32_T TrigonometricFunction1;     /* '<S9>/Trigonometric Function1' */
  real32_T TrigonometricFunction3;     /* '<S8>/Trigonometric Function3' */
  real32_T TrigonometricFunction6;     /* '<S8>/Trigonometric Function6' */
  real32_T TrigonometricFunction1_g;   /* '<S72>/Trigonometric Function1' */
  real32_T TrigonometricFunction5_j;   /* '<S72>/Trigonometric Function5' */
  real32_T TrigonometricFunction4_k;   /* '<S72>/Trigonometric Function4' */
  real32_T TrigonometricFunction2_a;   /* '<S72>/Trigonometric Function2' */
  real32_T TrigonometricFunction1_k;   /* '<S103>/Trigonometric Function1' */
  real32_T TrigonometricFunction5_o;   /* '<S103>/Trigonometric Function5' */
  real32_T TrigonometricFunction4_n;   /* '<S103>/Trigonometric Function4' */
  real32_T TrigonometricFunction2_l;   /* '<S103>/Trigonometric Function2' */
  real32_T TrigonometricFunction4_km;  /* '<S71>/Trigonometric Function4' */
  real32_T TrigonometricFunction5_f;   /* '<S71>/Trigonometric Function5' */
  real32_T TrigonometricFunction1_m;   /* '<S71>/Trigonometric Function1' */
  real32_T TrigonometricFunction2_ag;  /* '<S71>/Trigonometric Function2' */
  real32_T Sum2;                       /* '<S12>/Sum2' */
  real32_T Sum1;                       /* '<S12>/Sum1' */
} ConstB_QS_InnerRateLoop_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S29>/1-D Lookup Table2'
   *   '<S55>/1-D Lookup Table2'
   */
  real32_T pooled6[5];

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
  real32_T pooled7[5];

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S38>/1-D Lookup Table2'
   *   '<S79>/1-D Lookup Table2'
   */
  real32_T pooled8[5];

  /* Computed Parameter: uDLookupTable2_tableData
   * Referenced by: '<S45>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData[5];

  /* Computed Parameter: uDLookupTable2_tableData_f
   * Referenced by: '<S83>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_f[5];

  /* Computed Parameter: uDLookupTable2_tableData_e
   * Referenced by: '<S86>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_e[5];

  /* Pooled Parameter (Expression: KlatlonI)
   * Referenced by:
   *   '<S90>/1-D Lookup Table2'
   *   '<S93>/1-D Lookup Table2'
   */
  real32_T pooled13[5];

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S64>/Constant'
   *   '<S64>/1-D Lookup Table2'
   */
  real32_T pooled17[36];

  /* Computed Parameter: uDLookupTable2_tableData_g
   * Referenced by: '<S64>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_g[180];

  /* Pooled Parameter (Expression: [1:16])
   * Referenced by:
   *   '<S63>/Constant'
   *   '<S63>/1-D Lookup Table2'
   */
  real32_T pooled18[16];

  /* Computed Parameter: uDLookupTable2_tableData_b
   * Referenced by: '<S63>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_b[80];

  /* Pooled Parameter (Expression: Klatlon)
   * Referenced by:
   *   '<S89>/1-D Lookup Table2'
   *   '<S92>/1-D Lookup Table2'
   */
  real32_T pooled20[5];

  /* Computed Parameter: uDLookupTable2_tableData_k
   * Referenced by: '<S87>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_k[5];

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S12>/1-D Lookup Table1'
   *   '<S12>/1-D Lookup Table2'
   *   '<S12>/1-D Lookup Table3'
   */
  real32_T pooled26[24];

  /* Expression: VXdata'
   * Referenced by: '<S12>/1-D Lookup Table'
   */
  real32_T uDLookupTable_tableData[24];

  /* Computed Parameter: uDLookupTable2_tableData_j
   * Referenced by: '<S42>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_j[5];

  /* Computed Parameter: uDLookupTable2_tableData_o
   * Referenced by: '<S37>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_o[5];

  /* Computed Parameter: uDLookupTable_tableData_k
   * Referenced by: '<S24>/1-D Lookup Table'
   */
  real32_T uDLookupTable_tableData_k[5];

  /* Computed Parameter: uDLookupTable1_tableData
   * Referenced by: '<S24>/1-D Lookup Table1'
   */
  real32_T uDLookupTable1_tableData[5];

  /* Computed Parameter: uDLookupTable2_tableData_a
   * Referenced by: '<S28>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_a[5];

  /* Computed Parameter: uDLookupTable2_tableData_fu
   * Referenced by: '<S27>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_fu[5];

  /* Computed Parameter: uDLookupTable_tableData_l
   * Referenced by: '<S23>/1-D Lookup Table'
   */
  real32_T uDLookupTable_tableData_l[5];

  /* Computed Parameter: uDLookupTable1_tableData_n
   * Referenced by: '<S23>/1-D Lookup Table1'
   */
  real32_T uDLookupTable1_tableData_n[5];

  /* Computed Parameter: uDLookupTable2_tableData_ex
   * Referenced by: '<S44>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_ex[5];

  /* Computed Parameter: uDLookupTable2_tableData_c
   * Referenced by: '<S43>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_c[5];

  /* Computed Parameter: uDLookupTable_tableData_m
   * Referenced by: '<S25>/1-D Lookup Table'
   */
  real32_T uDLookupTable_tableData_m[5];

  /* Computed Parameter: uDLookupTable1_tableData_d
   * Referenced by: '<S25>/1-D Lookup Table1'
   */
  real32_T uDLookupTable1_tableData_d[5];

  /* Computed Parameter: uDLookupTable2_tableData_au
   * Referenced by: '<S54>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_au[5];

  /* Computed Parameter: uDLookupTable2_tableData_c0
   * Referenced by: '<S56>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_c0[5];

  /* Computed Parameter: uDLookupTable_tableData_h
   * Referenced by: '<S26>/1-D Lookup Table'
   */
  real32_T uDLookupTable_tableData_h[5];

  /* Computed Parameter: uDLookupTable1_tableData_f
   * Referenced by: '<S26>/1-D Lookup Table1'
   */
  real32_T uDLookupTable1_tableData_f[5];

  /* Computed Parameter: uDLookupTable2_tableData_kg
   * Referenced by: '<S82>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_kg[5];

  /* Computed Parameter: uDLookupTable_tableData_d
   * Referenced by: '<S74>/1-D Lookup Table'
   */
  real32_T uDLookupTable_tableData_d[5];

  /* Computed Parameter: uDLookupTable1_tableData_m
   * Referenced by: '<S74>/1-D Lookup Table1'
   */
  real32_T uDLookupTable1_tableData_m[5];

  /* Computed Parameter: uDLookupTable2_tableData_or
   * Referenced by: '<S51>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_or[5];

  /* Computed Parameter: uDLookupTable2_tableData_gp
   * Referenced by: '<S52>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_gp[5];

  /* Computed Parameter: uDLookupTable2_tableData_n
   * Referenced by: '<S78>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_n[5];

  /* Computed Parameter: uDLookupTable_tableData_hs
   * Referenced by: '<S73>/1-D Lookup Table'
   */
  real32_T uDLookupTable_tableData_hs[5];

  /* Computed Parameter: uDLookupTable1_tableData_p
   * Referenced by: '<S73>/1-D Lookup Table1'
   */
  real32_T uDLookupTable1_tableData_p[5];

  /* Computed Parameter: uDLookupTable2_tableData_ob
   * Referenced by: '<S35>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_ob[5];

  /* Computed Parameter: uDLookupTable2_tableData_l
   * Referenced by: '<S36>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_l[5];
} ConstP_QS_InnerRateLoop_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  boolean_T engage;                    /* '<Root>/engage' */
  real32_T input_lat;                  /* '<Root>/input_lat' */
  real32_T input_lon;                  /* '<Root>/input_lon' */
  real32_T input_col;                  /* '<Root>/input_col' */
  real32_T input_ped;                  /* '<Root>/input_ped' */
  real32_T vN_fpsKF;                   /* '<Root>/vN_fps (KF)' */
  real32_T vE_fpsKF;                   /* '<Root>/vE_fps (KF)' */
  real32_T vD_fpsKF;                   /* '<Root>/vD_fps (KF)' */
  real32_T p_rps;                      /* '<Root>/p_rps' */
  real32_T q_rps;                      /* '<Root>/q_rps' */
  real32_T r_rps;                      /* '<Root>/r_rps' */
  real32_T phi_rad;                    /* '<Root>/phi_rad' */
  real32_T theta_rad;                  /* '<Root>/theta_rad' */
  real32_T psi_rad;                    /* '<Root>/psi_rad' */
  real32_T posNorthKF;                 /* '<Root>/pos North (KF)' */
  real32_T posEastKF;                  /* '<Root>/pos East (KF)' */
  real32_T posDownKF;                  /* '<Root>/pos Down (KF)' */
  real32_T mixer_in_x;                 /* '<Root>/mixer_in_x' */
  real32_T mixer_in_y;                 /* '<Root>/mixer_in_y' */
  real32_T mixer_in_z;                 /* '<Root>/mixer_in_z' */
  real32_T mixer_in_throttle;          /* '<Root>/mixer_in_throttle' */
  real32_T CH8_flag;                   /* '<Root>/CH8_flag' */
  real32_T rc_6;                       /* '<Root>/rc_6' */
  real32_T sweep_multiplier;           /* '<Root>/sweep_multiplier' */
  real32_T vel;                        /* '<Root>/vel' */
  real32_T psi_flight_path;            /* '<Root>/psi_flight_path' */
  real_T gamma;                        /* '<Root>/gamma' */
  real32_T psi_quad;                   /* '<Root>/psi_quad' */
  boolean_T TrajectorySwitch;          /* '<Root>/Trajectory Switch' */
  real32_T Rv;                         /* '<Root>/Rv' */
  real32_T Rp;                         /* '<Root>/Rp' */
  real32_T Baro_Alt_m;                 /* '<Root>/Baro_Alt_m' */
  real32_T Ax_mpss;                    /* '<Root>/Ax_mpss' */
  real32_T Ay_mpss;                    /* '<Root>/Ay_mpss' */
  real32_T Az_mpss;                    /* '<Root>/Az_mpss' */
} ExtU_QS_InnerRateLoop_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real32_T mixer_throttle;             /* '<Root>/mixer_throttle' */
  real32_T mixer_x;                    /* '<Root>/mixer_x' */
  real32_T mixer_y;                    /* '<Root>/mixer_y' */
  real32_T mixer_z;                    /* '<Root>/mixer_z' */
  real32_T roll_sweep;                 /* '<Root>/roll_sweep' */
  real32_T pitch_sweep;                /* '<Root>/pitch_sweep' */
  real32_T yaw_sweep;                  /* '<Root>/yaw_sweep' */
  real32_T coll_sweep;                 /* '<Root>/coll_sweep' */
  boolean_T TrajectoryON;              /* '<Root>/TrajectoryON' */
  real32_T phi_cmd;                    /* '<Root>/phi_cmd' */
  real32_T theta_cmd;                  /* '<Root>/theta_cmd' */
  real32_T vz_cmd;                     /* '<Root>/vz_cmd' */
  real32_T psi_cmd;                    /* '<Root>/psi_cmd' */
  real32_T Vnorth_cmd;                 /* '<Root>/Vnorth_cmd' */
  real32_T Veast_cmd;                  /* '<Root>/Veast_cmd' */
  real32_T Vdown_cmd;                  /* '<Root>/Vdown_cmd' */
  real32_T Vnorth_corr;                /* '<Root>/Vnorth_corr' */
  real32_T Veast_corr;                 /* '<Root>/Veast_corr' */
  real32_T Vdown_corr;                 /* '<Root>/Vdown_corr' */
  boolean_T ScoreDisplay;              /* '<Root>/ScoreDisplay' */
  real32_T vehheadingcmd;              /* '<Root>/vehheadingcmd' */
  boolean_T ScoreOn;                   /* '<Root>/ScoreOn' */
  real32_T CF_Alt;                     /* '<Root>/CF_Alt' */
  real32_T CF_Vz;                      /* '<Root>/CF_Vz' */
} ExtY_QS_InnerRateLoop_T;

/* Real-time Model Data Structure */
struct tag_RTM_QS_InnerRateLoop_T {
  const char_T *errorStatus;
};

extern const ConstB_QS_InnerRateLoop_T QS_InnerRateLoop_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_QS_InnerRateLoop_T QS_InnerRateLoop_ConstP;

/* Class declaration for model QS_InnerRateLoop */
class QS_InnerRateLoopModelClass {
  /* public data and function members */
 public:
  /* External inputs */
  ExtU_QS_InnerRateLoop_T QS_InnerRateLoop_U;

  /* External outputs */
  ExtY_QS_InnerRateLoop_T QS_InnerRateLoop_Y;

  /* model initialize function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  void terminate();

  /* Constructor */
  QS_InnerRateLoopModelClass();

  /* Destructor */
  ~QS_InnerRateLoopModelClass();

  /* Real-Time Model get method */
  RT_MODEL_QS_InnerRateLoop_T * getRTM();

  /* private data and function members */
 private:
  /* Block signals */
  B_QS_InnerRateLoop_T QS_InnerRateLoop_B;

  /* Block states */
  DW_QS_InnerRateLoop_T QS_InnerRateLoop_DW;

  /* Real-Time Model */
  RT_MODEL_QS_InnerRateLoop_T QS_InnerRateLoop_M;
};

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'QS_InnerRateLoop'
 * '<S1>'   : 'QS_InnerRateLoop/Altitude Complimentary Filter1'
 * '<S2>'   : 'QS_InnerRateLoop/Attitude Control Loop1'
 * '<S3>'   : 'QS_InnerRateLoop/Compare To Zero'
 * '<S4>'   : 'QS_InnerRateLoop/Determine Heading at Start of Manuever  All trajectories relative to this heading'
 * '<S5>'   : 'QS_InnerRateLoop/Dynamic Inverse1'
 * '<S6>'   : 'QS_InnerRateLoop/Enabled Subsystem Grab and Freeze Value Upon Engagement'
 * '<S7>'   : 'QS_InnerRateLoop/LocalVertical 2 Inertial'
 * '<S8>'   : 'QS_InnerRateLoop/LocalVertical2Body '
 * '<S9>'   : 'QS_InnerRateLoop/NED to Local Vertical'
 * '<S10>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1'
 * '<S11>'  : 'QS_InnerRateLoop/Subsystem'
 * '<S12>'  : 'QS_InnerRateLoop/Subsystem1'
 * '<S13>'  : 'QS_InnerRateLoop/Switch Between Pilot Command and Trajectory Command1'
 * '<S14>'  : 'QS_InnerRateLoop/not equal'
 * '<S15>'  : 'QS_InnerRateLoop/not equal1'
 * '<S16>'  : 'QS_InnerRateLoop/not equal2'
 * '<S17>'  : 'QS_InnerRateLoop/not equal3'
 * '<S18>'  : 'QS_InnerRateLoop/Altitude Complimentary Filter1/2 rad//s Filter'
 * '<S19>'  : 'QS_InnerRateLoop/Altitude Complimentary Filter1/Subsystem'
 * '<S20>'  : 'QS_InnerRateLoop/Altitude Complimentary Filter1/Subsystem1'
 * '<S21>'  : 'QS_InnerRateLoop/Altitude Complimentary Filter1/Subsystem2'
 * '<S22>'  : 'QS_InnerRateLoop/Altitude Complimentary Filter1/Subsystem3'
 * '<S23>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem'
 * '<S24>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem1'
 * '<S25>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2'
 * '<S26>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3'
 * '<S27>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem/Kp Gain Schedule'
 * '<S28>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem/Kphi Gain Schedule'
 * '<S29>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem/KphiI Gain Schedule'
 * '<S30>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem/Psuedo Derivative 100*s//(s+100)'
 * '<S31>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem/Psuedo Derivative 2 100*s//(s+100)'
 * '<S32>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem/Subsystem'
 * '<S33>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem/integrator no windup'
 * '<S34>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem/lead-lag1'
 * '<S35>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem/Subsystem/k1 Gain Schedule'
 * '<S36>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem/Subsystem/k2 Gain Schedule1'
 * '<S37>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem1/KvZ Gain Schedule'
 * '<S38>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem1/KvZ Gain Schedule1'
 * '<S39>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem1/Subsystem'
 * '<S40>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem1/integrator no windup'
 * '<S41>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem1/lead-lag1'
 * '<S42>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem1/Subsystem/wcmd Gain Schedule'
 * '<S43>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2/Kq Gain Schedule'
 * '<S44>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2/Ktht Gain Schedule'
 * '<S45>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2/KthtI Gain Schedule'
 * '<S46>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2/Psuedo Derivative 100*s//(s+100)'
 * '<S47>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2/Psuedo Derivative 2 100*s//(s+100)'
 * '<S48>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2/Subsystem'
 * '<S49>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2/integrator no windup'
 * '<S50>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2/lead-lag1'
 * '<S51>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2/Subsystem/k1 Gain Schedule'
 * '<S52>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2/Subsystem/k2 Gain Schedule1'
 * '<S53>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3/-pi <= psi <= pi4'
 * '<S54>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3/Kpsi Gain Schedule'
 * '<S55>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3/KpsiI Gain Schedule'
 * '<S56>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3/Kr Gain Schedule'
 * '<S57>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3/Psuedo Derivative 100*s//(s+100)'
 * '<S58>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3/Psuedo Derivative 100*s//(s+100)1'
 * '<S59>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3/counter'
 * '<S60>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3/integrator no windup'
 * '<S61>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3/lead-lag1'
 * '<S62>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3/-pi <= psi <= pi4/0 <= psi <= 2*pi'
 * '<S63>'  : 'QS_InnerRateLoop/Dynamic Inverse1/(Cinv*Binv)^-1 Gain Schedule'
 * '<S64>'  : 'QS_InnerRateLoop/Dynamic Inverse1/Cinv*Ainv Gain Schedule'
 * '<S65>'  : 'QS_InnerRateLoop/Dynamic Inverse1/lead-lag'
 * '<S66>'  : 'QS_InnerRateLoop/Dynamic Inverse1/lead-lag1'
 * '<S67>'  : 'QS_InnerRateLoop/Dynamic Inverse1/lead-lag2'
 * '<S68>'  : 'QS_InnerRateLoop/Dynamic Inverse1/lead-lag3'
 * '<S69>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Command Transforms '
 * '<S70>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control'
 * '<S71>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Command Transforms /LocalVertical 2 Inertial'
 * '<S72>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Inertial2LocalVertical'
 * '<S73>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Lat Velocity Controller1'
 * '<S74>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Lon Velocity Controller1'
 * '<S75>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller Down1'
 * '<S76>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller East1'
 * '<S77>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller North1'
 * '<S78>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Lat Velocity Controller1/Kv Gain Schedule'
 * '<S79>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Lat Velocity Controller1/KvI Gain Schedule'
 * '<S80>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Lat Velocity Controller1/integrator no windup'
 * '<S81>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Lat Velocity Controller1/lead-lag1'
 * '<S82>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Lon Velocity Controller1/Ku Gain Schedule'
 * '<S83>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Lon Velocity Controller1/KuI Gain Schedule'
 * '<S84>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Lon Velocity Controller1/integrator no windup'
 * '<S85>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Lon Velocity Controller1/lead-lag1'
 * '<S86>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller Down1/Kdown Gain Schedule'
 * '<S87>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller Down1/Kldown Gain Schedule'
 * '<S88>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller Down1/integrator no windup'
 * '<S89>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller East1/Klatlon Gain Schedule'
 * '<S90>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller East1/KlatlonI Gain Schedule'
 * '<S91>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller East1/integrator no windup'
 * '<S92>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller North1/Klatlon Gain Schedule'
 * '<S93>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller North1/KlatlonI Gain Schedule'
 * '<S94>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller North1/integrator no windup'
 * '<S95>'  : 'QS_InnerRateLoop/Subsystem/Compare To Zero'
 * '<S96>'  : 'QS_InnerRateLoop/Subsystem/Determine if Traj Switch Has  Been Cycled since last trajectory'
 * '<S97>'  : 'QS_InnerRateLoop/Subsystem/Determine if still in position when switch was flipped to ON'
 * '<S98>'  : 'QS_InnerRateLoop/Subsystem/Enabled Subsystem1'
 * '<S99>'  : 'QS_InnerRateLoop/Subsystem/Determine if still in position when switch was flipped to ON/Compare To Zero'
 * '<S100>' : 'QS_InnerRateLoop/Subsystem/Determine if still in position when switch was flipped to ON/Compare To Zero1'
 * '<S101>' : 'QS_InnerRateLoop/Subsystem/Determine if still in position when switch was flipped to ON/Compare To Zero2'
 * '<S102>' : 'QS_InnerRateLoop/Subsystem/Determine if still in position when switch was flipped to ON/Compare To Zero3'
 * '<S103>' : 'QS_InnerRateLoop/Subsystem1/NED to Local Vertical'
 * '<S104>' : 'QS_InnerRateLoop/Switch Between Pilot Command and Trajectory Command1/Enabled Subsystem1'
 * '<S105>' : 'QS_InnerRateLoop/Switch Between Pilot Command and Trajectory Command1/Generic first order collective command model'
 * '<S106>' : 'QS_InnerRateLoop/Switch Between Pilot Command and Trajectory Command1/TFSWIT Transient Free Switch'
 * '<S107>' : 'QS_InnerRateLoop/Switch Between Pilot Command and Trajectory Command1/TFSWIT Transient Free Switch1'
 * '<S108>' : 'QS_InnerRateLoop/Switch Between Pilot Command and Trajectory Command1/TFSWIT Transient Free Switch2'
 * '<S109>' : 'QS_InnerRateLoop/Switch Between Pilot Command and Trajectory Command1/not equal'
 * '<S110>' : 'QS_InnerRateLoop/Switch Between Pilot Command and Trajectory Command1/not equal1'
 * '<S111>' : 'QS_InnerRateLoop/Switch Between Pilot Command and Trajectory Command1/not equal2'
 * '<S112>' : 'QS_InnerRateLoop/Switch Between Pilot Command and Trajectory Command1/not equal/Compare To Zero'
 * '<S113>' : 'QS_InnerRateLoop/Switch Between Pilot Command and Trajectory Command1/not equal1/Compare To Zero'
 * '<S114>' : 'QS_InnerRateLoop/Switch Between Pilot Command and Trajectory Command1/not equal2/Compare To Zero'
 * '<S115>' : 'QS_InnerRateLoop/not equal/Compare To Zero'
 * '<S116>' : 'QS_InnerRateLoop/not equal1/Compare To Zero'
 * '<S117>' : 'QS_InnerRateLoop/not equal2/Compare To Zero'
 * '<S118>' : 'QS_InnerRateLoop/not equal3/Compare To Zero'
 */
#endif                                 /* RTW_HEADER_QS_InnerRateLoop_h_ */
