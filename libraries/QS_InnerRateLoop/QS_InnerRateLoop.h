/*
 * QS_InnerRateLoop.h
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

#ifndef RTW_HEADER_QS_InnerRateLoop_h_
#define RTW_HEADER_QS_InnerRateLoop_h_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "QS_InnerRateLoop_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Block signals (default storage) */
struct B_QS_InnerRateLoop_T {
  real32_T col;                        /* '<S7>/col' */
  real32_T In1[4];                     /* '<S6>/In1' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_QS_InnerRateLoop_T {
  real_T DiscreteTimeIntegrator_DSTATE;/* '<S60>/Discrete-Time Integrator' */
  real32_T UnitDelay_DSTATE;           /* '<S20>/Unit Delay' */
  real32_T UnitDelay_DSTATE_d;         /* '<S21>/Unit Delay' */
  real32_T UnitDelay_DSTATE_a;         /* '<S22>/Unit Delay' */
  real32_T DiscreteTimeIntegrator_DSTATE_e;/* '<S22>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_c;/* '<S23>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_o;/* '<S21>/Discrete-Time Integrator' */
  real32_T UnitDelay1_DSTATE;          /* '<S19>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_ak;        /* '<S19>/Unit Delay' */
  real32_T DiscreteTimeIntegrator1_DSTATE;/* '<S19>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator_DSTATE_n;/* '<S20>/Discrete-Time Integrator' */
  real32_T UnitDelay_DSTATE_o;         /* '<S23>/Unit Delay' */
  real32_T UnitDelay_DSTATE_c;         /* '<S9>/Unit Delay' */
  real32_T UnitDelay_DSTATE_dr;        /* '<S8>/Unit Delay' */
  real32_T DiscreteTimeIntegrator_DSTATE_m;/* '<S14>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_a;/* '<S95>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_mo;/* '<S70>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_h;/* '<S92>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTAT_l;/* '<S70>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator_DSTAT_ca;/* '<S89>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator2_DSTATE;/* '<S70>/Discrete-Time Integrator2' */
  real32_T UnitDelay_DSTATE_aw;        /* '<S40>/Unit Delay' */
  real32_T DiscreteTimeIntegrator_DSTAT_hq;/* '<S32>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_b;/* '<S31>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_p;/* '<S9>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTAT_o;/* '<S33>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator_DSTAT_mk;/* '<S48>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_ou;/* '<S47>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_b3;/* '<S8>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator1_DSTAT_m;/* '<S49>/Discrete-Time Integrator1' */
  real32_T DiscreteTimeIntegrator_DSTATE_d;/* '<S58>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_mr;/* '<S59>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_f;/* '<S41>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_fs;/* '<S40>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_c4;/* '<S42>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_o2;/* '<S34>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_bw;/* '<S35>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_j;/* '<S50>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_di;/* '<S51>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_l;/* '<S61>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_hv;/* '<S62>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_co;/* '<S66>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_i;/* '<S67>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_ch;/* '<S68>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_a5;/* '<S69>/Discrete-Time Integrator' */
  real32_T UnitDelay_DSTATE_f;         /* '<S66>/Unit Delay' */
  real32_T UnitDelay_DSTATE_av;        /* '<S67>/Unit Delay' */
  real32_T UnitDelay_DSTATE_dj;        /* '<S68>/Unit Delay' */
  real32_T UnitDelay_DSTATE_e;         /* '<S69>/Unit Delay' */
  real32_T UnitDelay_DSTATE_of;        /* '<S62>/Unit Delay' */
  real32_T UnitDelay_DSTATE_eo;        /* '<S51>/Unit Delay' */
  real32_T UnitDelay_DSTATE_b;         /* '<S35>/Unit Delay' */
  real32_T UnitDelay_DSTATE_h;         /* '<S42>/Unit Delay' */
  real32_T DiscreteTimeIntegrator_DSTAT_ny;/* '<S85>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTATE_g;/* '<S86>/Discrete-Time Integrator' */
  real32_T UnitDelay1_DSTATE_c;        /* '<S49>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_j;         /* '<S49>/Unit Delay' */
  real32_T UnitDelay_DSTATE_oc;        /* '<S86>/Unit Delay' */
  real32_T DiscreteTimeIntegrator_DSTAT_cf;/* '<S81>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_cc;/* '<S82>/Discrete-Time Integrator' */
  real32_T UnitDelay1_DSTATE_a;        /* '<S33>/Unit Delay1' */
  real32_T UnitDelay_DSTATE_g;         /* '<S33>/Unit Delay' */
  real32_T UnitDelay_DSTATE_gi;        /* '<S82>/Unit Delay' */
  real32_T DiscreteTimeIntegrator_DSTAT_gh;/* '<S19>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_dn;/* '<S33>/Discrete-Time Integrator' */
  real32_T DiscreteTimeIntegrator_DSTAT_nq;/* '<S49>/Discrete-Time Integrator' */
  real32_T UnitDelay_DSTATE_k;         /* '<S96>/Unit Delay' */
  real32_T DiscreteTimeIntegrator_DSTAT_gy;/* '<S96>/Discrete-Time Integrator' */
  boolean_T Delay_DSTATE;              /* '<S76>/Delay' */
  int8_T DiscreteTimeIntegrator_PrevRese;/* '<S22>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_e;/* '<S23>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_h;/* '<S21>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_PrevRes;/* '<S19>/Discrete-Time Integrator1' */
  int8_T DiscreteTimeIntegrator_PrevRe_p;/* '<S20>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_n;/* '<S95>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_a;/* '<S70>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_o;/* '<S92>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_PrevR_b;/* '<S70>/Discrete-Time Integrator1' */
  int8_T DiscreteTimeIntegrator_PrevRe_b;/* '<S89>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator2_PrevRes;/* '<S70>/Discrete-Time Integrator2' */
  int8_T DiscreteTimeIntegrator_PrevR_ot;/* '<S9>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_PrevR_g;/* '<S33>/Discrete-Time Integrator1' */
  int8_T DiscreteTimeIntegrator_PrevR_ek;/* '<S8>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator1_PrevR_j;/* '<S49>/Discrete-Time Integrator1' */
  int8_T DiscreteTimeIntegrator_PrevR_bp;/* '<S60>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_m;/* '<S41>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevR_nq;/* '<S40>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevR_by;/* '<S42>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevR_es;/* '<S34>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_d;/* '<S50>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_j;/* '<S61>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevR_h2;/* '<S85>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevR_dc;/* '<S81>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_l;/* '<S19>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevRe_f;/* '<S33>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevR_f0;/* '<S49>/Discrete-Time Integrator' */
  int8_T DiscreteTimeIntegrator_PrevR_bv;/* '<S96>/Discrete-Time Integrator' */
  uint8_T DiscreteTimeIntegrator_IC_LOADI;/* '<S22>/Discrete-Time Integrator' */
  uint8_T DiscreteTimeIntegrator_IC_LOA_m;/* '<S21>/Discrete-Time Integrator' */
  uint8_T DiscreteTimeIntegrator1_IC_LOAD;/* '<S19>/Discrete-Time Integrator1' */
  uint8_T DiscreteTimeIntegrator_IC_LOA_g;/* '<S14>/Discrete-Time Integrator' */
  uint8_T DiscreteTimeIntegrator_IC_LOA_d;/* '<S70>/Discrete-Time Integrator' */
  uint8_T DiscreteTimeIntegrator1_IC_LO_k;/* '<S70>/Discrete-Time Integrator1' */
  uint8_T DiscreteTimeIntegrator2_IC_LOAD;/* '<S70>/Discrete-Time Integrator2' */
};

/* Invariant block signals (default storage) */
struct ConstB_QS_InnerRateLoop_T {
  real32_T TrigonometricFunction3;     /* '<S11>/Trigonometric Function3' */
  real32_T TrigonometricFunction6;     /* '<S11>/Trigonometric Function6' */
  real32_T TrigonometricFunction1;     /* '<S12>/Trigonometric Function1' */
  real32_T TrigonometricFunction2;     /* '<S12>/Trigonometric Function2' */
  real32_T TrigonometricFunction4;     /* '<S12>/Trigonometric Function4' */
  real32_T TrigonometricFunction5;     /* '<S12>/Trigonometric Function5' */
  real32_T TrigonometricFunction1_m;   /* '<S72>/Trigonometric Function1' */
  real32_T TrigonometricFunction2_a;   /* '<S72>/Trigonometric Function2' */
  real32_T TrigonometricFunction4_k;   /* '<S72>/Trigonometric Function4' */
  real32_T TrigonometricFunction5_f;   /* '<S72>/Trigonometric Function5' */
  real32_T TrigonometricFunction1_g;   /* '<S73>/Trigonometric Function1' */
  real32_T TrigonometricFunction2_ak;  /* '<S73>/Trigonometric Function2' */
  real32_T TrigonometricFunction4_km;  /* '<S73>/Trigonometric Function4' */
  real32_T TrigonometricFunction5_j;   /* '<S73>/Trigonometric Function5' */
  boolean_T LogicalOperator;           /* '<S77>/Logical Operator' */
  boolean_T LogicalOperator_a;         /* '<S78>/Logical Operator' */
};

/* Constant parameters (default storage) */
struct ConstP_QS_InnerRateLoop_T {
  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S30>/1-D Lookup Table2'
   *   '<S56>/1-D Lookup Table2'
   */
  real32_T pooled7[5];

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
  real32_T pooled8[5];

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S39>/1-D Lookup Table2'
   *   '<S80>/1-D Lookup Table2'
   */
  real32_T pooled9[5];

  /* Computed Parameter: uDLookupTable2_tableData
   * Referenced by: '<S46>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData[5];

  /* Computed Parameter: uDLookupTable2_tableData_f
   * Referenced by: '<S84>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_f[5];

  /* Computed Parameter: uDLookupTable2_tableData_e
   * Referenced by: '<S87>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_e[5];

  /* Pooled Parameter (Expression: KlatlonI)
   * Referenced by:
   *   '<S91>/1-D Lookup Table2'
   *   '<S94>/1-D Lookup Table2'
   */
  real32_T pooled13[5];

  /* Pooled Parameter (Expression: [ -8 -6 -4  -2 -1 0 1 2 4 6 8])
   * Referenced by:
   *   '<Root>/1-D Lookup Table1'
   *   '<Root>/1-D Lookup Table2'
   */
  real32_T pooled18[11];

  /* Computed Parameter: uDLookupTable2_tableData_g
   * Referenced by: '<S65>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_g[180];

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S65>/Constant'
   *   '<S65>/1-D Lookup Table2'
   */
  real32_T pooled21[36];

  /* Computed Parameter: uDLookupTable2_tableData_b
   * Referenced by: '<S64>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_b[80];

  /* Pooled Parameter (Expression: [1:16])
   * Referenced by:
   *   '<S64>/Constant'
   *   '<S64>/1-D Lookup Table2'
   */
  real32_T pooled22[16];

  /* Pooled Parameter (Expression: Klatlon)
   * Referenced by:
   *   '<S90>/1-D Lookup Table2'
   *   '<S93>/1-D Lookup Table2'
   */
  real32_T pooled23[5];

  /* Computed Parameter: uDLookupTable2_tableData_k
   * Referenced by: '<S88>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_k[5];

  /* Computed Parameter: uDLookupTable2_tableData_j
   * Referenced by: '<S43>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_j[5];

  /* Computed Parameter: uDLookupTable2_tableData_o
   * Referenced by: '<S38>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_o[5];

  /* Computed Parameter: uDLookupTable_tableData
   * Referenced by: '<S25>/1-D Lookup Table'
   */
  real32_T uDLookupTable_tableData[5];

  /* Computed Parameter: uDLookupTable1_tableData
   * Referenced by: '<S25>/1-D Lookup Table1'
   */
  real32_T uDLookupTable1_tableData[5];

  /* Computed Parameter: uDLookupTable2_tableData_a
   * Referenced by: '<S29>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_a[5];

  /* Computed Parameter: uDLookupTable2_tableData_fu
   * Referenced by: '<S28>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_fu[5];

  /* Computed Parameter: uDLookupTable_tableData_l
   * Referenced by: '<S24>/1-D Lookup Table'
   */
  real32_T uDLookupTable_tableData_l[5];

  /* Computed Parameter: uDLookupTable1_tableData_n
   * Referenced by: '<S24>/1-D Lookup Table1'
   */
  real32_T uDLookupTable1_tableData_n[5];

  /* Computed Parameter: uDLookupTable2_tableData_ex
   * Referenced by: '<S45>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_ex[5];

  /* Computed Parameter: uDLookupTable2_tableData_c
   * Referenced by: '<S44>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_c[5];

  /* Computed Parameter: uDLookupTable_tableData_m
   * Referenced by: '<S26>/1-D Lookup Table'
   */
  real32_T uDLookupTable_tableData_m[5];

  /* Computed Parameter: uDLookupTable1_tableData_d
   * Referenced by: '<S26>/1-D Lookup Table1'
   */
  real32_T uDLookupTable1_tableData_d[5];

  /* Computed Parameter: uDLookupTable2_tableData_au
   * Referenced by: '<S55>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_au[5];

  /* Computed Parameter: uDLookupTable2_tableData_c0
   * Referenced by: '<S57>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_c0[5];

  /* Computed Parameter: uDLookupTable_tableData_h
   * Referenced by: '<S27>/1-D Lookup Table'
   */
  real32_T uDLookupTable_tableData_h[5];

  /* Computed Parameter: uDLookupTable1_tableData_f
   * Referenced by: '<S27>/1-D Lookup Table1'
   */
  real32_T uDLookupTable1_tableData_f[5];

  /* Computed Parameter: uDLookupTable2_tableData_kg
   * Referenced by: '<S83>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_kg[5];

  /* Computed Parameter: uDLookupTable_tableData_d
   * Referenced by: '<S75>/1-D Lookup Table'
   */
  real32_T uDLookupTable_tableData_d[5];

  /* Computed Parameter: uDLookupTable1_tableData_m
   * Referenced by: '<S75>/1-D Lookup Table1'
   */
  real32_T uDLookupTable1_tableData_m[5];

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S36>/1-D Lookup Table2'
   *   '<S52>/1-D Lookup Table2'
   */
  real32_T pooled36[5];

  /* Pooled Parameter (Mixed Expressions)
   * Referenced by:
   *   '<S37>/1-D Lookup Table2'
   *   '<S53>/1-D Lookup Table2'
   */
  real32_T pooled37[5];

  /* Computed Parameter: uDLookupTable2_tableData_n
   * Referenced by: '<S79>/1-D Lookup Table2'
   */
  real32_T uDLookupTable2_tableData_n[5];

  /* Computed Parameter: uDLookupTable_tableData_hs
   * Referenced by: '<S74>/1-D Lookup Table'
   */
  real32_T uDLookupTable_tableData_hs[5];

  /* Computed Parameter: uDLookupTable1_tableData_p
   * Referenced by: '<S74>/1-D Lookup Table1'
   */
  real32_T uDLookupTable1_tableData_p[5];
};

/* External inputs (root inport signals with default storage) */
struct ExtU_QS_InnerRateLoop_T {
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
};

/* External outputs (root outports fed by signals with default storage) */
struct ExtY_QS_InnerRateLoop_T {
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
};

/* Real-time Model Data Structure */
struct tag_RTM_QS_InnerRateLoop_T {
  const char_T *errorStatus;
};

extern const ConstB_QS_InnerRateLoop_T QS_InnerRateLoop_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_QS_InnerRateLoop_T QS_InnerRateLoop_ConstP;

/* Class declaration for model QS_InnerRateLoop */
class QS_InnerRateLoop
{
  /* public data and function members */
 public:
  /* Real-Time Model get method */
  RT_MODEL_QS_InnerRateLoop_T * getRTM();

  /* External inputs */
  ExtU_QS_InnerRateLoop_T QS_InnerRateLoop_U;

  /* External outputs */
  ExtY_QS_InnerRateLoop_T QS_InnerRateLoop_Y;

  /* Initial conditions function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  static void terminate();

  /* Constructor */
  QS_InnerRateLoop();

  /* Destructor */
  ~QS_InnerRateLoop();

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
 * '<S7>'   : 'QS_InnerRateLoop/Enabled Subsystem1'
 * '<S8>'   : 'QS_InnerRateLoop/Generic first order  command model'
 * '<S9>'   : 'QS_InnerRateLoop/Generic first order  command model1'
 * '<S10>'  : 'QS_InnerRateLoop/LocalVertical 2 Inertial'
 * '<S11>'  : 'QS_InnerRateLoop/LocalVertical2Body '
 * '<S12>'  : 'QS_InnerRateLoop/NED to Local Vertical'
 * '<S13>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1'
 * '<S14>'  : 'QS_InnerRateLoop/Yaw Cmd Model'
 * '<S15>'  : 'QS_InnerRateLoop/not equal'
 * '<S16>'  : 'QS_InnerRateLoop/not equal1'
 * '<S17>'  : 'QS_InnerRateLoop/not equal2'
 * '<S18>'  : 'QS_InnerRateLoop/not equal3'
 * '<S19>'  : 'QS_InnerRateLoop/Altitude Complimentary Filter1/2 rad//s Filter'
 * '<S20>'  : 'QS_InnerRateLoop/Altitude Complimentary Filter1/Subsystem'
 * '<S21>'  : 'QS_InnerRateLoop/Altitude Complimentary Filter1/Subsystem1'
 * '<S22>'  : 'QS_InnerRateLoop/Altitude Complimentary Filter1/Subsystem2'
 * '<S23>'  : 'QS_InnerRateLoop/Altitude Complimentary Filter1/Subsystem3'
 * '<S24>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem'
 * '<S25>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem1'
 * '<S26>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2'
 * '<S27>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3'
 * '<S28>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem/Kp Gain Schedule'
 * '<S29>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem/Kphi Gain Schedule'
 * '<S30>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem/KphiI Gain Schedule'
 * '<S31>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem/Psuedo Derivative 100*s//(s+100)'
 * '<S32>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem/Psuedo Derivative 2 100*s//(s+100)'
 * '<S33>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem/Subsystem'
 * '<S34>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem/integrator no windup'
 * '<S35>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem/lead-lag1'
 * '<S36>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem/Subsystem/k1 Gain Schedule'
 * '<S37>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem/Subsystem/k2 Gain Schedule1'
 * '<S38>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem1/KvZ Gain Schedule'
 * '<S39>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem1/KvZ Gain Schedule1'
 * '<S40>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem1/Subsystem'
 * '<S41>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem1/integrator no windup'
 * '<S42>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem1/lead-lag1'
 * '<S43>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem1/Subsystem/wcmd Gain Schedule'
 * '<S44>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2/Kq Gain Schedule'
 * '<S45>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2/Ktht Gain Schedule'
 * '<S46>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2/KthtI Gain Schedule'
 * '<S47>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2/Psuedo Derivative 100*s//(s+100)'
 * '<S48>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2/Psuedo Derivative 2 100*s//(s+100)'
 * '<S49>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2/Subsystem'
 * '<S50>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2/integrator no windup'
 * '<S51>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2/lead-lag1'
 * '<S52>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2/Subsystem/k1 Gain Schedule'
 * '<S53>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem2/Subsystem/k2 Gain Schedule1'
 * '<S54>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3/-pi <= psi <= pi4'
 * '<S55>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3/Kpsi Gain Schedule'
 * '<S56>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3/KpsiI Gain Schedule'
 * '<S57>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3/Kr Gain Schedule'
 * '<S58>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3/Psuedo Derivative 100*s//(s+100)'
 * '<S59>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3/Psuedo Derivative 100*s//(s+100)1'
 * '<S60>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3/counter'
 * '<S61>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3/integrator no windup'
 * '<S62>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3/lead-lag1'
 * '<S63>'  : 'QS_InnerRateLoop/Attitude Control Loop1/Subsystem3/-pi <= psi <= pi4/0 <= psi <= 2*pi'
 * '<S64>'  : 'QS_InnerRateLoop/Dynamic Inverse1/(Cinv*Binv)^-1 Gain Schedule'
 * '<S65>'  : 'QS_InnerRateLoop/Dynamic Inverse1/Cinv*Ainv Gain Schedule'
 * '<S66>'  : 'QS_InnerRateLoop/Dynamic Inverse1/lead-lag'
 * '<S67>'  : 'QS_InnerRateLoop/Dynamic Inverse1/lead-lag1'
 * '<S68>'  : 'QS_InnerRateLoop/Dynamic Inverse1/lead-lag2'
 * '<S69>'  : 'QS_InnerRateLoop/Dynamic Inverse1/lead-lag3'
 * '<S70>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Command Transforms '
 * '<S71>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control'
 * '<S72>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Command Transforms /LocalVertical 2 Inertial'
 * '<S73>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Inertial2LocalVertical'
 * '<S74>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Lat Velocity Controller1'
 * '<S75>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Lon Velocity Controller1'
 * '<S76>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller Down1'
 * '<S77>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller East1'
 * '<S78>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller North1'
 * '<S79>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Lat Velocity Controller1/Kv Gain Schedule'
 * '<S80>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Lat Velocity Controller1/KvI Gain Schedule'
 * '<S81>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Lat Velocity Controller1/integrator no windup'
 * '<S82>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Lat Velocity Controller1/lead-lag1'
 * '<S83>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Lon Velocity Controller1/Ku Gain Schedule'
 * '<S84>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Lon Velocity Controller1/KuI Gain Schedule'
 * '<S85>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Lon Velocity Controller1/integrator no windup'
 * '<S86>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/Lon Velocity Controller1/lead-lag1'
 * '<S87>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller Down1/Kdown Gain Schedule'
 * '<S88>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller Down1/Kldown Gain Schedule'
 * '<S89>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller Down1/integrator no windup'
 * '<S90>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller East1/Klatlon Gain Schedule'
 * '<S91>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller East1/KlatlonI Gain Schedule'
 * '<S92>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller East1/integrator no windup'
 * '<S93>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller North1/Klatlon Gain Schedule'
 * '<S94>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller North1/KlatlonI Gain Schedule'
 * '<S95>'  : 'QS_InnerRateLoop/Outer Loop//Trajectory Control1/Position and Velocity Control/PI controller North1/integrator no windup'
 * '<S96>'  : 'QS_InnerRateLoop/Yaw Cmd Model/Generic first order collective command model'
 * '<S97>'  : 'QS_InnerRateLoop/not equal/Compare To Zero'
 * '<S98>'  : 'QS_InnerRateLoop/not equal1/Compare To Zero'
 * '<S99>'  : 'QS_InnerRateLoop/not equal2/Compare To Zero'
 * '<S100>' : 'QS_InnerRateLoop/not equal3/Compare To Zero'
 */
#endif                                 /* RTW_HEADER_QS_InnerRateLoop_h_ */
