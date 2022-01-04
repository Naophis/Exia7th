//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: mpc_tgt_calc_data.cpp
//
// Code generated for Simulink model 'mpc_tgt_calc'.
//
// Model version                  : 2.10
// Simulink Coder version         : 9.4 (R2020b) 29-Jul-2020
// C/C++ source code generated on : Mon Jan  3 10:15:30 2022
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Windows64)
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "mpc_tgt_calc.h"
#include "mpc_tgt_calc_private.h"

// Block parameters (default storage)
mpc_tgt_calcModelClass::P_mpc_tgt_calc_T mpc_tgt_calcModelClass::mpc_tgt_calc_P =
{
  // Expression: 0
  //  Referenced by: '<S29>/Constant1'

  0.0,

  // Expression: 0
  //  Referenced by: '<S28>/Constant1'

  0.0,

  // Expression: dt
  //  Referenced by: '<S28>/Gain'

  0.001,

  // Expression: 0
  //  Referenced by: '<S24>/Constant'

  0.0,

  // Expression: 0
  //  Referenced by: '<S26>/Constant'

  0.0,

  // Expression: 0
  //  Referenced by: '<S34>/Constant'

  0.0,

  // Expression: 0.01
  //  Referenced by: '<S14>/Constant1'

  0.01,

  // Expression: 2
  //  Referenced by: '<S14>/Gain'

  2.0,

  // Expression: -1
  //  Referenced by: '<S14>/Gain1'

  -1.0,

  // Expression: 0
  //  Referenced by: '<S14>/Constant'

  0.0,

  // Expression: 0
  //  Referenced by: '<S13>/Constant'

  0.0,

  // Expression: 0.01
  //  Referenced by: '<S18>/Constant1'

  0.01,

  // Expression: 2
  //  Referenced by: '<S18>/Gain'

  2.0,

  // Expression: 1
  //  Referenced by: '<S18>/Gain1'

  1.0,

  // Expression: 0
  //  Referenced by: '<S18>/Constant'

  0.0,

  // Expression: 0
  //  Referenced by: '<S17>/Constant'

  0.0,

  // Computed Parameter: Constant_Value_j
  //  Referenced by: '<S29>/Constant'

  0.0F,

  // Computed Parameter: Constant5_Value
  //  Referenced by: '<S30>/Constant5'

  1.0F,

  // Computed Parameter: Constant1_Value_m
  //  Referenced by: '<S30>/Constant1'

  1.0F,

  // Computed Parameter: Constant_Value_ne
  //  Referenced by: '<S30>/Constant'

  1.0F,

  // Expression: dt
  //  Referenced by: '<S30>/Gain'

  0.001F,

  // Computed Parameter: Constant6_Value
  //  Referenced by: '<S30>/Constant6'

  1.0F,

  // Computed Parameter: Constant4_Value
  //  Referenced by: '<S30>/Constant4'

  1.0F,

  // Computed Parameter: Constant2_Value
  //  Referenced by: '<S30>/Constant2'

  1.0F,

  // Computed Parameter: Gain1_Gain_l
  //  Referenced by: '<S30>/Gain1'

  -1.0F,

  // Computed Parameter: Constant4_Value_i
  //  Referenced by: '<S24>/Constant4'

  2.5F,

  // Computed Parameter: Constant3_Value
  //  Referenced by: '<S24>/Constant3'

  1.0F,

  // Computed Parameter: Gain3_Gain
  //  Referenced by: '<S23>/Gain3'

  -1.0F,

  // Computed Parameter: Gain2_Gain
  //  Referenced by: '<S23>/Gain2'

  -1.0F,

  // Computed Parameter: Gain1_Gain_m
  //  Referenced by: '<S23>/Gain1'

  2.0F,

  // Computed Parameter: Switch_Threshold
  //  Referenced by: '<S23>/Switch'

  0.0F,

  // Expression: dt
  //  Referenced by: '<S5>/Gain'

  0.001F,

  // Computed Parameter: Constant_Value_pr
  //  Referenced by: '<S5>/Constant'

  0.0F,

  // Computed Parameter: Gain2_Gain_i
  //  Referenced by: '<S31>/Gain2'

  -1.0F,

  // Computed Parameter: Gain1_Gain_h
  //  Referenced by: '<S31>/Gain1'

  2.0F,

  // Computed Parameter: Gain_Gain_m
  //  Referenced by: '<S31>/Gain'

  1.0F,

  // Computed Parameter: Gain4_Gain
  //  Referenced by: '<S31>/Gain4'

  0.5F,

  // Expression: dt
  //  Referenced by: '<S8>/Gain'

  0.001F,

  // Computed Parameter: Constant_Value_m
  //  Referenced by: '<S8>/Constant'

  0.0F,

  // Computed Parameter: Constant_Value_e
  //  Referenced by: '<S9>/Constant'

  0.0F,

  // Expression: dt
  //  Referenced by: '<S9>/Gain'

  0.001F,

  // Computed Parameter: Constant3_Value_c
  //  Referenced by: '<S13>/Constant3'

  1.0F,

  // Computed Parameter: Gain1_Gain_o
  //  Referenced by: '<S12>/Gain1'

  2.0F,

  // Computed Parameter: Merge_InitialOutput
  //  Referenced by: '<S12>/Merge'

  0.0F,

  // Expression: dt
  //  Referenced by: '<S10>/Gain'

  0.001F,

  // Computed Parameter: Constant_Value_nex
  //  Referenced by: '<S10>/Constant'

  0.0F,

  // Computed Parameter: Constant4_Value_c
  //  Referenced by: '<S17>/Constant4'

  2.5F,

  // Computed Parameter: Constant3_Value_cn
  //  Referenced by: '<S17>/Constant3'

  1.0F,

  // Computed Parameter: Gain1_Gain_n
  //  Referenced by: '<S16>/Gain1'

  2.0F,

  // Computed Parameter: Merge_InitialOutput_m
  //  Referenced by: '<S16>/Merge'

  0.0F,

  // Expression: dt
  //  Referenced by: '<S11>/Gain'

  0.001F,

  // Computed Parameter: Constant_Value_h
  //  Referenced by: '<S11>/Constant'

  0.0F,

  // Computed Parameter: Constant1_Value_i
  //  Referenced by: '<S20>/Constant1'

  1.0F,

  // Computed Parameter: Constant_Value_hn
  //  Referenced by: '<S20>/Constant'

  1.0F,

  // Computed Parameter: Saturation_UpperSat
  //  Referenced by: '<S20>/Saturation'

  1.0F,

  // Computed Parameter: Saturation_LowerSat
  //  Referenced by: '<S20>/Saturation'

  0.0F,

  // Computed Parameter: Constant1_Value_op
  //  Referenced by: '<S21>/Constant1'

  1.0F,

  // Computed Parameter: Constant_Value_my
  //  Referenced by: '<S21>/Constant'

  1.0F,

  // Computed Parameter: Saturation_UpperSat_i
  //  Referenced by: '<S21>/Saturation'

  1.0F,

  // Computed Parameter: Saturation_LowerSat_e
  //  Referenced by: '<S21>/Saturation'

  0.0F,

  // Computed Parameter: mms2ms_Gain
  //  Referenced by: '<S27>/mm//s 2 m//s'

  0.001F,

  // Computed Parameter: Constant1_Value_hj
  //  Referenced by: '<S27>/Constant1'

  1000.0F,

  // Computed Parameter: Constant_Value_g
  //  Referenced by: '<S27>/Constant'

  1000.0F,

  // Computed Parameter: Constant3_Value_a
  //  Referenced by: '<S27>/Constant3'

  0.0F,

  // Expression: dt
  //  Referenced by: '<S2>/Gain1'

  0.001F,

  // Expression: dt
  //  Referenced by: '<S3>/Gain1'

  0.001F,

  // Expression: dt
  //  Referenced by: '<S27>/Gain3'

  0.001F,

  // Expression: dt
  //  Referenced by: '<S27>/Gain4'

  0.001F,

  // Expression: dt
  //  Referenced by: '<S6>/Gain1'

  0.001F,

  // Expression: dt
  //  Referenced by: '<S6>/Gain2'

  0.001F,

  // Computed Parameter: Gain5_Gain
  //  Referenced by: '<S6>/Gain5'

  1.0F,

  // Computed Parameter: Gain6_Gain
  //  Referenced by: '<S6>/Gain6'

  1.0F,

  // Expression: dt
  //  Referenced by: '<S4>/Gain'

  0.001F,

  // Computed Parameter: DataStoreMemory_InitialValue
  //  Referenced by: '<Root>/Data Store Memory'

  0.0F,

  // Computed Parameter: Constant1_Value_p
  //  Referenced by: '<S24>/Constant1'

  0,

  // Computed Parameter: Constant2_Value_f
  //  Referenced by: '<S24>/Constant2'

  1,

  // Computed Parameter: Constant2_Value_g
  //  Referenced by: '<S26>/Constant2'

  1,

  // Computed Parameter: Constant1_Value_c
  //  Referenced by: '<S5>/Constant1'

  3,

  // Computed Parameter: Constant1_Value_oj
  //  Referenced by: '<S32>/Constant1'

  0,

  // Computed Parameter: Constant2_Value_j
  //  Referenced by: '<S34>/Constant2'

  1,

  // Computed Parameter: Constant1_Value_e
  //  Referenced by: '<S8>/Constant1'

  3,

  // Computed Parameter: Constant2_Value_p
  //  Referenced by: '<S14>/Constant2'

  2,

  // Computed Parameter: Constant1_Value_l
  //  Referenced by: '<S13>/Constant1'

  0,

  // Computed Parameter: Constant2_Value_gw
  //  Referenced by: '<S13>/Constant2'

  1,

  // Computed Parameter: Constant2_Value_i
  //  Referenced by: '<S18>/Constant2'

  2,

  // Computed Parameter: Constant1_Value_k
  //  Referenced by: '<S17>/Constant1'

  0,

  // Computed Parameter: Constant2_Value_d
  //  Referenced by: '<S17>/Constant2'

  1,

  // Computed Parameter: ManualSwitch_CurrentSetting
  //  Referenced by: '<S24>/Manual Switch'

  1U,

  // Computed Parameter: ManualSwitch_CurrentSetting_i
  //  Referenced by: '<S14>/Manual Switch'

  0U,

  // Computed Parameter: ManualSwitch_CurrentSetting_e
  //  Referenced by: '<S18>/Manual Switch'

  0U,

  // Computed Parameter: ManualSwitch_CurrentSetting_b
  //  Referenced by: '<S17>/Manual Switch'

  0U,

  // Start of '<S16>/keep'
  {
    // Expression: 0
    //  Referenced by: '<S19>/Constant'

    0.0,

    // Computed Parameter: Constant2_Value
    //  Referenced by: '<S19>/Constant2'

    1
  }
  ,

  // End of '<S16>/keep'

  // Start of '<S12>/keep'
  {
    // Expression: 0
    //  Referenced by: '<S15>/Constant'

    0.0,

    // Computed Parameter: Constant2_Value
    //  Referenced by: '<S15>/Constant2'

    1
  }
  ,

  // End of '<S12>/keep'

  // Start of '<S31>/decel'
  {
    // Expression: 0.01*2*pi/360
    //  Referenced by: '<S33>/Constant1'

    0.00017453292519943296,

    // Expression: 2
    //  Referenced by: '<S33>/Gain'

    2.0,

    // Computed Parameter: Gain1_Gain
    //  Referenced by: '<S33>/Gain1'

    -1.0F,

    // Computed Parameter: Switch1_Threshold
    //  Referenced by: '<S33>/Switch1'

    0.0F,

    // Computed Parameter: Constant_Value
    //  Referenced by: '<S33>/Constant'

    0.0F,

    // Computed Parameter: Switch2_Threshold
    //  Referenced by: '<S33>/Switch2'

    0.0F,

    // Computed Parameter: Constant2_Value
    //  Referenced by: '<S33>/Constant2'

    2,

    // Computed Parameter: ManualSwitch_CurrentSetting
    //  Referenced by: '<S33>/Manual Switch'

    0U
  }
  ,

  // End of '<S31>/decel'

  // Start of '<S23>/decel'
  {
    // Expression: 0.01*2*pi/360
    //  Referenced by: '<S25>/Constant1'

    0.00017453292519943296,

    // Expression: 2
    //  Referenced by: '<S25>/Gain'

    2.0,

    // Computed Parameter: Gain1_Gain
    //  Referenced by: '<S25>/Gain1'

    -1.0F,

    // Computed Parameter: Switch1_Threshold
    //  Referenced by: '<S25>/Switch1'

    0.0F,

    // Computed Parameter: Constant_Value
    //  Referenced by: '<S25>/Constant'

    0.0F,

    // Computed Parameter: Switch2_Threshold
    //  Referenced by: '<S25>/Switch2'

    0.0F,

    // Computed Parameter: Constant2_Value
    //  Referenced by: '<S25>/Constant2'

    2,

    // Computed Parameter: ManualSwitch_CurrentSetting
    //  Referenced by: '<S25>/Manual Switch'

    0U
  }
  // End of '<S23>/decel'
};

//
// File trailer for generated code.
//
// [EOF]
//
