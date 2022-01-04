//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: mpc_tgt_calc.h
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
#ifndef RTW_HEADER_mpc_tgt_calc_h_
#define RTW_HEADER_mpc_tgt_calc_h_
#include <cmath>
#include "rtwtypes.h"
#include "mpc_tgt_calc_types.h"
#include "rt_nonfinite.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

// user code (top of header file)
#include "bus.h"

// Class declaration for model mpc_tgt_calc
class mpc_tgt_calcModelClass {
  // public data and function members
 public:
  // Parameters for system: '<S23>/decel'
  struct P_decel_mpc_tgt_calc_T {
    real_T Constant1_Value;            // Expression: 0.01*2*pi/360
                                          //  Referenced by: '<S25>/Constant1'

    real_T Gain_Gain;                  // Expression: 2
                                          //  Referenced by: '<S25>/Gain'

    real32_T Gain1_Gain;               // Computed Parameter: Gain1_Gain
                                          //  Referenced by: '<S25>/Gain1'

    real32_T Switch1_Threshold;        // Computed Parameter: Switch1_Threshold
                                          //  Referenced by: '<S25>/Switch1'

    real32_T Constant_Value;           // Computed Parameter: Constant_Value
                                          //  Referenced by: '<S25>/Constant'

    real32_T Switch2_Threshold;        // Computed Parameter: Switch2_Threshold
                                          //  Referenced by: '<S25>/Switch2'

    int8_T Constant2_Value;            // Computed Parameter: Constant2_Value
                                          //  Referenced by: '<S25>/Constant2'

    uint8_T ManualSwitch_CurrentSetting;
                              // Computed Parameter: ManualSwitch_CurrentSetting
                                 //  Referenced by: '<S25>/Manual Switch'

  };

  // Parameters for system: '<S12>/keep'
  struct P_keep_mpc_tgt_calc_T {
    real_T Constant_Value;             // Expression: 0
                                          //  Referenced by: '<S15>/Constant'

    int8_T Constant2_Value;            // Computed Parameter: Constant2_Value
                                          //  Referenced by: '<S15>/Constant2'

  };

  // Parameters (default storage)
  struct P_mpc_tgt_calc_T {
    real_T Constant1_Value;            // Expression: 0
                                          //  Referenced by: '<S29>/Constant1'

    real_T Constant1_Value_d;          // Expression: 0
                                          //  Referenced by: '<S28>/Constant1'

    real_T Gain_Gain;                  // Expression: dt
                                          //  Referenced by: '<S28>/Gain'

    real_T Constant_Value;             // Expression: 0
                                          //  Referenced by: '<S24>/Constant'

    real_T Constant_Value_n;           // Expression: 0
                                          //  Referenced by: '<S26>/Constant'

    real_T Constant_Value_p;           // Expression: 0
                                          //  Referenced by: '<S34>/Constant'

    real_T Constant1_Value_o;          // Expression: 0.01
                                          //  Referenced by: '<S14>/Constant1'

    real_T Gain_Gain_n;                // Expression: 2
                                          //  Referenced by: '<S14>/Gain'

    real_T Gain1_Gain;                 // Expression: -1
                                          //  Referenced by: '<S14>/Gain1'

    real_T Constant_Value_nl;          // Expression: 0
                                          //  Referenced by: '<S14>/Constant'

    real_T Constant_Value_c;           // Expression: 0
                                          //  Referenced by: '<S13>/Constant'

    real_T Constant1_Value_h;          // Expression: 0.01
                                          //  Referenced by: '<S18>/Constant1'

    real_T Gain_Gain_i;                // Expression: 2
                                          //  Referenced by: '<S18>/Gain'

    real_T Gain1_Gain_c;               // Expression: 1
                                          //  Referenced by: '<S18>/Gain1'

    real_T Constant_Value_cx;          // Expression: 0
                                          //  Referenced by: '<S18>/Constant'

    real_T Constant_Value_cf;          // Expression: 0
                                          //  Referenced by: '<S17>/Constant'

    real32_T Constant_Value_j;         // Computed Parameter: Constant_Value_j
                                          //  Referenced by: '<S29>/Constant'

    real32_T Constant5_Value;          // Computed Parameter: Constant5_Value
                                          //  Referenced by: '<S30>/Constant5'

    real32_T Constant1_Value_m;        // Computed Parameter: Constant1_Value_m
                                          //  Referenced by: '<S30>/Constant1'

    real32_T Constant_Value_ne;        // Computed Parameter: Constant_Value_ne
                                          //  Referenced by: '<S30>/Constant'

    real32_T Gain_Gain_e;              // Expression: dt
                                          //  Referenced by: '<S30>/Gain'

    real32_T Constant6_Value;          // Computed Parameter: Constant6_Value
                                          //  Referenced by: '<S30>/Constant6'

    real32_T Constant4_Value;          // Computed Parameter: Constant4_Value
                                          //  Referenced by: '<S30>/Constant4'

    real32_T Constant2_Value;          // Computed Parameter: Constant2_Value
                                          //  Referenced by: '<S30>/Constant2'

    real32_T Gain1_Gain_l;             // Computed Parameter: Gain1_Gain_l
                                          //  Referenced by: '<S30>/Gain1'

    real32_T Constant4_Value_i;        // Computed Parameter: Constant4_Value_i
                                          //  Referenced by: '<S24>/Constant4'

    real32_T Constant3_Value;          // Computed Parameter: Constant3_Value
                                          //  Referenced by: '<S24>/Constant3'

    real32_T Gain3_Gain;               // Computed Parameter: Gain3_Gain
                                          //  Referenced by: '<S23>/Gain3'

    real32_T Gain2_Gain;               // Computed Parameter: Gain2_Gain
                                          //  Referenced by: '<S23>/Gain2'

    real32_T Gain1_Gain_m;             // Computed Parameter: Gain1_Gain_m
                                          //  Referenced by: '<S23>/Gain1'

    real32_T Switch_Threshold;         // Computed Parameter: Switch_Threshold
                                          //  Referenced by: '<S23>/Switch'

    real32_T Gain_Gain_d;              // Expression: dt
                                          //  Referenced by: '<S5>/Gain'

    real32_T Constant_Value_pr;        // Computed Parameter: Constant_Value_pr
                                          //  Referenced by: '<S5>/Constant'

    real32_T Gain2_Gain_i;             // Computed Parameter: Gain2_Gain_i
                                          //  Referenced by: '<S31>/Gain2'

    real32_T Gain1_Gain_h;             // Computed Parameter: Gain1_Gain_h
                                          //  Referenced by: '<S31>/Gain1'

    real32_T Gain_Gain_m;              // Computed Parameter: Gain_Gain_m
                                          //  Referenced by: '<S31>/Gain'

    real32_T Gain4_Gain;               // Computed Parameter: Gain4_Gain
                                          //  Referenced by: '<S31>/Gain4'

    real32_T Gain_Gain_c;              // Expression: dt
                                          //  Referenced by: '<S8>/Gain'

    real32_T Constant_Value_m;         // Computed Parameter: Constant_Value_m
                                          //  Referenced by: '<S8>/Constant'

    real32_T Constant_Value_e;         // Computed Parameter: Constant_Value_e
                                          //  Referenced by: '<S9>/Constant'

    real32_T Gain_Gain_b;              // Expression: dt
                                          //  Referenced by: '<S9>/Gain'

    real32_T Constant3_Value_c;        // Computed Parameter: Constant3_Value_c
                                          //  Referenced by: '<S13>/Constant3'

    real32_T Gain1_Gain_o;             // Computed Parameter: Gain1_Gain_o
                                          //  Referenced by: '<S12>/Gain1'

    real32_T Merge_InitialOutput;     // Computed Parameter: Merge_InitialOutput
                                         //  Referenced by: '<S12>/Merge'

    real32_T Gain_Gain_ca;             // Expression: dt
                                          //  Referenced by: '<S10>/Gain'

    real32_T Constant_Value_nex;       // Computed Parameter: Constant_Value_nex
                                          //  Referenced by: '<S10>/Constant'

    real32_T Constant4_Value_c;        // Computed Parameter: Constant4_Value_c
                                          //  Referenced by: '<S17>/Constant4'

    real32_T Constant3_Value_cn;       // Computed Parameter: Constant3_Value_cn
                                          //  Referenced by: '<S17>/Constant3'

    real32_T Gain1_Gain_n;             // Computed Parameter: Gain1_Gain_n
                                          //  Referenced by: '<S16>/Gain1'

    real32_T Merge_InitialOutput_m; // Computed Parameter: Merge_InitialOutput_m
                                       //  Referenced by: '<S16>/Merge'

    real32_T Gain_Gain_k;              // Expression: dt
                                          //  Referenced by: '<S11>/Gain'

    real32_T Constant_Value_h;         // Computed Parameter: Constant_Value_h
                                          //  Referenced by: '<S11>/Constant'

    real32_T Constant1_Value_i;        // Computed Parameter: Constant1_Value_i
                                          //  Referenced by: '<S20>/Constant1'

    real32_T Constant_Value_hn;        // Computed Parameter: Constant_Value_hn
                                          //  Referenced by: '<S20>/Constant'

    real32_T Saturation_UpperSat;     // Computed Parameter: Saturation_UpperSat
                                         //  Referenced by: '<S20>/Saturation'

    real32_T Saturation_LowerSat;     // Computed Parameter: Saturation_LowerSat
                                         //  Referenced by: '<S20>/Saturation'

    real32_T Constant1_Value_op;       // Computed Parameter: Constant1_Value_op
                                          //  Referenced by: '<S21>/Constant1'

    real32_T Constant_Value_my;        // Computed Parameter: Constant_Value_my
                                          //  Referenced by: '<S21>/Constant'

    real32_T Saturation_UpperSat_i; // Computed Parameter: Saturation_UpperSat_i
                                       //  Referenced by: '<S21>/Saturation'

    real32_T Saturation_LowerSat_e; // Computed Parameter: Saturation_LowerSat_e
                                       //  Referenced by: '<S21>/Saturation'

    real32_T mms2ms_Gain;              // Computed Parameter: mms2ms_Gain
                                          //  Referenced by: '<S27>/mm//s 2 m//s'

    real32_T Constant1_Value_hj;       // Computed Parameter: Constant1_Value_hj
                                          //  Referenced by: '<S27>/Constant1'

    real32_T Constant_Value_g;         // Computed Parameter: Constant_Value_g
                                          //  Referenced by: '<S27>/Constant'

    real32_T Constant3_Value_a;        // Computed Parameter: Constant3_Value_a
                                          //  Referenced by: '<S27>/Constant3'

    real32_T Gain1_Gain_e;             // Expression: dt
                                          //  Referenced by: '<S2>/Gain1'

    real32_T Gain1_Gain_g;             // Expression: dt
                                          //  Referenced by: '<S3>/Gain1'

    real32_T Gain3_Gain_m;             // Expression: dt
                                          //  Referenced by: '<S27>/Gain3'

    real32_T Gain4_Gain_o;             // Expression: dt
                                          //  Referenced by: '<S27>/Gain4'

    real32_T Gain1_Gain_cq;            // Expression: dt
                                          //  Referenced by: '<S6>/Gain1'

    real32_T Gain2_Gain_c;             // Expression: dt
                                          //  Referenced by: '<S6>/Gain2'

    real32_T Gain5_Gain;               // Computed Parameter: Gain5_Gain
                                          //  Referenced by: '<S6>/Gain5'

    real32_T Gain6_Gain;               // Computed Parameter: Gain6_Gain
                                          //  Referenced by: '<S6>/Gain6'

    real32_T Gain_Gain_kq;             // Expression: dt
                                          //  Referenced by: '<S4>/Gain'

    real32_T DataStoreMemory_InitialValue;
                             // Computed Parameter: DataStoreMemory_InitialValue
                                //  Referenced by: '<Root>/Data Store Memory'

    int8_T Constant1_Value_p;          // Computed Parameter: Constant1_Value_p
                                          //  Referenced by: '<S24>/Constant1'

    int8_T Constant2_Value_f;          // Computed Parameter: Constant2_Value_f
                                          //  Referenced by: '<S24>/Constant2'

    int8_T Constant2_Value_g;          // Computed Parameter: Constant2_Value_g
                                          //  Referenced by: '<S26>/Constant2'

    int8_T Constant1_Value_c;          // Computed Parameter: Constant1_Value_c
                                          //  Referenced by: '<S5>/Constant1'

    int8_T Constant1_Value_oj;         // Computed Parameter: Constant1_Value_oj
                                          //  Referenced by: '<S32>/Constant1'

    int8_T Constant2_Value_j;          // Computed Parameter: Constant2_Value_j
                                          //  Referenced by: '<S34>/Constant2'

    int8_T Constant1_Value_e;          // Computed Parameter: Constant1_Value_e
                                          //  Referenced by: '<S8>/Constant1'

    int8_T Constant2_Value_p;          // Computed Parameter: Constant2_Value_p
                                          //  Referenced by: '<S14>/Constant2'

    int8_T Constant1_Value_l;          // Computed Parameter: Constant1_Value_l
                                          //  Referenced by: '<S13>/Constant1'

    int8_T Constant2_Value_gw;         // Computed Parameter: Constant2_Value_gw
                                          //  Referenced by: '<S13>/Constant2'

    int8_T Constant2_Value_i;          // Computed Parameter: Constant2_Value_i
                                          //  Referenced by: '<S18>/Constant2'

    int8_T Constant1_Value_k;          // Computed Parameter: Constant1_Value_k
                                          //  Referenced by: '<S17>/Constant1'

    int8_T Constant2_Value_d;          // Computed Parameter: Constant2_Value_d
                                          //  Referenced by: '<S17>/Constant2'

    uint8_T ManualSwitch_CurrentSetting;
                              // Computed Parameter: ManualSwitch_CurrentSetting
                                 //  Referenced by: '<S24>/Manual Switch'

    uint8_T ManualSwitch_CurrentSetting_i;
                            // Computed Parameter: ManualSwitch_CurrentSetting_i
                               //  Referenced by: '<S14>/Manual Switch'

    uint8_T ManualSwitch_CurrentSetting_e;
                            // Computed Parameter: ManualSwitch_CurrentSetting_e
                               //  Referenced by: '<S18>/Manual Switch'

    uint8_T ManualSwitch_CurrentSetting_b;
                            // Computed Parameter: ManualSwitch_CurrentSetting_b
                               //  Referenced by: '<S17>/Manual Switch'

    P_keep_mpc_tgt_calc_T keep_p;      // '<S16>/keep'
    P_keep_mpc_tgt_calc_T keep_h;      // '<S12>/keep'
    P_decel_mpc_tgt_calc_T decel_d;    // '<S31>/decel'
    P_decel_mpc_tgt_calc_T decel;      // '<S23>/decel'
  };

  // Real-time Model Data Structure
  struct RT_MODEL_mpc_tgt_calc_T {
    const char_T * volatile errorStatus;
  };

  // model data, for system '<S23>/decel'
  typedef struct {
    P_decel_mpc_tgt_calc_T* defaultParam;
  } self_decel_mpc_tgt_calc_T;

  // model data, for system '<S12>/keep'
  typedef struct {
    P_keep_mpc_tgt_calc_T* defaultParam;
  } self_keep_mpc_tgt_calc_T;

  // model initialize function
  void initialize();

  // model step function
  void step(const t_tgt *arg_tgt, const t_ego *arg_ego, int32_T arg_mode,
            int32_T arg_time_step, t_ego *arg_next_ego);

  // model terminate function
  void terminate();

  // Constructor
  mpc_tgt_calcModelClass();

  // Destructor
  ~mpc_tgt_calcModelClass();

  // Real-Time Model get method
  mpc_tgt_calcModelClass::RT_MODEL_mpc_tgt_calc_T * getRTM();

  // private data and function members
 private:
  // Tunable parameters
  static P_mpc_tgt_calc_T mpc_tgt_calc_P;

  // Block variables
  self_keep_mpc_tgt_calc_T self_keep_p;
  self_keep_mpc_tgt_calc_T self_keep_h;
  self_decel_mpc_tgt_calc_T self_decel_d;
  self_decel_mpc_tgt_calc_T self_decel;

  // Real-Time Model
  RT_MODEL_mpc_tgt_calc_T mpc_tgt_calc_M;

  // private member function(s) for subsystem '<S23>/decel'
  static void mpc_tgt_calc_decel(self_decel_mpc_tgt_calc_T
    *mpc_tgt_calc_self_arg, real32_T rtu_decel_alpha, real32_T rtu_now_w,
    real32_T rtu_end_w, real32_T rtu_ang, real32_T *rty_decel_out, int8_T
    *rty_state_out);

  // private member function(s) for subsystem '<S12>/keep'
  static void mpc_tgt_calc_keep(self_keep_mpc_tgt_calc_T *mpc_tgt_calc_self_arg,
    real32_T *rty_accl_out, int8_T *rty_state_out);
};

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Use the MATLAB hilite_system command to trace the generated code back
//  to the model.  For example,
//
//  hilite_system('<S3>')    - opens system 3
//  hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'mpc_tgt_calc'
//  '<S1>'   : 'mpc_tgt_calc/Subsystem'
//  '<S2>'   : 'mpc_tgt_calc/Subsystem1'
//  '<S3>'   : 'mpc_tgt_calc/Subsystem3'
//  '<S4>'   : 'mpc_tgt_calc/decel_ratio'
//  '<S5>'   : 'mpc_tgt_calc/pivot'
//  '<S6>'   : 'mpc_tgt_calc/position_estimate'
//  '<S7>'   : 'mpc_tgt_calc/slalom'
//  '<S8>'   : 'mpc_tgt_calc/slalom2'
//  '<S9>'   : 'mpc_tgt_calc/undefined'
//  '<S10>'  : 'mpc_tgt_calc/Subsystem/straight_ahead'
//  '<S11>'  : 'mpc_tgt_calc/Subsystem/straight_back'
//  '<S12>'  : 'mpc_tgt_calc/Subsystem/straight_ahead/Subsystem'
//  '<S13>'  : 'mpc_tgt_calc/Subsystem/straight_ahead/Subsystem/accl'
//  '<S14>'  : 'mpc_tgt_calc/Subsystem/straight_ahead/Subsystem/decel'
//  '<S15>'  : 'mpc_tgt_calc/Subsystem/straight_ahead/Subsystem/keep'
//  '<S16>'  : 'mpc_tgt_calc/Subsystem/straight_back/Subsystem'
//  '<S17>'  : 'mpc_tgt_calc/Subsystem/straight_back/Subsystem/accl'
//  '<S18>'  : 'mpc_tgt_calc/Subsystem/straight_back/Subsystem/decel'
//  '<S19>'  : 'mpc_tgt_calc/Subsystem/straight_back/Subsystem/keep'
//  '<S20>'  : 'mpc_tgt_calc/decel_ratio/If Action Subsystem'
//  '<S21>'  : 'mpc_tgt_calc/decel_ratio/If Action Subsystem1'
//  '<S22>'  : 'mpc_tgt_calc/decel_ratio/If Action Subsystem2'
//  '<S23>'  : 'mpc_tgt_calc/pivot/Subsystem'
//  '<S24>'  : 'mpc_tgt_calc/pivot/Subsystem/accl'
//  '<S25>'  : 'mpc_tgt_calc/pivot/Subsystem/decel'
//  '<S26>'  : 'mpc_tgt_calc/pivot/Subsystem/keep'
//  '<S27>'  : 'mpc_tgt_calc/position_estimate/slipped_position_estimate'
//  '<S28>'  : 'mpc_tgt_calc/slalom/Subsystem2'
//  '<S29>'  : 'mpc_tgt_calc/slalom/Subsystem2/Subsystem'
//  '<S30>'  : 'mpc_tgt_calc/slalom/Subsystem2/Subsystem/calc_neiper'
//  '<S31>'  : 'mpc_tgt_calc/slalom2/Subsystem'
//  '<S32>'  : 'mpc_tgt_calc/slalom2/Subsystem/accl'
//  '<S33>'  : 'mpc_tgt_calc/slalom2/Subsystem/decel'
//  '<S34>'  : 'mpc_tgt_calc/slalom2/Subsystem/keep'

#endif                                 // RTW_HEADER_mpc_tgt_calc_h_

//
// File trailer for generated code.
//
// [EOF]
//
