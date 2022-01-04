//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: mpc_tgt_calc.cpp
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

//
// Output and update for action system:
//    '<S23>/decel'
//    '<S31>/decel'
//
void mpc_tgt_calcModelClass::mpc_tgt_calc_decel(self_decel_mpc_tgt_calc_T
  *mpc_tgt_calc_self_arg, real32_T rtu_decel_alpha, real32_T rtu_now_w, real32_T
  rtu_end_w, real32_T rtu_ang, real32_T *rty_decel_out, int8_T *rty_state_out)
{
  mpc_tgt_calcModelClass::P_decel_mpc_tgt_calc_T *localP_0;
  real_T tmp;
  boolean_T rtu_decel_alpha_0;
  localP_0 = mpc_tgt_calc_self_arg->defaultParam;

  // Switch: '<S25>/Switch2' incorporates:
  //   RelationalOperator: '<S25>/Relational Operator'
  //   RelationalOperator: '<S25>/Relational Operator1'

  if (rtu_decel_alpha > localP_0->Switch2_Threshold) {
    rtu_decel_alpha_0 = rtu_now_w < rtu_end_w;
  } else {
    rtu_decel_alpha_0 = rtu_now_w > rtu_end_w;
  }

  // End of Switch: '<S25>/Switch2'

  // Switch: '<S25>/Switch' incorporates:
  //   Constant: '<S25>/Constant'

  if (rtu_decel_alpha_0) {
    // ManualSwitch: '<S25>/Manual Switch' incorporates:
    //   Constant: '<S25>/Constant1'
    //   MinMax: '<S25>/Min'
    //   Switch: '<S25>/Switch1'

    if (localP_0->ManualSwitch_CurrentSetting == 1) {
      *rty_decel_out = rtu_decel_alpha;
    } else if (rtu_decel_alpha > localP_0->Switch1_Threshold) {
      // MinMax: '<S25>/Min' incorporates:
      //   Constant: '<S25>/Constant1'

      if (localP_0->Constant1_Value > rtu_ang || rtIsNaN(static_cast<real_T>
           (rtu_ang))) {
        tmp = localP_0->Constant1_Value;
      } else {
        tmp = rtu_ang;
      }

      // Switch: '<S25>/Switch1' incorporates:
      //   Abs: '<S25>/Abs'
      //   DataTypeConversion: '<S25>/Data Type Conversion'
      //   Gain: '<S25>/Gain'
      //   Math: '<S25>/Square'
      //   Math: '<S25>/Square1'
      //   Product: '<S25>/Divide'
      //   Sum: '<S25>/Subtract'

      *rty_decel_out = static_cast<real32_T>(std::abs((rtu_now_w * rtu_now_w -
        rtu_end_w * rtu_end_w) / (localP_0->Gain_Gain * tmp)));
    } else {
      if (localP_0->Constant1_Value > rtu_ang || rtIsNaN(static_cast<real_T>
           (rtu_ang))) {
        // MinMax: '<S25>/Min' incorporates:
        //   Constant: '<S25>/Constant1'

        tmp = localP_0->Constant1_Value;
      } else {
        // MinMax: '<S25>/Min'
        tmp = rtu_ang;
      }

      // Switch: '<S25>/Switch1' incorporates:
      //   Abs: '<S25>/Abs'
      //   DataTypeConversion: '<S25>/Data Type Conversion'
      //   Gain: '<S25>/Gain'
      //   Gain: '<S25>/Gain1'
      //   Math: '<S25>/Square'
      //   Math: '<S25>/Square1'
      //   Product: '<S25>/Divide'
      //   Sum: '<S25>/Subtract'

      *rty_decel_out = static_cast<real32_T>(std::abs((rtu_now_w * rtu_now_w -
        rtu_end_w * rtu_end_w) / (localP_0->Gain_Gain * tmp))) *
        localP_0->Gain1_Gain;
    }

    // End of ManualSwitch: '<S25>/Manual Switch'
  } else {
    *rty_decel_out = localP_0->Constant_Value;
  }

  // End of Switch: '<S25>/Switch'

  // SignalConversion generated from: '<S25>/state_out' incorporates:
  //   Constant: '<S25>/Constant2'

  *rty_state_out = localP_0->Constant2_Value;
}

//
// Output and update for action system:
//    '<S12>/keep'
//    '<S16>/keep'
//
void mpc_tgt_calcModelClass::mpc_tgt_calc_keep(self_keep_mpc_tgt_calc_T
  *mpc_tgt_calc_self_arg, real32_T *rty_accl_out, int8_T *rty_state_out)
{
  mpc_tgt_calcModelClass::P_keep_mpc_tgt_calc_T *localP_1;
  localP_1 = mpc_tgt_calc_self_arg->defaultParam;

  // DataTypeConversion: '<S15>/Data Type Conversion' incorporates:
  //   Constant: '<S15>/Constant'

  *rty_accl_out = static_cast<real32_T>(localP_1->Constant_Value);

  // SignalConversion generated from: '<S15>/state_out' incorporates:
  //   Constant: '<S15>/Constant2'

  *rty_state_out = localP_1->Constant2_Value;
}

real32_T rt_powf_snf(real32_T u0, real32_T u1)
{
  real32_T tmp;
  real32_T tmp_0;
  real32_T y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = (rtNaNF);
  } else {
    tmp = std::abs(u0);
    tmp_0 = std::abs(u1);
    if (rtIsInfF(u1)) {
      if (tmp == 1.0F) {
        y = 1.0F;
      } else if (tmp > 1.0F) {
        if (u1 > 0.0F) {
          y = (rtInfF);
        } else {
          y = 0.0F;
        }
      } else if (u1 > 0.0F) {
        y = 0.0F;
      } else {
        y = (rtInfF);
      }
    } else if (tmp_0 == 0.0F) {
      y = 1.0F;
    } else if (tmp_0 == 1.0F) {
      if (u1 > 0.0F) {
        y = u0;
      } else {
        y = 1.0F / u0;
      }
    } else if (u1 == 2.0F) {
      y = u0 * u0;
    } else if (u1 == 0.5F && u0 >= 0.0F) {
      y = std::sqrt(u0);
    } else if (u0 < 0.0F && u1 > std::floor(u1)) {
      y = (rtNaNF);
    } else {
      y = std::pow(u0, u1);
    }
  }

  return y;
}

// Model step function
void mpc_tgt_calcModelClass::step(const t_tgt *arg_tgt, const t_ego *arg_ego,
  int32_T arg_mode, int32_T arg_time_step, t_ego *arg_next_ego)
{
  real_T rtb_Switch_e;
  real32_T rtb_Abs6;
  real32_T rtb_Abs7;
  real32_T rtb_Gain2_h;
  real32_T rtb_Merge;
  real32_T rtb_Merge1;
  real32_T rtb_Merge_b;
  real32_T rtb_Merge_f;
  real32_T rtb_Product2;
  real32_T rtb_Product_b;
  int8_T rtb_Merge1_p;
  boolean_T rtb_RelationalOperator;

  // If: '<S1>/If' incorporates:
  //   Inport: '<Root>/tgt'

  if (arg_tgt->v_max >= 0.0F) {
    // Outputs for IfAction SubSystem: '<S1>/straight_ahead' incorporates:
    //   ActionPort: '<S10>/Action Port'

    // Outputs for IfAction SubSystem: '<S12>/decel' incorporates:
    //   ActionPort: '<S14>/Action Port'

    // If: '<S12>/If' incorporates:
    //   Abs: '<S12>/Abs1'
    //   Abs: '<S12>/Abs2'
    //   Abs: '<S14>/Abs'
    //   Constant: '<S14>/Constant'
    //   DataTypeConversion: '<S12>/Data Type Conversion'
    //   Gain: '<S12>/Gain1'
    //   Gain: '<S14>/Gain'
    //   Gain: '<S14>/Gain1'
    //   Inport: '<Root>/ego'
    //   ManualSwitch: '<S14>/Manual Switch'
    //   Math: '<S12>/Square'
    //   Math: '<S12>/Square1'
    //   Product: '<S12>/Divide'
    //   Product: '<S14>/Divide'
    //   RelationalOperator: '<S14>/Relational Operator'
    //   Sum: '<S12>/Add1'
    //   Sum: '<S12>/Subtract'
    //   Sum: '<S14>/Subtract'
    //   Switch: '<S14>/Switch'

    rtb_Merge_b = arg_ego->v * arg_ego->v - arg_tgt->end_v * arg_tgt->end_v;

    // End of Outputs for SubSystem: '<S12>/decel'
    if (arg_ego->state == 2 || std::abs(rtb_Merge_b) /
        (mpc_tgt_calc_P.Gain1_Gain_o * std::abs(arg_tgt->decel)) + arg_ego->dist
        >= arg_tgt->tgt_dist) {
      // Outputs for IfAction SubSystem: '<S12>/decel' incorporates:
      //   ActionPort: '<S14>/Action Port'

      if (arg_ego->v > arg_tgt->end_v) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_i == 1) {
          rtb_Switch_e = arg_tgt->decel;
        } else {
          // MinMax: '<S14>/Min' incorporates:
          //   Constant: '<S14>/Constant1'
          //   Sum: '<S12>/Subtract1'

          rtb_Switch_e = arg_tgt->tgt_dist - arg_ego->dist;
          if (mpc_tgt_calc_P.Constant1_Value_o > rtb_Switch_e || rtIsNaN
              (rtb_Switch_e)) {
            rtb_Switch_e = mpc_tgt_calc_P.Constant1_Value_o;
          }

          // End of MinMax: '<S14>/Min'
          rtb_Switch_e = std::abs(rtb_Merge_b / (mpc_tgt_calc_P.Gain_Gain_n *
            rtb_Switch_e)) * mpc_tgt_calc_P.Gain1_Gain;
        }
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_nl;
      }

      // Merge: '<S12>/Merge' incorporates:
      //   Abs: '<S14>/Abs'
      //   Constant: '<S14>/Constant'
      //   DataTypeConversion: '<S14>/Data Type Conversion'
      //   Gain: '<S14>/Gain'
      //   Gain: '<S14>/Gain1'
      //   ManualSwitch: '<S14>/Manual Switch'
      //   Product: '<S14>/Divide'
      //   RelationalOperator: '<S14>/Relational Operator'

      rtb_Abs6 = static_cast<real32_T>(rtb_Switch_e);

      // Outport: '<Root>/next_ego' incorporates:
      //   Constant: '<S14>/Constant2'
      //   SignalConversion generated from: '<S14>/state_out'

      arg_next_ego->state = mpc_tgt_calc_P.Constant2_Value_p;

      // End of Outputs for SubSystem: '<S12>/decel'
    } else if (arg_ego->state == 0) {
      // Outputs for IfAction SubSystem: '<S12>/accl' incorporates:
      //   ActionPort: '<S13>/Action Port'

      // RelationalOperator: '<S13>/Relational Operator'
      rtb_RelationalOperator = arg_ego->v < arg_tgt->v_max;

      // Switch: '<S13>/Switch' incorporates:
      //   Constant: '<S13>/Constant'
      //   Switch: '<S13>/Switch1'

      if (rtb_RelationalOperator) {
        // Switch: '<S13>/Switch2' incorporates:
        //   Constant: '<S13>/Constant3'
        //   Math: '<S13>/Square'
        //   Product: '<S13>/Divide'
        //   Product: '<S13>/Product1'
        //   RelationalOperator: '<S13>/Relational Operator1'
        //   Sum: '<S13>/Subtract'

        if (arg_ego->v > arg_tgt->accl_param.limit) {
          rtb_Abs7 = (mpc_tgt_calc_P.Constant3_Value_c - rt_powf_snf(arg_ego->v /
            arg_tgt->v_max, arg_tgt->accl_param.n)) * arg_tgt->accl;
        } else {
          rtb_Abs7 = arg_tgt->accl;
        }

        // End of Switch: '<S13>/Switch2'
        rtb_Switch_e = rtb_Abs7;

        // Outport: '<Root>/next_ego' incorporates:
        //   Constant: '<S13>/Constant1'

        arg_next_ego->state = mpc_tgt_calc_P.Constant1_Value_l;
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_c;

        // Outport: '<Root>/next_ego' incorporates:
        //   Constant: '<S13>/Constant'
        //   Constant: '<S13>/Constant2'

        arg_next_ego->state = mpc_tgt_calc_P.Constant2_Value_gw;
      }

      // End of Switch: '<S13>/Switch'

      // Merge: '<S12>/Merge' incorporates:
      //   DataTypeConversion: '<S13>/Data Type Conversion'

      rtb_Abs6 = static_cast<real32_T>(rtb_Switch_e);

      // End of Outputs for SubSystem: '<S12>/accl'
    } else {
      // Outputs for IfAction SubSystem: '<S12>/keep' incorporates:
      //   ActionPort: '<S15>/Action Port'

      // Outport: '<Root>/next_ego'
      mpc_tgt_calc_keep(&self_keep_h, &rtb_Abs6, &arg_next_ego->state);

      // End of Outputs for SubSystem: '<S12>/keep'
    }

    // End of If: '<S12>/If'

    // Sum: '<S10>/Add' incorporates:
    //   DataTypeConversion: '<Root>/Data Type Conversion2'
    //   Gain: '<S10>/Gain'
    //   Inport: '<Root>/ego'
    //   Inport: '<Root>/time_step'
    //   Product: '<S10>/Product'

    rtb_Abs7 = mpc_tgt_calc_P.Gain_Gain_ca * rtb_Abs6 * static_cast<real32_T>
      (arg_time_step) + arg_ego->v;

    // MinMax: '<S10>/Min'
    if (arg_tgt->v_max < rtb_Abs7 || rtIsNaNF(rtb_Abs7)) {
      rtb_Abs7 = arg_tgt->v_max;
    }

    // End of MinMax: '<S10>/Min'

    // MinMax: '<S10>/Max' incorporates:
    //   Constant: '<S10>/Constant'

    if (!(rtb_Abs7 > mpc_tgt_calc_P.Constant_Value_nex) && !rtIsNaNF
        (mpc_tgt_calc_P.Constant_Value_nex)) {
      rtb_Abs7 = mpc_tgt_calc_P.Constant_Value_nex;
    }

    // End of MinMax: '<S10>/Max'
    // End of Outputs for SubSystem: '<S1>/straight_ahead'
  } else {
    // Outputs for IfAction SubSystem: '<S1>/straight_back' incorporates:
    //   ActionPort: '<S11>/Action Port'

    // Abs: '<S16>/Abs7' incorporates:
    //   Inport: '<Root>/ego'

    rtb_Abs7 = std::abs(arg_ego->dist);

    // Abs: '<S16>/Abs6'
    rtb_Abs6 = std::abs(arg_tgt->tgt_dist);

    // If: '<S16>/If' incorporates:
    //   Abs: '<S16>/Abs1'
    //   Abs: '<S16>/Abs2'
    //   Abs: '<S16>/Abs3'
    //   Abs: '<S16>/Abs4'
    //   DataTypeConversion: '<S16>/Data Type Conversion'
    //   Gain: '<S16>/Gain1'
    //   Inport: '<Root>/ego'
    //   Math: '<S16>/Square'
    //   Math: '<S16>/Square1'
    //   Product: '<S16>/Divide'
    //   Sum: '<S16>/Add1'
    //   Sum: '<S16>/Subtract'

    if (arg_ego->state == 2 || std::abs(std::abs(arg_ego->v * arg_ego->v -
          arg_tgt->end_v * arg_tgt->end_v) / (mpc_tgt_calc_P.Gain1_Gain_n * std::
          abs(arg_tgt->decel))) + rtb_Abs7 >= rtb_Abs6) {
      // Outputs for IfAction SubSystem: '<S16>/decel' incorporates:
      //   ActionPort: '<S18>/Action Port'

      // Switch: '<S18>/Switch' incorporates:
      //   Constant: '<S18>/Constant'
      //   RelationalOperator: '<S18>/Relational Operator'

      if (arg_ego->v < arg_tgt->end_v) {
        // ManualSwitch: '<S18>/Manual Switch' incorporates:
        //   Abs: '<S18>/Abs'
        //   Gain: '<S18>/Gain'
        //   Gain: '<S18>/Gain1'
        //   Math: '<S18>/Square'
        //   Math: '<S18>/Square1'
        //   Product: '<S18>/Divide'
        //   Sum: '<S18>/Subtract'

        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_e == 1) {
          rtb_Switch_e = arg_tgt->decel;
        } else {
          // MinMax: '<S18>/Min' incorporates:
          //   Constant: '<S18>/Constant1'
          //   Sum: '<S16>/Subtract1'

          rtb_Switch_e = rtb_Abs6 - rtb_Abs7;
          if (mpc_tgt_calc_P.Constant1_Value_h > rtb_Switch_e || rtIsNaN
              (rtb_Switch_e)) {
            rtb_Switch_e = mpc_tgt_calc_P.Constant1_Value_h;
          }

          // End of MinMax: '<S18>/Min'
          rtb_Switch_e = std::abs((arg_ego->v * arg_ego->v - arg_tgt->end_v *
            arg_tgt->end_v) / (mpc_tgt_calc_P.Gain_Gain_i * rtb_Switch_e)) *
            mpc_tgt_calc_P.Gain1_Gain_c;
        }

        // End of ManualSwitch: '<S18>/Manual Switch'
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_cx;
      }

      // End of Switch: '<S18>/Switch'

      // Merge: '<S16>/Merge' incorporates:
      //   DataTypeConversion: '<S18>/Data Type Conversion'

      rtb_Abs6 = static_cast<real32_T>(rtb_Switch_e);

      // Outport: '<Root>/next_ego' incorporates:
      //   Constant: '<S18>/Constant2'
      //   SignalConversion generated from: '<S18>/state_out'

      arg_next_ego->state = mpc_tgt_calc_P.Constant2_Value_i;

      // End of Outputs for SubSystem: '<S16>/decel'
    } else if (arg_ego->state == 0) {
      // Outputs for IfAction SubSystem: '<S16>/accl' incorporates:
      //   ActionPort: '<S17>/Action Port'

      // RelationalOperator: '<S17>/Relational Operator'
      rtb_RelationalOperator = arg_ego->v > arg_tgt->v_max;

      // Switch: '<S17>/Switch' incorporates:
      //   Constant: '<S17>/Constant'
      //   Switch: '<S17>/Switch1'

      if (rtb_RelationalOperator) {
        // ManualSwitch: '<S17>/Manual Switch' incorporates:
        //   Constant: '<S17>/Constant3'
        //   Constant: '<S17>/Constant4'
        //   Math: '<S17>/Square'
        //   Product: '<S17>/Divide'
        //   Product: '<S17>/Product1'
        //   Sum: '<S17>/Subtract'

        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_b == 1) {
          rtb_Abs7 = arg_tgt->accl;
        } else {
          rtb_Abs7 = (mpc_tgt_calc_P.Constant3_Value_cn - rt_powf_snf(arg_ego->v
            / arg_tgt->v_max, mpc_tgt_calc_P.Constant4_Value_c)) * arg_tgt->accl;
        }

        // End of ManualSwitch: '<S17>/Manual Switch'
        rtb_Switch_e = rtb_Abs7;

        // Outport: '<Root>/next_ego' incorporates:
        //   Constant: '<S17>/Constant1'

        arg_next_ego->state = mpc_tgt_calc_P.Constant1_Value_k;
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_cf;

        // Outport: '<Root>/next_ego' incorporates:
        //   Constant: '<S17>/Constant'
        //   Constant: '<S17>/Constant2'

        arg_next_ego->state = mpc_tgt_calc_P.Constant2_Value_d;
      }

      // End of Switch: '<S17>/Switch'

      // Merge: '<S16>/Merge' incorporates:
      //   DataTypeConversion: '<S17>/Data Type Conversion'

      rtb_Abs6 = static_cast<real32_T>(rtb_Switch_e);

      // End of Outputs for SubSystem: '<S16>/accl'
    } else {
      // Outputs for IfAction SubSystem: '<S16>/keep' incorporates:
      //   ActionPort: '<S19>/Action Port'

      // Outport: '<Root>/next_ego'
      mpc_tgt_calc_keep(&self_keep_p, &rtb_Abs6, &arg_next_ego->state);

      // End of Outputs for SubSystem: '<S16>/keep'
    }

    // End of If: '<S16>/If'

    // Sum: '<S11>/Add' incorporates:
    //   DataTypeConversion: '<Root>/Data Type Conversion2'
    //   Gain: '<S11>/Gain'
    //   Inport: '<Root>/ego'
    //   Inport: '<Root>/time_step'
    //   Product: '<S11>/Product'

    rtb_Abs7 = mpc_tgt_calc_P.Gain_Gain_k * rtb_Abs6 * static_cast<real32_T>
      (arg_time_step) + arg_ego->v;

    // MinMax: '<S11>/Min'
    if (arg_tgt->v_max > rtb_Abs7 || rtIsNaNF(rtb_Abs7)) {
      rtb_Abs7 = arg_tgt->v_max;
    }

    // End of MinMax: '<S11>/Min'

    // MinMax: '<S11>/Max' incorporates:
    //   Constant: '<S11>/Constant'

    if (!(rtb_Abs7 < mpc_tgt_calc_P.Constant_Value_h) && !rtIsNaNF
        (mpc_tgt_calc_P.Constant_Value_h)) {
      rtb_Abs7 = mpc_tgt_calc_P.Constant_Value_h;
    }

    // End of MinMax: '<S11>/Max'
    // End of Outputs for SubSystem: '<S1>/straight_back'
  }

  // End of If: '<S1>/If'

  // Gain: '<S2>/Gain1' incorporates:
  //   DataTypeConversion: '<Root>/Data Type Conversion2'
  //   Inport: '<Root>/time_step'
  //   Product: '<S2>/Product'

  rtb_Product_b = rtb_Abs7 * static_cast<real32_T>(arg_time_step) *
    mpc_tgt_calc_P.Gain1_Gain_e;

  // Outport: '<Root>/next_ego' incorporates:
  //   BusAssignment: '<Root>/Bus Assignment'
  //   Inport: '<Root>/ego'
  //   Sum: '<S2>/Add1'

  arg_next_ego->dist = arg_ego->dist + rtb_Product_b;

  // If: '<Root>/If' incorporates:
  //   Inport: '<Root>/mode'

  if (arg_mode == 1) {
    // Outputs for IfAction SubSystem: '<Root>/slalom' incorporates:
    //   ActionPort: '<S7>/Action Port'

    // Switch: '<S29>/Switch' incorporates:
    //   Constant: '<S29>/Constant'
    //   Inport: '<Root>/ego'
    //   Product: '<S29>/Product'

    if (arg_ego->sla_param.state != 0) {
      rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_j;
    } else {
      // Sum: '<S29>/Add' incorporates:
      //   DataTypeConversion: '<Root>/Data Type Conversion2'
      //   Inport: '<Root>/time_step'
      //   Sum: '<S30>/Add1'
      //   Switch: '<S29>/Switch1'

      rtb_Merge_b = static_cast<real32_T>(arg_time_step) + static_cast<real32_T>
        (arg_ego->sla_param.counter);

      // Switch: '<S29>/Switch1' incorporates:
      //   Constant: '<S29>/Constant1'
      //   Gain: '<S30>/Gain1'
      //   Math: '<S30>/Exp'
      //   Math: '<S30>/Square'
      //   Product: '<S30>/Divide1'
      //   Product: '<S30>/Divide2'
      //   Product: '<S30>/Product1'
      //   Product: '<S30>/Product2'
      //   RelationalOperator: '<S29>/Relational Operator'
      //   Sum: '<S29>/Add'
      //
      //  About '<S30>/Exp':
      //   Operator: exp

      if (rtb_Merge_b >= arg_ego->sla_param.limit_time_count) {
        rtb_Switch_e = mpc_tgt_calc_P.Constant1_Value;
      } else {
        // Sum: '<S30>/Subtract' incorporates:
        //   Constant: '<S30>/Constant'
        //   Gain: '<S30>/Gain'
        //   Product: '<S30>/Divide'

        rtb_Product2 = rtb_Merge_b * mpc_tgt_calc_P.Gain_Gain_e /
          arg_ego->sla_param.base_time - mpc_tgt_calc_P.Constant_Value_ne;

        // Math: '<S30>/Power' incorporates:
        //   Constant: '<S30>/Constant1'
        //   Sum: '<S30>/Subtract1'

        rtb_Merge = rt_powf_snf(rtb_Product2, arg_ego->sla_param.pow_n -
          mpc_tgt_calc_P.Constant1_Value_m);

        // Product: '<S30>/Product'
        rtb_Product2 *= rtb_Merge;

        // Sum: '<S30>/Add' incorporates:
        //   Constant: '<S30>/Constant4'
        //   Constant: '<S30>/Constant5'
        //   Constant: '<S30>/Constant6'
        //   Product: '<S30>/Divide3'
        //   Sum: '<S30>/Subtract3'

        rtb_Gain2_h = mpc_tgt_calc_P.Constant6_Value / (rtb_Product2 -
          mpc_tgt_calc_P.Constant5_Value) + mpc_tgt_calc_P.Constant4_Value;

        // Sum: '<S30>/Subtract2' incorporates:
        //   Constant: '<S30>/Constant2'

        rtb_Product2 -= mpc_tgt_calc_P.Constant2_Value;
        rtb_Switch_e = mpc_tgt_calc_P.Gain1_Gain_l * arg_ego->sla_param.pow_n *
          rtb_Merge / (rtb_Product2 * rtb_Product2) * std::exp(rtb_Gain2_h) /
          arg_ego->sla_param.base_time;
      }

      rtb_Switch_e *= arg_ego->sla_param.base_alpha;
    }

    // End of Switch: '<S29>/Switch'

    // Outport: '<Root>/next_ego' incorporates:
    //   BusAssignment: '<S7>/Bus Assignment'
    //   Inport: '<Root>/ego'

    arg_next_ego->pivot_state = arg_ego->pivot_state;

    // Switch: '<S28>/Switch' incorporates:
    //   Inport: '<Root>/ego'
    //   RelationalOperator: '<S28>/Relational Operator'

    if (arg_ego->sla_param.counter >= arg_ego->sla_param.limit_time_count) {
      // BusAssignment: '<S7>/Bus Assignment' incorporates:
      //   BusAssignment: '<S6>/Bus Assignment'
      //   Constant: '<S28>/Constant1'
      //   DataTypeConversion: '<S7>/Data Type Conversion1'

      rtb_Product2 = static_cast<real32_T>(mpc_tgt_calc_P.Constant1_Value_d);
    } else {
      // BusAssignment: '<S7>/Bus Assignment' incorporates:
      //   BusAssignment: '<S6>/Bus Assignment'
      //   DataTypeConversion: '<Root>/Data Type Conversion2'
      //   DataTypeConversion: '<S28>/Data Type Conversion'
      //   Gain: '<S28>/Gain'
      //   Inport: '<Root>/time_step'
      //   Product: '<S28>/Product'
      //   Sum: '<S28>/Add'

      rtb_Product2 = static_cast<real32_T>(mpc_tgt_calc_P.Gain_Gain *
        rtb_Switch_e * static_cast<real32_T>(arg_time_step)) + arg_ego->w;
    }

    // End of Switch: '<S28>/Switch'

    // Outport: '<Root>/next_ego' incorporates:
    //   BusAssignment: '<S7>/Bus Assignment'
    //   DataTypeConversion: '<S7>/Data Type Conversion2'

    arg_next_ego->alpha = static_cast<real32_T>(rtb_Switch_e);

    // DataTypeConversion: '<S7>/Data Type Conversion' incorporates:
    //   DataTypeConversion: '<Root>/Data Type Conversion2'
    //   Inport: '<Root>/ego'
    //   Inport: '<Root>/time_step'
    //   Sum: '<S28>/Add1'

    rtb_Merge_b = std::fmod(static_cast<real32_T>(arg_ego->sla_param.counter) +
      static_cast<real32_T>(arg_time_step), 4.2949673E+9F);

    // Outport: '<Root>/next_ego' incorporates:
    //   BusAssignment: '<S7>/Bus Assignment'
    //   DataTypeConversion: '<S7>/Data Type Conversion'

    arg_next_ego->sla_param.counter = rtb_Merge_b < 0.0F ? -static_cast<int32_T>
      (static_cast<uint32_T>(-rtb_Merge_b)) : static_cast<int32_T>
      (static_cast<uint32_T>(rtb_Merge_b));

    // End of Outputs for SubSystem: '<Root>/slalom'
  } else if (arg_mode == 2) {
    // Outputs for IfAction SubSystem: '<Root>/pivot' incorporates:
    //   ActionPort: '<S5>/Action Port'

    // Gain: '<S23>/Gain2' incorporates:
    //   Inport: '<Root>/tgt'

    rtb_Gain2_h = mpc_tgt_calc_P.Gain2_Gain * arg_tgt->alpha;

    // Abs: '<S23>/Abs' incorporates:
    //   Abs: '<S5>/Abs'
    //   Inport: '<Root>/ego'

    rtb_Product2 = std::abs(arg_ego->ang);

    // Abs: '<S23>/Abs3' incorporates:
    //   Abs: '<S5>/Abs1'
    //   Inport: '<Root>/tgt'

    rtb_Merge = std::abs(arg_tgt->tgt_angle);

    // Switch: '<S23>/Switch' incorporates:
    //   Gain: '<S23>/Gain3'
    //   Inport: '<Root>/tgt'

    if (arg_tgt->alpha > mpc_tgt_calc_P.Switch_Threshold) {
      rtb_Merge_b = arg_tgt->w_max;
    } else {
      rtb_Merge_b = mpc_tgt_calc_P.Gain3_Gain * arg_tgt->w_max;
    }

    // End of Switch: '<S23>/Switch'

    // If: '<S23>/If' incorporates:
    //   Abs: '<S23>/Abs'
    //   Abs: '<S23>/Abs1'
    //   Abs: '<S23>/Abs2'
    //   Abs: '<S23>/Abs3'
    //   DataTypeConversion: '<S23>/Data Type Conversion'
    //   Gain: '<S23>/Gain1'
    //   Inport: '<Root>/ego'
    //   Inport: '<Root>/tgt'
    //   Math: '<S23>/Square'
    //   Math: '<S23>/Square1'
    //   Product: '<S23>/Divide'
    //   Sum: '<S23>/Add1'
    //   Sum: '<S23>/Subtract'
    //   Sum: '<S23>/Subtract1'

    if (arg_ego->pivot_state == 2 || std::abs(arg_ego->w * arg_ego->w -
         arg_tgt->end_w * arg_tgt->end_w) / (mpc_tgt_calc_P.Gain1_Gain_m * std::
         abs(rtb_Gain2_h)) + rtb_Product2 >= rtb_Merge) {
      // Outputs for IfAction SubSystem: '<S23>/decel' incorporates:
      //   ActionPort: '<S25>/Action Port'

      mpc_tgt_calc_decel(&self_decel, rtb_Gain2_h, arg_ego->w, arg_tgt->end_w,
                         rtb_Merge - rtb_Product2, &rtb_Merge_b, &rtb_Merge1_p);

      // End of Outputs for SubSystem: '<S23>/decel'
    } else if (arg_ego->pivot_state == 0) {
      // Outputs for IfAction SubSystem: '<S23>/accl' incorporates:
      //   ActionPort: '<S24>/Action Port'

      // Switch: '<S24>/Switch1' incorporates:
      //   Abs: '<S24>/Abs'
      //   Abs: '<S24>/Abs1'
      //   Constant: '<S24>/Constant'
      //   Constant: '<S24>/Constant1'
      //   Constant: '<S24>/Constant2'
      //   RelationalOperator: '<S24>/Relational Operator'
      //   Switch: '<S24>/Switch'

      if (std::abs(arg_ego->w) < std::abs(rtb_Merge_b)) {
        rtb_Merge1_p = mpc_tgt_calc_P.Constant1_Value_p;

        // ManualSwitch: '<S24>/Manual Switch' incorporates:
        //   Constant: '<S24>/Constant1'
        //   Constant: '<S24>/Constant3'
        //   Constant: '<S24>/Constant4'
        //   Math: '<S24>/Square'
        //   Product: '<S24>/Divide'
        //   Product: '<S24>/Product1'
        //   Sum: '<S24>/Subtract'

        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting == 1) {
          rtb_Gain2_h = arg_tgt->alpha;
        } else {
          rtb_Gain2_h = (mpc_tgt_calc_P.Constant3_Value - rt_powf_snf(arg_ego->w
            / rtb_Merge_b, mpc_tgt_calc_P.Constant4_Value_i)) * arg_tgt->alpha;
        }

        // End of ManualSwitch: '<S24>/Manual Switch'
        rtb_Switch_e = rtb_Gain2_h;
      } else {
        rtb_Merge1_p = mpc_tgt_calc_P.Constant2_Value_f;
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value;
      }

      // End of Switch: '<S24>/Switch1'

      // DataTypeConversion: '<S24>/Data Type Conversion'
      rtb_Merge_b = static_cast<real32_T>(rtb_Switch_e);

      // End of Outputs for SubSystem: '<S23>/accl'
    } else {
      // Outputs for IfAction SubSystem: '<S23>/keep' incorporates:
      //   ActionPort: '<S26>/Action Port'

      // DataTypeConversion: '<S26>/Data Type Conversion' incorporates:
      //   Constant: '<S26>/Constant'

      rtb_Merge_b = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_n);

      // SignalConversion generated from: '<S26>/state_out' incorporates:
      //   Constant: '<S26>/Constant2'

      rtb_Merge1_p = mpc_tgt_calc_P.Constant2_Value_g;

      // End of Outputs for SubSystem: '<S23>/keep'
    }

    // End of If: '<S23>/If'

    // Switch: '<S5>/Switch2' incorporates:
    //   BusAssignment: '<S6>/Bus Assignment'
    //   Constant: '<S5>/Constant'
    //   DataTypeConversion: '<Root>/Data Type Conversion2'
    //   Gain: '<S5>/Gain'
    //   Inport: '<Root>/ego'
    //   Inport: '<Root>/time_step'
    //   Product: '<S5>/Product'
    //   RelationalOperator: '<S5>/Relational Operator1'
    //   Sum: '<S5>/Add'

    if (rtb_Product2 < rtb_Merge) {
      rtb_Product2 = mpc_tgt_calc_P.Gain_Gain_d * rtb_Merge_b * static_cast<
        real32_T>(arg_time_step) + arg_ego->w;

      // Outport: '<Root>/next_ego' incorporates:
      //   BusAssignment: '<S5>/Bus Assignment'
      //   BusAssignment: '<S6>/Bus Assignment'
      //   DataTypeConversion: '<Root>/Data Type Conversion2'
      //   Gain: '<S5>/Gain'
      //   Inport: '<Root>/ego'
      //   Inport: '<Root>/time_step'
      //   Product: '<S5>/Product'
      //   Sum: '<S5>/Add'

      arg_next_ego->alpha = rtb_Merge_b;
      arg_next_ego->sla_param.counter = arg_ego->sla_param.counter;
      arg_next_ego->pivot_state = rtb_Merge1_p;
    } else {
      rtb_Product2 = mpc_tgt_calc_P.Constant_Value_pr;

      // Outport: '<Root>/next_ego' incorporates:
      //   BusAssignment: '<S5>/Bus Assignment'
      //   BusAssignment: '<S6>/Bus Assignment'
      //   Constant: '<S5>/Constant'
      //   Constant: '<S5>/Constant1'
      //   Inport: '<Root>/ego'

      arg_next_ego->alpha = mpc_tgt_calc_P.Constant_Value_pr;
      arg_next_ego->sla_param.counter = arg_ego->sla_param.counter;
      arg_next_ego->pivot_state = mpc_tgt_calc_P.Constant1_Value_c;
    }

    // End of Switch: '<S5>/Switch2'
    // End of Outputs for SubSystem: '<Root>/pivot'
  } else if (arg_mode == 4) {
    // Outputs for IfAction SubSystem: '<Root>/slalom2' incorporates:
    //   ActionPort: '<S8>/Action Port'

    // Abs: '<S31>/Abs' incorporates:
    //   Abs: '<S8>/Abs'
    //   Inport: '<Root>/ego'

    rtb_Product2 = std::abs(arg_ego->img_ang);

    // Gain: '<S31>/Gain2' incorporates:
    //   Inport: '<Root>/tgt'

    rtb_Gain2_h = mpc_tgt_calc_P.Gain2_Gain_i * arg_tgt->alpha;

    // Abs: '<S31>/Abs3' incorporates:
    //   Abs: '<S8>/Abs1'
    //   Inport: '<Root>/tgt'

    rtb_Merge_b = std::abs(arg_tgt->tgt_angle);
    rtb_Merge = rtb_Merge_b;

    // If: '<S31>/If' incorporates:
    //   Abs: '<S31>/Abs'
    //   Abs: '<S31>/Abs1'
    //   Abs: '<S31>/Abs2'
    //   Abs: '<S31>/Abs3'
    //   DataTypeConversion: '<S31>/Data Type Conversion'
    //   Gain: '<S31>/Gain'
    //   Gain: '<S31>/Gain1'
    //   Gain: '<S31>/Gain4'
    //   Inport: '<Root>/ego'
    //   Inport: '<Root>/tgt'
    //   Math: '<S31>/Square'
    //   Math: '<S31>/Square1'
    //   Product: '<S31>/Divide'
    //   Sum: '<S31>/Add1'
    //   Sum: '<S31>/Subtract'
    //   Sum: '<S31>/Subtract1'

    if (arg_ego->pivot_state == 2 || std::abs(arg_ego->w * arg_ego->w -
         arg_tgt->end_w * arg_tgt->end_w) / (mpc_tgt_calc_P.Gain1_Gain_h * std::
         abs(rtb_Gain2_h)) + rtb_Product2 >= mpc_tgt_calc_P.Gain_Gain_m *
        rtb_Merge_b || rtb_Product2 >= mpc_tgt_calc_P.Gain4_Gain * rtb_Merge_b)
    {
      // Outputs for IfAction SubSystem: '<S31>/decel' incorporates:
      //   ActionPort: '<S33>/Action Port'

      mpc_tgt_calc_decel(&self_decel_d, rtb_Gain2_h, arg_ego->w, arg_tgt->end_w,
                         rtb_Merge_b - rtb_Product2, &rtb_Merge, &rtb_Merge1_p);

      // End of Outputs for SubSystem: '<S31>/decel'
    } else if (arg_ego->pivot_state == 0) {
      // Outputs for IfAction SubSystem: '<S31>/accl' incorporates:
      //   ActionPort: '<S32>/Action Port'

      // SignalConversion generated from: '<S32>/state_out' incorporates:
      //   Constant: '<S32>/Constant1'

      rtb_Merge1_p = mpc_tgt_calc_P.Constant1_Value_oj;

      // DataTypeConversion: '<S32>/Data Type Conversion'
      rtb_Merge = arg_tgt->alpha;

      // End of Outputs for SubSystem: '<S31>/accl'
    } else {
      // Outputs for IfAction SubSystem: '<S31>/keep' incorporates:
      //   ActionPort: '<S34>/Action Port'

      // DataTypeConversion: '<S34>/Data Type Conversion' incorporates:
      //   Constant: '<S34>/Constant'

      rtb_Merge = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_p);

      // SignalConversion generated from: '<S34>/state_out' incorporates:
      //   Constant: '<S34>/Constant2'

      rtb_Merge1_p = mpc_tgt_calc_P.Constant2_Value_j;

      // End of Outputs for SubSystem: '<S31>/keep'
    }

    // End of If: '<S31>/If'

    // Switch: '<S8>/Switch2' incorporates:
    //   BusAssignment: '<S6>/Bus Assignment'
    //   Constant: '<S8>/Constant'
    //   DataTypeConversion: '<Root>/Data Type Conversion2'
    //   Gain: '<S8>/Gain'
    //   Inport: '<Root>/ego'
    //   Inport: '<Root>/time_step'
    //   Product: '<S8>/Product'
    //   RelationalOperator: '<S8>/Relational Operator1'
    //   Sum: '<S8>/Add'

    if (rtb_Product2 < rtb_Merge_b) {
      rtb_Product2 = mpc_tgt_calc_P.Gain_Gain_c * rtb_Merge * static_cast<
        real32_T>(arg_time_step) + arg_ego->w;

      // Outport: '<Root>/next_ego' incorporates:
      //   BusAssignment: '<S6>/Bus Assignment'
      //   BusAssignment: '<S8>/Bus Assignment'
      //   DataTypeConversion: '<Root>/Data Type Conversion2'
      //   Gain: '<S8>/Gain'
      //   Inport: '<Root>/ego'
      //   Inport: '<Root>/time_step'
      //   Product: '<S8>/Product'
      //   Sum: '<S8>/Add'

      arg_next_ego->alpha = rtb_Merge;
      arg_next_ego->sla_param.counter = arg_ego->sla_param.counter;
      arg_next_ego->pivot_state = rtb_Merge1_p;
    } else {
      rtb_Product2 = mpc_tgt_calc_P.Constant_Value_m;

      // Outport: '<Root>/next_ego' incorporates:
      //   BusAssignment: '<S6>/Bus Assignment'
      //   BusAssignment: '<S8>/Bus Assignment'
      //   Constant: '<S8>/Constant'
      //   Constant: '<S8>/Constant1'
      //   Inport: '<Root>/ego'

      arg_next_ego->alpha = mpc_tgt_calc_P.Constant_Value_m;
      arg_next_ego->sla_param.counter = arg_ego->sla_param.counter;
      arg_next_ego->pivot_state = mpc_tgt_calc_P.Constant1_Value_e;
    }

    // End of Switch: '<S8>/Switch2'
    // End of Outputs for SubSystem: '<Root>/slalom2'
  } else {
    // Outputs for IfAction SubSystem: '<Root>/undefined' incorporates:
    //   ActionPort: '<S9>/Action Port'

    // Outport: '<Root>/next_ego' incorporates:
    //   BusAssignment: '<S9>/Bus Assignment1'
    //   Inport: '<Root>/ego'

    arg_next_ego->sla_param.counter = arg_ego->sla_param.counter;
    arg_next_ego->pivot_state = arg_ego->pivot_state;

    // BusAssignment: '<S9>/Bus Assignment1' incorporates:
    //   BusAssignment: '<S6>/Bus Assignment'
    //   Constant: '<S9>/Constant'

    rtb_Product2 = mpc_tgt_calc_P.Constant_Value_e;

    // Outport: '<Root>/next_ego' incorporates:
    //   BusAssignment: '<S9>/Bus Assignment1'
    //   Constant: '<S9>/Constant'
    //   DataTypeConversion: '<Root>/Data Type Conversion2'
    //   Gain: '<S9>/Gain'
    //   Inport: '<Root>/ego'
    //   Inport: '<Root>/time_step'
    //   Product: '<S9>/Divide'
    //   Sum: '<S9>/Subtract'

    arg_next_ego->alpha = (mpc_tgt_calc_P.Constant_Value_e - arg_ego->w) /
      (mpc_tgt_calc_P.Gain_Gain_b * static_cast<real32_T>(arg_time_step));

    // End of Outputs for SubSystem: '<Root>/undefined'
  }

  // End of If: '<Root>/If'

  // Gain: '<S3>/Gain1' incorporates:
  //   BusAssignment: '<S6>/Bus Assignment'
  //   DataTypeConversion: '<Root>/Data Type Conversion2'
  //   Inport: '<Root>/time_step'
  //   Product: '<S3>/Product'
  //   SignalConversion generated from: '<Root>/Bus Selector1'

  rtb_Merge_b = rtb_Product2 * static_cast<real32_T>(arg_time_step) *
    mpc_tgt_calc_P.Gain1_Gain_g;

  // Outport: '<Root>/next_ego' incorporates:
  //   Inport: '<Root>/ego'
  //   Sum: '<S2>/Add2'
  //   Sum: '<S3>/Add1'

  arg_next_ego->ang = arg_ego->ang + rtb_Merge_b;
  arg_next_ego->img_dist = arg_ego->img_dist + rtb_Product_b;

  // Sum: '<S3>/Add2' incorporates:
  //   Inport: '<Root>/ego'

  rtb_Merge_b += arg_ego->img_ang;

  // Switch: '<S27>/Switch' incorporates:
  //   BusAssignment: '<Root>/Bus Assignment'
  //   Constant: '<S27>/Constant'
  //   Constant: '<S27>/Constant1'
  //   Constant: '<S27>/Constant3'
  //   DataTypeConversion: '<Root>/Data Type Conversion2'
  //   Gain: '<S27>/mm//s 2 m//s'
  //   Inport: '<Root>/ego'
  //   Inport: '<Root>/tgt'
  //   Inport: '<Root>/time_step'
  //   Product: '<S27>/Divide'
  //   Product: '<S27>/Divide1'
  //   Product: '<S27>/Divide2'
  //   Product: '<S27>/Divide3'
  //   Product: '<S27>/Product1'
  //   Sum: '<S27>/Add'
  //   Sum: '<S27>/Add1'

  if (rtb_Abs7 != 0.0F) {
    rtb_Merge_f = (mpc_tgt_calc_P.Constant_Value_g / static_cast<real32_T>
                   (arg_time_step) * arg_ego->slip_point.slip_angle -
                   arg_ego->slip_point.w) / (arg_tgt->slip_gain /
      (mpc_tgt_calc_P.mms2ms_Gain * rtb_Abs7) +
      mpc_tgt_calc_P.Constant1_Value_hj / static_cast<real32_T>(arg_time_step));
  } else {
    rtb_Merge_f = mpc_tgt_calc_P.Constant3_Value_a;
  }

  // End of Switch: '<S27>/Switch'

  // Sum: '<S27>/Add2' incorporates:
  //   BusAssignment: '<Root>/Bus Assignment1'

  rtb_Merge1 = rtb_Merge_b + rtb_Merge_f;

  // Outport: '<Root>/next_ego' incorporates:
  //   BusAssignment: '<Root>/Bus Assignment1'
  //   BusAssignment: '<S27>/Bus Assignment'

  arg_next_ego->img_ang = rtb_Merge_b;
  arg_next_ego->slip_point.slip_angle = rtb_Merge_f;

  // BusAssignment: '<S27>/Bus Assignment' incorporates:
  //   BusAssignment: '<S6>/Bus Assignment'

  rtb_Product_b = rtb_Merge1;

  // Gain: '<S27>/Gain3' incorporates:
  //   BusAssignment: '<Root>/Bus Assignment'
  //   BusAssignment: '<S27>/Bus Assignment'
  //   Product: '<S27>/Product'
  //   Trigonometry: '<S27>/Cos'

  rtb_Merge = rtb_Abs7 * std::cos(rtb_Merge1) * mpc_tgt_calc_P.Gain3_Gain_m;

  // Trigonometry: '<S27>/Cos1' incorporates:
  //   BusAssignment: '<S27>/Bus Assignment'

  rtb_Merge1 = std::sin(rtb_Merge1);

  // Product: '<S27>/Product2' incorporates:
  //   BusAssignment: '<Root>/Bus Assignment'

  rtb_Merge1 *= rtb_Abs7;

  // BusAssignment: '<S27>/Bus Assignment1' incorporates:
  //   BusAssignment: '<Root>/Bus Assignment'
  //   BusAssignment: '<S6>/Bus Assignment'
  //   DataTypeConversion: '<Root>/Data Type Conversion2'
  //   Gain: '<S27>/Gain4'
  //   Inport: '<Root>/ego'
  //   Inport: '<Root>/time_step'
  //   Product: '<S27>/Product3'
  //   Product: '<S27>/Product4'
  //   Sum: '<S27>/Add3'
  //   Sum: '<S27>/Add4'

  rtb_Merge = static_cast<real32_T>(arg_time_step) * rtb_Merge +
    arg_ego->slip_point.x;
  rtb_Gain2_h = mpc_tgt_calc_P.Gain4_Gain_o * rtb_Merge1 * static_cast<real32_T>
    (arg_time_step) + arg_ego->slip_point.y;

  // Sum: '<S6>/Add' incorporates:
  //   BusAssignment: '<Root>/Bus Assignment'
  //   BusAssignment: '<Root>/Bus Assignment1'
  //   DataTypeConversion: '<Root>/Data Type Conversion2'
  //   Gain: '<S6>/Gain1'
  //   Inport: '<Root>/ego'
  //   Inport: '<Root>/time_step'
  //   Product: '<S6>/Product'
  //   Product: '<S6>/Product2'
  //   Trigonometry: '<S6>/Cos'

  rtb_Merge_f = rtb_Abs7 * std::cos(rtb_Merge_b) * mpc_tgt_calc_P.Gain1_Gain_cq *
    static_cast<real32_T>(arg_time_step) + arg_ego->ideal_point.x;

  // Sum: '<S6>/Add1' incorporates:
  //   BusAssignment: '<Root>/Bus Assignment'
  //   BusAssignment: '<Root>/Bus Assignment1'
  //   DataTypeConversion: '<Root>/Data Type Conversion2'
  //   Gain: '<S6>/Gain2'
  //   Inport: '<Root>/ego'
  //   Inport: '<Root>/time_step'
  //   Product: '<S6>/Product1'
  //   Product: '<S6>/Product3'
  //   Trigonometry: '<S6>/Cos1'

  rtb_Merge1 = rtb_Abs7 * std::sin(rtb_Merge_b) * mpc_tgt_calc_P.Gain2_Gain_c *
    static_cast<real32_T>(arg_time_step) + arg_ego->ideal_point.y;

  // Outport: '<Root>/next_ego' incorporates:
  //   BusAssignment: '<S6>/Bus Assignment'
  //   BusAssignment: '<S6>/Bus Assignment1'
  //   Sum: '<S6>/Subtract'
  //   Sum: '<S6>/Subtract1'

  arg_next_ego->ideal_point.x = rtb_Merge_f;
  arg_next_ego->ideal_point.y = rtb_Merge1;
  arg_next_ego->trj_diff.x = rtb_Merge_f - rtb_Merge;
  arg_next_ego->trj_diff.y = rtb_Merge1 - rtb_Gain2_h;

  // Gain: '<S6>/Gain5' incorporates:
  //   BusAssignment: '<Root>/Bus Assignment1'

  rtb_Merge_b *= mpc_tgt_calc_P.Gain5_Gain;

  // Outport: '<Root>/next_ego' incorporates:
  //   BusAssignment: '<S6>/Bus Assignment'
  //   BusAssignment: '<S6>/Bus Assignment1'
  //   Gain: '<S6>/Gain6'
  //   Sum: '<S6>/Subtract2'

  arg_next_ego->trj_diff.theta = rtb_Merge_b - mpc_tgt_calc_P.Gain6_Gain *
    rtb_Product_b;

  // If: '<S4>/If' incorporates:
  //   BusAssignment: '<Root>/Bus Assignment'
  //   BusAssignment: '<Root>/Bus Assignment1'

  if (rtb_Abs6 > 0.0F) {
    // Outputs for IfAction SubSystem: '<S4>/If Action Subsystem' incorporates:
    //   ActionPort: '<S20>/Action Port'

    // Sum: '<S20>/Add' incorporates:
    //   BusAssignment: '<Root>/Bus Assignment'
    //   Constant: '<S20>/Constant'
    //   Inport: '<Root>/ego'

    rtb_Merge1 = arg_ego->cnt_delay_accl_ratio +
      mpc_tgt_calc_P.Constant_Value_hn;

    // Switch: '<S20>/Switch' incorporates:
    //   Constant: '<S20>/Constant1'
    //   Inport: '<Root>/tgt'

    if (arg_tgt->limit_accl_ratio_cnt != 0.0F) {
      rtb_Merge_b = arg_tgt->limit_accl_ratio_cnt;
    } else {
      rtb_Merge_b = mpc_tgt_calc_P.Constant1_Value_i;
    }

    // End of Switch: '<S20>/Switch'

    // Product: '<S20>/Divide'
    rtb_Merge_b = rtb_Merge1 / rtb_Merge_b;

    // Saturate: '<S20>/Saturation'
    if (rtb_Merge_b > mpc_tgt_calc_P.Saturation_UpperSat) {
      rtb_Merge_b = mpc_tgt_calc_P.Saturation_UpperSat;
    } else {
      if (rtb_Merge_b < mpc_tgt_calc_P.Saturation_LowerSat) {
        rtb_Merge_b = mpc_tgt_calc_P.Saturation_LowerSat;
      }
    }

    // End of Saturate: '<S20>/Saturation'

    // Outport: '<Root>/next_ego' incorporates:
    //   BusAssignment: '<Root>/Bus Assignment'
    //   Inport: '<Root>/ego'
    //   Product: '<S20>/Product'
    //   SignalConversion generated from: '<S20>/cnt_delay_accl_ratio'
    //   SignalConversion generated from: '<S20>/Bus Selector'

    arg_next_ego->delay_accl = rtb_Abs6 * rtb_Merge_b;
    arg_next_ego->cnt_delay_accl_ratio = rtb_Merge1;
    arg_next_ego->cnt_delay_decel_ratio = arg_ego->cnt_delay_decel_ratio;

    // End of Outputs for SubSystem: '<S4>/If Action Subsystem'
  } else if (rtb_Abs6 < 0.0F) {
    // Outputs for IfAction SubSystem: '<S4>/If Action Subsystem1' incorporates:
    //   ActionPort: '<S21>/Action Port'

    // Sum: '<S21>/Add' incorporates:
    //   BusAssignment: '<Root>/Bus Assignment'
    //   Constant: '<S21>/Constant'
    //   Inport: '<Root>/ego'

    rtb_Merge1 = arg_ego->cnt_delay_decel_ratio +
      mpc_tgt_calc_P.Constant_Value_my;

    // Switch: '<S21>/Switch' incorporates:
    //   Constant: '<S21>/Constant1'
    //   Inport: '<Root>/tgt'

    if (arg_tgt->limit_decel_ratio_cnt != 0.0F) {
      rtb_Merge_b = arg_tgt->limit_decel_ratio_cnt;
    } else {
      rtb_Merge_b = mpc_tgt_calc_P.Constant1_Value_op;
    }

    // End of Switch: '<S21>/Switch'

    // Product: '<S21>/Divide'
    rtb_Merge_b = rtb_Merge1 / rtb_Merge_b;

    // Saturate: '<S21>/Saturation'
    if (rtb_Merge_b > mpc_tgt_calc_P.Saturation_UpperSat_i) {
      rtb_Merge_b = mpc_tgt_calc_P.Saturation_UpperSat_i;
    } else {
      if (rtb_Merge_b < mpc_tgt_calc_P.Saturation_LowerSat_e) {
        rtb_Merge_b = mpc_tgt_calc_P.Saturation_LowerSat_e;
      }
    }

    // End of Saturate: '<S21>/Saturation'

    // Outport: '<Root>/next_ego' incorporates:
    //   BusAssignment: '<Root>/Bus Assignment'
    //   Inport: '<Root>/ego'
    //   Product: '<S21>/Product'
    //   SignalConversion generated from: '<S21>/cnt_delay_decel_ratio'
    //   SignalConversion generated from: '<S21>/Bus Selector'

    arg_next_ego->delay_accl = rtb_Abs6 * rtb_Merge_b;
    arg_next_ego->cnt_delay_decel_ratio = rtb_Merge1;
    arg_next_ego->cnt_delay_accl_ratio = arg_ego->cnt_delay_accl_ratio;

    // End of Outputs for SubSystem: '<S4>/If Action Subsystem1'
  } else {
    // Outputs for IfAction SubSystem: '<S4>/If Action Subsystem2' incorporates:
    //   ActionPort: '<S22>/Action Port'

    // Outport: '<Root>/next_ego' incorporates:
    //   BusAssignment: '<Root>/Bus Assignment'
    //   Inport: '<Root>/ego'
    //   SignalConversion generated from: '<S22>/Bus Selector'
    //
    arg_next_ego->delay_accl = rtb_Abs6;
    arg_next_ego->cnt_delay_accl_ratio = arg_ego->cnt_delay_accl_ratio;
    arg_next_ego->cnt_delay_decel_ratio = arg_ego->cnt_delay_decel_ratio;

    // End of Outputs for SubSystem: '<S4>/If Action Subsystem2'
  }

  // End of If: '<S4>/If'

  // Outport: '<Root>/next_ego' incorporates:
  //   BusAssignment: '<Root>/Bus Assignment'
  //   BusAssignment: '<Root>/Bus Assignment1'
  //   BusAssignment: '<S6>/Bus Assignment'
  //   BusAssignment: '<S6>/Bus Assignment1'
  //   DataTypeConversion: '<Root>/Data Type Conversion2'
  //   Gain: '<S4>/Gain'
  //   Inport: '<Root>/ego'
  //   Inport: '<Root>/time_step'
  //   Product: '<S4>/Product'

  arg_next_ego->v = rtb_Abs7;
  arg_next_ego->accl = rtb_Abs6;
  arg_next_ego->w = rtb_Product2;
  arg_next_ego->sla_param.base_alpha = arg_ego->sla_param.base_alpha;
  arg_next_ego->sla_param.base_time = arg_ego->sla_param.base_time;
  arg_next_ego->sla_param.limit_time_count = arg_ego->sla_param.limit_time_count;
  arg_next_ego->sla_param.pow_n = arg_ego->sla_param.pow_n;
  arg_next_ego->sla_param.state = arg_ego->sla_param.state;
  arg_next_ego->ideal_point.theta = arg_ego->ideal_point.theta;
  arg_next_ego->ideal_point.v = arg_ego->ideal_point.v;
  arg_next_ego->ideal_point.w = arg_ego->ideal_point.w;
  arg_next_ego->ideal_point.slip_angle = arg_ego->ideal_point.slip_angle;
  arg_next_ego->slip_point.x = rtb_Merge;
  arg_next_ego->slip_point.y = rtb_Gain2_h;
  arg_next_ego->slip_point.theta = rtb_Product_b;
  arg_next_ego->slip_point.v = arg_ego->slip_point.v;
  arg_next_ego->slip_point.w = arg_ego->slip_point.w;
  arg_next_ego->kanayama_point = arg_ego->kanayama_point;
  arg_next_ego->delay_v = mpc_tgt_calc_P.Gain_Gain_kq * arg_ego->v *
    static_cast<real32_T>(arg_time_step);
}

// Model initialize function
void mpc_tgt_calcModelClass::initialize()
{
  // Registration code

  // initialize non-finites
  rt_InitInfAndNaN(sizeof(real_T));
}

// Model terminate function
void mpc_tgt_calcModelClass::terminate()
{
  // (no terminate code required)
}

// Constructor
mpc_tgt_calcModelClass::mpc_tgt_calcModelClass() :
  self_keep_p(),
  self_keep_h(),
  self_decel_d(),
  self_decel(),
  mpc_tgt_calc_M()
{
  self_keep_p.defaultParam = &mpc_tgt_calc_P.keep_p;
  self_keep_h.defaultParam = &mpc_tgt_calc_P.keep_h;
  self_decel_d.defaultParam = &mpc_tgt_calc_P.decel_d;
  self_decel.defaultParam = &mpc_tgt_calc_P.decel;
}

// Destructor
mpc_tgt_calcModelClass::~mpc_tgt_calcModelClass()
{
  // Currently there is no destructor body generated.
}

// Real-Time Model get method
mpc_tgt_calcModelClass::RT_MODEL_mpc_tgt_calc_T * mpc_tgt_calcModelClass::getRTM
  ()
{
  return (&mpc_tgt_calc_M);
}

//
// File trailer for generated code.
//
// [EOF]
//
