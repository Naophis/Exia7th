#include "mpc_tgt_calc.h"
#include "rtwtypes.h"
#include "bus.h"
#include <cmath>
#include "mpc_tgt_calc_private.h"

extern "C" {

#include "rt_nonfinite.h"

}
  void mpc_tgt_calcModelClass::mpc_tgt_calc_keep(self_keep_mpc_tgt_calc_T
  *mpc_tgt_calc_self_arg, real32_T *rty_accl_out, int32_T *rty_state_out)
{
  mpc_tgt_calcModelClass::P_keep_mpc_tgt_calc_T *localP_0;
  localP_0 = mpc_tgt_calc_self_arg->defaultParam;
  *rty_accl_out = static_cast<real32_T>(localP_0->Constant_Value);
  *rty_state_out = localP_0->Constant2_Value;
}

real32_T rt_powf_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = (rtNaNF);
  } else {
    real32_T tmp;
    real32_T tmp_0;
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

void mpc_tgt_calcModelClass::step(const t_tgt *arg_tgt, const t_ego *arg_ego,
  int32_T arg_mode, int32_T arg_time_step, t_ego *arg_next_ego)
{
  real_T rtb_Switch_e;
  int32_T rtb_BusAssignment_o_sla_param_c;
  int32_T rtb_Merge1_n;
  int32_T rtb_pivot_state;
  real32_T Merge_k;
  real32_T rtb_Abs6;
  real32_T rtb_Abs7;
  real32_T rtb_Add1_c;
  real32_T rtb_Add1_e;
  real32_T rtb_Add_d;
  real32_T rtb_Add_nd;
  real32_T rtb_BusAssignment_o_slip_point_;
  real32_T rtb_Gain2_p;
  real32_T rtb_Merge1_o;
  real32_T rtb_Merge_f;
  real32_T rtb_Merge_l;
  real32_T rtb_Merge_p5;
  real32_T rtb_Product_b;
  real32_T rtb_Product_g;
  real32_T rtb_Product_h;
  real32_T rtb_Switch_ju;
  if (arg_tgt->v_max >= 0.0F) {
    rtb_Merge_f = arg_ego->v * arg_ego->v - arg_tgt->end_v * arg_tgt->end_v;
    if (arg_ego->state == 2.0F || std::abs(rtb_Merge_f) /
        (mpc_tgt_calc_P.Gain1_Gain_o0 * std::abs(arg_tgt->decel)) +
        arg_ego->dist >= arg_tgt->tgt_dist) {
      if (arg_ego->v > arg_tgt->end_v) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_i == 1) {
          rtb_Switch_e = arg_tgt->decel;
        } else {
          rtb_Switch_e = arg_tgt->tgt_dist - arg_ego->dist;
          if (mpc_tgt_calc_P.Constant1_Value_o >= rtb_Switch_e || rtIsNaN
              (rtb_Switch_e)) {
            rtb_Switch_e = mpc_tgt_calc_P.Constant1_Value_o;
          }

          rtb_Switch_e = std::abs(rtb_Merge_f / (mpc_tgt_calc_P.Gain_Gain_n *
            rtb_Switch_e)) * mpc_tgt_calc_P.Gain1_Gain;
        }
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_nl;
      }

      Merge_k = static_cast<real32_T>(rtb_Switch_e);
      rtb_pivot_state = mpc_tgt_calc_P.Constant2_Value_p;
    } else if (arg_ego->state == 0.0F) {
      if (arg_ego->v < arg_tgt->v_max) {
        if (arg_ego->v > arg_tgt->accl_param.limit) {
          rtb_Abs7 = (mpc_tgt_calc_P.Constant3_Value_c - rt_powf_snf(arg_ego->v /
            arg_tgt->v_max, arg_tgt->accl_param.n)) * arg_tgt->accl;
        } else {
          rtb_Abs7 = arg_tgt->accl;
        }

        rtb_Switch_e = rtb_Abs7;
        rtb_pivot_state = mpc_tgt_calc_P.Constant1_Value_lj;
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_c;
        rtb_pivot_state = mpc_tgt_calc_P.Constant2_Value_gw;
      }

      Merge_k = static_cast<real32_T>(rtb_Switch_e);
    } else {
      mpc_tgt_calc_keep(&self_keep_h, &Merge_k, &rtb_pivot_state);
    }

    rtb_Add1_e = mpc_tgt_calc_P.Gain_Gain_ca * Merge_k * static_cast<real32_T>
      (arg_time_step) + arg_ego->v;
    if (arg_tgt->v_max <= rtb_Add1_e || rtIsNaNF(rtb_Add1_e)) {
      rtb_Add1_e = arg_tgt->v_max;
    }

    if (!(rtb_Add1_e >= mpc_tgt_calc_P.Constant_Value_nex) && !rtIsNaNF
        (mpc_tgt_calc_P.Constant_Value_nex)) {
      rtb_Add1_e = mpc_tgt_calc_P.Constant_Value_nex;
    }
  } else {
    rtb_Abs7 = std::abs(arg_ego->dist);
    rtb_Abs6 = std::abs(arg_tgt->tgt_dist);
    if (arg_ego->state == 2.0F || std::abs(std::abs(arg_ego->v * arg_ego->v -
          arg_tgt->end_v * arg_tgt->end_v) / (mpc_tgt_calc_P.Gain1_Gain_nz * std::
          abs(arg_tgt->decel))) + rtb_Abs7 >= rtb_Abs6) {
      if (arg_ego->v < arg_tgt->end_v) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_e == 1) {
          rtb_Switch_e = arg_tgt->decel;
        } else {
          rtb_Switch_e = rtb_Abs6 - rtb_Abs7;
          if (mpc_tgt_calc_P.Constant1_Value_h >= rtb_Switch_e || rtIsNaN
              (rtb_Switch_e)) {
            rtb_Switch_e = mpc_tgt_calc_P.Constant1_Value_h;
          }

          rtb_Switch_e = std::abs((arg_ego->v * arg_ego->v - arg_tgt->end_v *
            arg_tgt->end_v) / (mpc_tgt_calc_P.Gain_Gain_i * rtb_Switch_e)) *
            mpc_tgt_calc_P.Gain1_Gain_c;
        }
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_cx;
      }

      Merge_k = static_cast<real32_T>(rtb_Switch_e);
      rtb_pivot_state = mpc_tgt_calc_P.Constant2_Value_i;
    } else if (arg_ego->state == 0.0F) {
      if (arg_ego->v > arg_tgt->v_max) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_b == 1) {
          rtb_Abs7 = arg_tgt->accl;
        } else {
          rtb_Abs7 = (mpc_tgt_calc_P.Constant3_Value_cn - rt_powf_snf(arg_ego->v
            / arg_tgt->v_max, mpc_tgt_calc_P.Constant4_Value_c)) * arg_tgt->accl;
        }

        rtb_Switch_e = rtb_Abs7;
        rtb_pivot_state = mpc_tgt_calc_P.Constant1_Value_k;
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_cf;
        rtb_pivot_state = mpc_tgt_calc_P.Constant2_Value_d;
      }

      Merge_k = static_cast<real32_T>(rtb_Switch_e);
    } else {
      mpc_tgt_calc_keep(&self_keep_p, &Merge_k, &rtb_pivot_state);
    }

    rtb_Add1_e = mpc_tgt_calc_P.Gain_Gain_k * Merge_k * static_cast<real32_T>
      (arg_time_step) + arg_ego->v;
    if (arg_tgt->v_max >= rtb_Add1_e || rtIsNaNF(rtb_Add1_e)) {
      rtb_Add1_e = arg_tgt->v_max;
    }

    if (!(rtb_Add1_e <= mpc_tgt_calc_P.Constant_Value_hs) && !rtIsNaNF
        (mpc_tgt_calc_P.Constant_Value_hs)) {
      rtb_Add1_e = mpc_tgt_calc_P.Constant_Value_hs;
    }
  }

  rtb_Abs6 = rtb_Add1_e * static_cast<real32_T>(arg_time_step) *
    mpc_tgt_calc_P.Gain1_Gain_e;
  rtb_Add1_c = rtb_Add1_e;
  rtb_Abs7 = arg_ego->dist + rtb_Abs6;
  if (arg_mode == 1) {
    if (arg_ego->sla_param.state != 0) {
      rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_j;
    } else {
      rtb_Merge_f = static_cast<real32_T>(arg_time_step) + static_cast<real32_T>
        (arg_ego->sla_param.counter);
      if (rtb_Merge_f >= arg_ego->sla_param.limit_time_count) {
        rtb_Switch_e = mpc_tgt_calc_P.Constant1_Value;
      } else {
        rtb_Gain2_p = rtb_Merge_f * mpc_tgt_calc_P.Gain_Gain_e /
          arg_ego->sla_param.base_time - mpc_tgt_calc_P.Constant_Value_ne;
        rtb_Add1_e = rt_powf_snf(rtb_Gain2_p, arg_ego->sla_param.pow_n -
          mpc_tgt_calc_P.Constant1_Value_mn);
        rtb_Gain2_p *= rtb_Add1_e;
        rtb_Add_d = mpc_tgt_calc_P.Constant6_Value / (rtb_Gain2_p -
          mpc_tgt_calc_P.Constant5_Value) + mpc_tgt_calc_P.Constant4_Value;
        rtb_Gain2_p -= mpc_tgt_calc_P.Constant2_Value_e;
        rtb_Switch_e = mpc_tgt_calc_P.Gain1_Gain_l * arg_ego->sla_param.pow_n *
          rtb_Add1_e / (rtb_Gain2_p * rtb_Gain2_p) * std::exp(rtb_Add_d) /
          arg_ego->sla_param.base_time;
      }

      rtb_Switch_e *= arg_ego->sla_param.base_alpha;
    }

    rtb_Merge1_n = arg_ego->pivot_state;
    if (arg_ego->sla_param.counter >= arg_ego->sla_param.limit_time_count) {
      rtb_Add_d = static_cast<real32_T>(mpc_tgt_calc_P.Constant1_Value_d);
    } else {
      rtb_Add_d = static_cast<real32_T>(mpc_tgt_calc_P.Gain_Gain * rtb_Switch_e *
        static_cast<real32_T>(arg_time_step)) + arg_ego->w;
    }

    rtb_Gain2_p = static_cast<real32_T>(rtb_Switch_e);
    rtb_Merge_f = std::fmod(static_cast<real32_T>(arg_ego->sla_param.counter) +
      static_cast<real32_T>(arg_time_step), 4.2949673E+9F);
    rtb_BusAssignment_o_sla_param_c = rtb_Merge_f < 0.0F ? -static_cast<int32_T>
      (static_cast<uint32_T>(-rtb_Merge_f)) : static_cast<int32_T>
      (static_cast<uint32_T>(rtb_Merge_f));
  } else if (arg_mode == 2) {
    rtb_Add1_e = std::abs(arg_ego->ang);
    rtb_Gain2_p = mpc_tgt_calc_P.Gain2_Gain * arg_tgt->alpha;
    rtb_Add_d = std::abs(arg_tgt->tgt_angle);
    if (arg_tgt->alpha > mpc_tgt_calc_P.Switch_Threshold) {
      rtb_Switch_ju = arg_tgt->w_max;
    } else {
      rtb_Switch_ju = mpc_tgt_calc_P.Gain3_Gain * arg_tgt->w_max;
    }

    if (arg_ego->pivot_state == 2.0F || std::abs(arg_ego->w * arg_ego->w -
         arg_tgt->end_w * arg_tgt->end_w) / (mpc_tgt_calc_P.Gain1_Gain_m * std::
         abs(rtb_Gain2_p)) + rtb_Add1_e >= rtb_Add_d) {
      boolean_T rtb_RelationalOperator_a;
      if (rtb_Gain2_p > mpc_tgt_calc_P.Switch2_Threshold) {
        rtb_RelationalOperator_a = arg_ego->w < arg_tgt->end_w;
      } else {
        rtb_RelationalOperator_a = arg_ego->w > arg_tgt->end_w;
      }

      if (rtb_RelationalOperator_a) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting != 1) {
          if (rtb_Gain2_p > mpc_tgt_calc_P.Switch1_Threshold) {
            rtb_Switch_e = rtb_Add_d - rtb_Add1_e;
            if (mpc_tgt_calc_P.Constant1_Value_l >= rtb_Switch_e || rtIsNaN
                (rtb_Switch_e)) {
              rtb_Switch_e = mpc_tgt_calc_P.Constant1_Value_l;
            }

            rtb_Gain2_p = static_cast<real32_T>(std::abs((arg_ego->w *
              arg_ego->w - arg_tgt->end_w * arg_tgt->end_w) /
              (mpc_tgt_calc_P.Gain_Gain_j * rtb_Switch_e)));
          } else {
            rtb_Switch_e = rtb_Add_d - rtb_Add1_e;
            if (mpc_tgt_calc_P.Constant1_Value_l >= rtb_Switch_e || rtIsNaN
                (rtb_Switch_e)) {
              rtb_Switch_e = mpc_tgt_calc_P.Constant1_Value_l;
            }

            rtb_Gain2_p = static_cast<real32_T>(std::abs((arg_ego->w *
              arg_ego->w - arg_tgt->end_w * arg_tgt->end_w) /
              (mpc_tgt_calc_P.Gain_Gain_j * rtb_Switch_e))) *
              mpc_tgt_calc_P.Gain1_Gain_h;
          }
        }
      } else {
        rtb_Gain2_p = mpc_tgt_calc_P.Constant_Value_ph;
      }

      rtb_Merge1_n = mpc_tgt_calc_P.Constant2_Value;
    } else if (arg_ego->pivot_state == 0.0F) {
      if (std::abs(arg_ego->w) < std::abs(rtb_Switch_ju)) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_c == 1) {
          rtb_Gain2_p = arg_tgt->alpha;
        } else {
          rtb_Gain2_p = (mpc_tgt_calc_P.Constant3_Value - rt_powf_snf(arg_ego->w
            / rtb_Switch_ju, mpc_tgt_calc_P.Constant4_Value_i)) * arg_tgt->alpha;
        }

        rtb_Switch_e = rtb_Gain2_p;
        rtb_Merge1_n = mpc_tgt_calc_P.Constant1_Value_p;
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value;
        rtb_Merge1_n = mpc_tgt_calc_P.Constant2_Value_f;
      }

      rtb_Gain2_p = static_cast<real32_T>(rtb_Switch_e);
    } else {
      rtb_Gain2_p = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_n);
      rtb_Merge1_n = mpc_tgt_calc_P.Constant2_Value_g;
    }

    if (rtb_Add1_e < rtb_Add_d) {
      rtb_Add_d = mpc_tgt_calc_P.Gain_Gain_d * rtb_Gain2_p *
        static_cast<real32_T>(arg_time_step) + arg_ego->w;
      rtb_BusAssignment_o_sla_param_c = arg_ego->sla_param.counter;
    } else {
      rtb_Add_d = mpc_tgt_calc_P.Constant_Value_pr;
      rtb_Gain2_p = mpc_tgt_calc_P.Constant_Value_pr;
      rtb_BusAssignment_o_sla_param_c = arg_ego->sla_param.counter;
      rtb_Merge1_n = mpc_tgt_calc_P.Constant1_Value_c;
    }
  } else if (arg_mode == 4) {
    rtb_Add1_e = std::abs(arg_ego->img_ang);
    rtb_Gain2_p = mpc_tgt_calc_P.Gain2_Gain_i * arg_tgt->alpha;
    rtb_Add_d = std::abs(arg_tgt->tgt_angle);
    rtb_Merge_f = arg_ego->w * arg_ego->w - arg_tgt->end_w * arg_tgt->end_w;
    if (arg_ego->pivot_state == 2.0F || std::abs(rtb_Merge_f) /
        (mpc_tgt_calc_P.Gain1_Gain_hy * std::abs(rtb_Gain2_p)) + rtb_Add1_e >=
        mpc_tgt_calc_P.Gain_Gain_m * rtb_Add_d || rtb_Add1_e >=
        mpc_tgt_calc_P.Gain4_Gain * rtb_Add_d) {
      boolean_T rtb_RelationalOperator_a;
      if (rtb_Gain2_p > mpc_tgt_calc_P.Switch2_Threshold_m) {
        rtb_RelationalOperator_a = arg_ego->w < arg_tgt->end_w;
      } else {
        rtb_RelationalOperator_a = arg_ego->w > arg_tgt->end_w;
      }

      if (rtb_RelationalOperator_a) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_j != 1) {
          if (rtb_Gain2_p > mpc_tgt_calc_P.Switch1_Threshold_i) {
            rtb_Switch_e = rtb_Add_d - rtb_Add1_e;
            if (mpc_tgt_calc_P.Constant1_Value_m >= rtb_Switch_e || rtIsNaN
                (rtb_Switch_e)) {
              rtb_Switch_e = mpc_tgt_calc_P.Constant1_Value_m;
            }

            rtb_Gain2_p = static_cast<real32_T>(std::abs(rtb_Merge_f /
              (mpc_tgt_calc_P.Gain_Gain_g * rtb_Switch_e)));
          } else {
            rtb_Switch_e = rtb_Add_d - rtb_Add1_e;
            if (mpc_tgt_calc_P.Constant1_Value_m >= rtb_Switch_e || rtIsNaN
                (rtb_Switch_e)) {
              rtb_Switch_e = mpc_tgt_calc_P.Constant1_Value_m;
            }

            rtb_Gain2_p = static_cast<real32_T>(std::abs(rtb_Merge_f /
              (mpc_tgt_calc_P.Gain_Gain_g * rtb_Switch_e))) *
              mpc_tgt_calc_P.Gain1_Gain_n;
          }
        }
      } else {
        rtb_Gain2_p = mpc_tgt_calc_P.Constant_Value_h;
      }

      rtb_Merge1_n = mpc_tgt_calc_P.Constant2_Value_f2;
    } else if (arg_ego->pivot_state == 0.0F && rtb_Add1_e <
               mpc_tgt_calc_P.Gain5_Gain * rtb_Add_d) {
      rtb_Gain2_p = arg_tgt->alpha;
      rtb_Merge1_n = mpc_tgt_calc_P.Constant1_Value_oj;
    } else {
      rtb_Gain2_p = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_p);
      rtb_Merge1_n = mpc_tgt_calc_P.Constant2_Value_j;
    }

    if (rtb_Add1_e < rtb_Add_d) {
      rtb_Add_d = mpc_tgt_calc_P.Gain_Gain_c * rtb_Gain2_p *
        static_cast<real32_T>(arg_time_step) + arg_ego->w;
      rtb_BusAssignment_o_sla_param_c = arg_ego->sla_param.counter;
    } else {
      rtb_Add_d = mpc_tgt_calc_P.Constant_Value_m;
      rtb_Gain2_p = mpc_tgt_calc_P.Constant_Value_m;
      rtb_BusAssignment_o_sla_param_c = arg_ego->sla_param.counter;
      rtb_Merge1_n = mpc_tgt_calc_P.Constant1_Value_e;
    }
  } else {
    rtb_BusAssignment_o_sla_param_c = arg_ego->sla_param.counter;
    rtb_Merge1_n = arg_ego->pivot_state;
    rtb_Add_d = mpc_tgt_calc_P.Constant_Value_e;
    rtb_Gain2_p = (mpc_tgt_calc_P.Constant_Value_e - arg_ego->w) /
      (mpc_tgt_calc_P.Gain_Gain_b * static_cast<real32_T>(arg_time_step));
  }

  rtb_Add_nd = rtb_Add_d * static_cast<real32_T>(arg_time_step) *
    mpc_tgt_calc_P.Gain1_Gain_g;
  rtb_Add1_e = arg_ego->ang + rtb_Add_nd;
  rtb_Switch_ju = arg_ego->img_dist + rtb_Abs6;
  rtb_Merge1_o = arg_ego->img_ang + rtb_Add_nd;
  if (rtb_Add1_c != 0.0F) {
    rtb_Product_b = (mpc_tgt_calc_P.Constant_Value_g / static_cast<real32_T>
                     (arg_time_step) * arg_ego->slip_point.slip_angle -
                     arg_ego->slip_point.w) / (arg_tgt->slip_gain /
      (mpc_tgt_calc_P.mms2ms_Gain * rtb_Add1_c) +
      mpc_tgt_calc_P.Constant1_Value_hj / static_cast<real32_T>(arg_time_step));
  } else {
    rtb_Product_b = mpc_tgt_calc_P.Constant3_Value_a;
  }

  rtb_Merge_f = rtb_Merge1_o + rtb_Product_b;
  rtb_BusAssignment_o_slip_point_ = rtb_Merge_f;
  rtb_Product_h = rtb_Add1_c * std::cos(rtb_Merge_f) *
    mpc_tgt_calc_P.Gain3_Gain_m * static_cast<real32_T>(arg_time_step) +
    arg_ego->slip_point.x;
  rtb_Merge_l = rtb_Add1_c * std::sin(rtb_Merge_f) * mpc_tgt_calc_P.Gain4_Gain_o
    * static_cast<real32_T>(arg_time_step) + arg_ego->slip_point.y;
  rtb_Product_g = rtb_Add1_c * std::cos(rtb_Merge1_o) *
    mpc_tgt_calc_P.Gain1_Gain_cq * static_cast<real32_T>(arg_time_step) +
    arg_ego->ideal_point.x;
  rtb_Merge_p5 = rtb_Add1_c * std::sin(rtb_Merge1_o) *
    mpc_tgt_calc_P.Gain2_Gain_c * static_cast<real32_T>(arg_time_step) +
    arg_ego->ideal_point.y;
  if (Merge_k > 0.0F) {
    rtb_Add_nd = arg_ego->cnt_delay_accl_ratio +
      mpc_tgt_calc_P.Constant_Value_hn;
    if (arg_tgt->limit_accl_ratio_cnt != 0.0F) {
      rtb_Merge_f = arg_tgt->limit_accl_ratio_cnt;
    } else {
      rtb_Merge_f = mpc_tgt_calc_P.Constant1_Value_ii;
    }

    rtb_Merge_f = rtb_Add_nd / rtb_Merge_f;
    if (rtb_Merge_f > mpc_tgt_calc_P.Saturation_UpperSat) {
      rtb_Merge_f = mpc_tgt_calc_P.Saturation_UpperSat;
    } else if (rtb_Merge_f < mpc_tgt_calc_P.Saturation_LowerSat) {
      rtb_Merge_f = mpc_tgt_calc_P.Saturation_LowerSat;
    }

    rtb_Merge_f *= Merge_k;
    rtb_Abs6 = arg_ego->cnt_delay_decel_ratio;
  } else if (Merge_k < 0.0F) {
    rtb_Abs6 = arg_ego->cnt_delay_decel_ratio + mpc_tgt_calc_P.Constant_Value_my;
    if (arg_tgt->limit_decel_ratio_cnt != 0.0F) {
      rtb_Merge_f = arg_tgt->limit_decel_ratio_cnt;
    } else {
      rtb_Merge_f = mpc_tgt_calc_P.Constant1_Value_op;
    }

    rtb_Merge_f = rtb_Abs6 / rtb_Merge_f;
    if (rtb_Merge_f > mpc_tgt_calc_P.Saturation_UpperSat_i) {
      rtb_Merge_f = mpc_tgt_calc_P.Saturation_UpperSat_i;
    } else if (rtb_Merge_f < mpc_tgt_calc_P.Saturation_LowerSat_e) {
      rtb_Merge_f = mpc_tgt_calc_P.Saturation_LowerSat_e;
    }

    rtb_Merge_f *= Merge_k;
    rtb_Add_nd = arg_ego->cnt_delay_accl_ratio;
  } else {
    rtb_Merge_f = Merge_k;
    rtb_Add_nd = arg_ego->cnt_delay_accl_ratio;
    rtb_Abs6 = arg_ego->cnt_delay_decel_ratio;
  }

  if (arg_mode == 1) {
    real32_T rtb_Add3;
    real32_T rtb_Gain1_mg;
    Merge_k = mpc_tgt_calc_P.Gain_Gain_p * static_cast<real32_T>(arg_time_step);
    rtb_Add1_c = (mpc_tgt_calc_P.Constant_Value_o / arg_tgt->mass + rtb_Add_d *
                  arg_ego->slip.vy) * Merge_k + arg_ego->slip.vx;
    rtb_Add3 = (mpc_tgt_calc_P.Gain3_Gain_d * arg_tgt->slip_gain_K1 *
                arg_ego->slip.beta / arg_tgt->mass - rtb_Add_d *
                arg_ego->slip.vx) * Merge_k + arg_ego->slip.vy;
    rtb_Gain1_mg = std::sqrt(rtb_Add1_c * rtb_Add1_c + rtb_Add3 * rtb_Add3);
    arg_next_ego->slip.beta = (arg_ego->slip.beta / Merge_k - rtb_Add_d) /
      (mpc_tgt_calc_P.Constant1_Value_i / Merge_k + arg_tgt->slip_gain_K2 /
       rtb_Gain1_mg);
    arg_next_ego->slip.v = rtb_Gain1_mg;
    rtb_Gain1_mg *= mpc_tgt_calc_P.Gain1_Gain_o;
    arg_next_ego->w = rtb_Add_d;
    arg_next_ego->alpha = rtb_Gain2_p;
    arg_next_ego->dist = rtb_Abs7;
    arg_next_ego->ang = rtb_Add1_e;
    arg_next_ego->img_dist = rtb_Switch_ju;
    arg_next_ego->img_ang = rtb_Merge1_o;
    arg_next_ego->sla_param.base_alpha = arg_ego->sla_param.base_alpha;
    arg_next_ego->sla_param.base_time = arg_ego->sla_param.base_time;
    arg_next_ego->sla_param.limit_time_count =
      arg_ego->sla_param.limit_time_count;
    arg_next_ego->sla_param.pow_n = arg_ego->sla_param.pow_n;
    arg_next_ego->sla_param.state = arg_ego->sla_param.state;
    arg_next_ego->sla_param.counter = rtb_BusAssignment_o_sla_param_c;
    arg_next_ego->state = rtb_pivot_state;
    arg_next_ego->pivot_state = rtb_Merge1_n;
    arg_next_ego->ideal_point.x = rtb_Product_g;
    arg_next_ego->ideal_point.y = rtb_Merge_p5;
    arg_next_ego->ideal_point.theta = arg_ego->ideal_point.theta;
    arg_next_ego->ideal_point.v = arg_ego->ideal_point.v;
    arg_next_ego->ideal_point.w = arg_ego->ideal_point.w;
    arg_next_ego->ideal_point.slip_angle = arg_ego->ideal_point.slip_angle;
    arg_next_ego->slip_point.x = rtb_Product_h;
    arg_next_ego->slip_point.y = rtb_Merge_l;
    arg_next_ego->slip_point.theta = rtb_BusAssignment_o_slip_point_;
    arg_next_ego->slip_point.v = arg_ego->slip_point.v;
    arg_next_ego->slip_point.w = arg_ego->slip_point.w;
    arg_next_ego->slip_point.slip_angle = rtb_Product_b;
    arg_next_ego->kanayama_point = arg_ego->kanayama_point;
    arg_next_ego->trj_diff.x = rtb_Product_g - rtb_Product_h;
    arg_next_ego->trj_diff.y = rtb_Merge_p5 - rtb_Merge_l;
    arg_next_ego->trj_diff.theta = mpc_tgt_calc_P.Gain5_Gain_e * rtb_Merge1_o -
      mpc_tgt_calc_P.Gain6_Gain * rtb_BusAssignment_o_slip_point_;
    arg_next_ego->delay_accl = rtb_Merge_f;
    arg_next_ego->delay_v = mpc_tgt_calc_P.Gain_Gain_kq * arg_ego->v *
      static_cast<real32_T>(arg_time_step);
    arg_next_ego->cnt_delay_accl_ratio = rtb_Add_nd;
    arg_next_ego->cnt_delay_decel_ratio = rtb_Abs6;
    arg_next_ego->slip.vx = rtb_Add1_c;
    arg_next_ego->slip.vy = rtb_Add3;
    arg_next_ego->slip.accl = arg_ego->slip.accl;
    arg_next_ego->v = rtb_Gain1_mg;
    arg_next_ego->accl = (rtb_Gain1_mg - arg_ego->v) / Merge_k;
  } else {
    arg_next_ego->v = rtb_Add1_c;
    arg_next_ego->accl = Merge_k;
    arg_next_ego->w = rtb_Add_d;
    arg_next_ego->alpha = rtb_Gain2_p;
    arg_next_ego->dist = rtb_Abs7;
    arg_next_ego->ang = rtb_Add1_e;
    arg_next_ego->img_dist = rtb_Switch_ju;
    arg_next_ego->img_ang = rtb_Merge1_o;
    arg_next_ego->sla_param.base_alpha = arg_ego->sla_param.base_alpha;
    arg_next_ego->sla_param.base_time = arg_ego->sla_param.base_time;
    arg_next_ego->sla_param.limit_time_count =
      arg_ego->sla_param.limit_time_count;
    arg_next_ego->sla_param.pow_n = arg_ego->sla_param.pow_n;
    arg_next_ego->sla_param.state = arg_ego->sla_param.state;
    arg_next_ego->sla_param.counter = rtb_BusAssignment_o_sla_param_c;
    arg_next_ego->state = rtb_pivot_state;
    arg_next_ego->pivot_state = rtb_Merge1_n;
    arg_next_ego->ideal_point.x = rtb_Product_g;
    arg_next_ego->ideal_point.y = rtb_Merge_p5;
    arg_next_ego->ideal_point.theta = arg_ego->ideal_point.theta;
    arg_next_ego->ideal_point.v = arg_ego->ideal_point.v;
    arg_next_ego->ideal_point.w = arg_ego->ideal_point.w;
    arg_next_ego->ideal_point.slip_angle = arg_ego->ideal_point.slip_angle;
    arg_next_ego->slip_point.x = rtb_Product_h;
    arg_next_ego->slip_point.y = rtb_Merge_l;
    arg_next_ego->slip_point.theta = rtb_BusAssignment_o_slip_point_;
    arg_next_ego->slip_point.v = arg_ego->slip_point.v;
    arg_next_ego->slip_point.w = arg_ego->slip_point.w;
    arg_next_ego->slip_point.slip_angle = rtb_Product_b;
    arg_next_ego->kanayama_point = arg_ego->kanayama_point;
    arg_next_ego->trj_diff.x = rtb_Product_g - rtb_Product_h;
    arg_next_ego->trj_diff.y = rtb_Merge_p5 - rtb_Merge_l;
    arg_next_ego->trj_diff.theta = mpc_tgt_calc_P.Gain5_Gain_e * rtb_Merge1_o -
      mpc_tgt_calc_P.Gain6_Gain * rtb_BusAssignment_o_slip_point_;
    arg_next_ego->delay_accl = rtb_Merge_f;
    arg_next_ego->delay_v = mpc_tgt_calc_P.Gain_Gain_kq * arg_ego->v *
      static_cast<real32_T>(arg_time_step);
    arg_next_ego->cnt_delay_accl_ratio = rtb_Add_nd;
    arg_next_ego->cnt_delay_decel_ratio = rtb_Abs6;
    arg_next_ego->slip.beta = mpc_tgt_calc_P.Constant_Value_nu;
    arg_next_ego->slip.vx = mpc_tgt_calc_P.Gain1_Gain_i * rtb_Add1_c;
    arg_next_ego->slip.vy = mpc_tgt_calc_P.Constant_Value_nu;
    arg_next_ego->slip.v = mpc_tgt_calc_P.Gain_Gain_h * rtb_Add1_c;
    arg_next_ego->slip.accl = arg_ego->slip.accl;
  }
}

void mpc_tgt_calcModelClass::initialize()
{
  rt_InitInfAndNaN(sizeof(real_T));
}

void mpc_tgt_calcModelClass::terminate()
{
}

mpc_tgt_calcModelClass::mpc_tgt_calcModelClass() :
  self_keep_p(),
  self_keep_h(),
  mpc_tgt_calc_M()
{
  self_keep_p.defaultParam = &mpc_tgt_calc_P.keep_p;
  self_keep_h.defaultParam = &mpc_tgt_calc_P.keep_h;
}

mpc_tgt_calcModelClass::~mpc_tgt_calcModelClass()
{
}

mpc_tgt_calcModelClass::RT_MODEL_mpc_tgt_calc_T * mpc_tgt_calcModelClass::getRTM
  ()
{
  return (&mpc_tgt_calc_M);
}
