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
  if (std::isnan(u0) || std::isnan(u1)) {
    y = (rtNaNF);
  } else {
    real32_T tmp;
    real32_T tmp_0;
    tmp = std::abs(u0);
    tmp_0 = std::abs(u1);
    if (std::isinf(u1)) {
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
  int32_T arg_mode, int32_T arg_time_step, t_ego *arg_next_ego, t_dynamics
  *arg_ego1)
{
  t_ego rtb_BusAssignment1_o;
  real_T rtb_Switch_e;
  int32_T rtb_pivot_state;
  real32_T Merge_k;
  real32_T rtb_Abs6;
  real32_T rtb_Abs7;
  real32_T rtb_Gain2_n;
  real32_T rtb_Gain2_p;
  real32_T rtb_Product2_h;
  real32_T rtb_Switch_ju;
  if (arg_tgt->v_max >= 0.0F) {
    rtb_Switch_ju = arg_ego->v * arg_ego->v - arg_tgt->end_v * arg_tgt->end_v;
    if (arg_ego->state == 2.0F || std::abs(rtb_Switch_ju) /
        (mpc_tgt_calc_P.Gain1_Gain_o0 * std::abs(arg_tgt->decel)) +
        arg_ego->dist >= arg_tgt->tgt_dist) {
      if (arg_ego->v > arg_tgt->end_v) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_i == 1) {
          rtb_Switch_e = arg_tgt->decel;
        } else {
          rtb_Switch_e = std::abs(rtb_Switch_ju / (std::fmax
            (mpc_tgt_calc_P.Constant1_Value_o, static_cast<real_T>
             (arg_tgt->tgt_dist - arg_ego->dist)) * mpc_tgt_calc_P.Gain_Gain_n))
            * mpc_tgt_calc_P.Gain1_Gain;
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

    rtb_Abs6 = std::fmax(std::fmin(arg_tgt->v_max, mpc_tgt_calc_P.Gain_Gain_ca *
      Merge_k * static_cast<real32_T>(arg_time_step) + arg_ego->v),
                         mpc_tgt_calc_P.Constant_Value_nex);
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
          rtb_Switch_e = std::abs((arg_ego->v * arg_ego->v - arg_tgt->end_v *
            arg_tgt->end_v) / (std::fmax(mpc_tgt_calc_P.Constant1_Value_h,
            static_cast<real_T>(rtb_Abs6 - rtb_Abs7)) *
                               mpc_tgt_calc_P.Gain_Gain_i)) *
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

    rtb_Abs6 = std::fmin(std::fmax(arg_tgt->v_max, mpc_tgt_calc_P.Gain_Gain_k *
      Merge_k * static_cast<real32_T>(arg_time_step) + arg_ego->v),
                         mpc_tgt_calc_P.Constant_Value_hs);
  }

  rtb_Abs7 = rtb_Abs6 * static_cast<real32_T>(arg_time_step) *
    mpc_tgt_calc_P.Gain1_Gain_e;
  if (arg_mode == 1) {
    if (arg_ego->sla_param.state != 0) {
      rtb_Switch_e = mpc_tgt_calc_P.Constant_Value_j;
    } else {
      rtb_Switch_ju = static_cast<real32_T>(arg_time_step) +
        static_cast<real32_T>(arg_ego->sla_param.counter);
      if (rtb_Switch_ju >= arg_ego->sla_param.limit_time_count) {
        rtb_Switch_e = mpc_tgt_calc_P.Constant1_Value;
      } else {
        rtb_Product2_h = rtb_Switch_ju * mpc_tgt_calc_P.Gain_Gain_e /
          arg_ego->sla_param.base_time - mpc_tgt_calc_P.Constant_Value_ne;
        rtb_Gain2_n = rt_powf_snf(rtb_Product2_h, arg_ego->sla_param.pow_n -
          mpc_tgt_calc_P.Constant1_Value_mn);
        rtb_Product2_h *= rtb_Gain2_n;
        rtb_Gain2_p = mpc_tgt_calc_P.Constant6_Value / (rtb_Product2_h -
          mpc_tgt_calc_P.Constant5_Value) + mpc_tgt_calc_P.Constant4_Value;
        rtb_Product2_h -= mpc_tgt_calc_P.Constant2_Value_e;
        rtb_Switch_e = mpc_tgt_calc_P.Gain1_Gain_l * arg_ego->sla_param.pow_n *
          rtb_Gain2_n / (rtb_Product2_h * rtb_Product2_h) * std::exp(rtb_Gain2_p)
          / arg_ego->sla_param.base_time;
      }

      rtb_Switch_e *= arg_ego->sla_param.base_alpha;
    }

    rtb_BusAssignment1_o = *arg_ego;
    if (arg_ego->sla_param.counter >= arg_ego->sla_param.limit_time_count) {
      rtb_BusAssignment1_o.w = static_cast<real32_T>
        (mpc_tgt_calc_P.Constant1_Value_d);
    } else {
      rtb_BusAssignment1_o.w = static_cast<real32_T>(mpc_tgt_calc_P.Gain_Gain *
        rtb_Switch_e * static_cast<real32_T>(arg_time_step)) + arg_ego->w;
    }

    rtb_BusAssignment1_o.alpha = static_cast<real32_T>(rtb_Switch_e);
    rtb_Switch_ju = std::fmod(static_cast<real32_T>(arg_ego->sla_param.counter)
      + static_cast<real32_T>(arg_time_step), 4.2949673E+9F);
    rtb_BusAssignment1_o.sla_param.counter = rtb_Switch_ju < 0.0F ? -
      static_cast<int32_T>(static_cast<uint32_T>(-rtb_Switch_ju)) : static_cast<
      int32_T>(static_cast<uint32_T>(rtb_Switch_ju));
  } else if (arg_mode == 2) {
    t_ego rtb_BusAssignment_m;
    int32_T rtb_Merge1;
    rtb_Product2_h = std::abs(arg_ego->ang);
    rtb_Gain2_p = mpc_tgt_calc_P.Gain2_Gain * arg_tgt->alpha;
    rtb_Gain2_n = std::abs(arg_tgt->tgt_angle);
    if (arg_tgt->alpha > mpc_tgt_calc_P.Switch_Threshold) {
      rtb_Switch_ju = arg_tgt->w_max;
    } else {
      rtb_Switch_ju = mpc_tgt_calc_P.Gain3_Gain * arg_tgt->w_max;
    }

    if (arg_ego->pivot_state == 2.0F || std::abs(arg_ego->w * arg_ego->w -
         arg_tgt->end_w * arg_tgt->end_w) / (mpc_tgt_calc_P.Gain1_Gain_m * std::
         abs(rtb_Gain2_p)) + rtb_Product2_h >= rtb_Gain2_n) {
      boolean_T rtb_RelationalOperator_a;
      if (rtb_Gain2_p > mpc_tgt_calc_P.Switch2_Threshold) {
        rtb_RelationalOperator_a = arg_ego->w < arg_tgt->end_w;
      } else {
        rtb_RelationalOperator_a = arg_ego->w > arg_tgt->end_w;
      }

      if (rtb_RelationalOperator_a) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting != 1) {
          if (rtb_Gain2_p > mpc_tgt_calc_P.Switch1_Threshold) {
            rtb_Gain2_p = static_cast<real32_T>(std::abs((arg_ego->w *
              arg_ego->w - arg_tgt->end_w * arg_tgt->end_w) / (std::fmax
              (mpc_tgt_calc_P.Constant1_Value_l, static_cast<real_T>(rtb_Gain2_n
              - rtb_Product2_h)) * mpc_tgt_calc_P.Gain_Gain_j)));
          } else {
            rtb_Gain2_p = static_cast<real32_T>(std::abs((arg_ego->w *
              arg_ego->w - arg_tgt->end_w * arg_tgt->end_w) / (std::fmax
              (mpc_tgt_calc_P.Constant1_Value_l, static_cast<real_T>(rtb_Gain2_n
              - rtb_Product2_h)) * mpc_tgt_calc_P.Gain_Gain_j))) *
              mpc_tgt_calc_P.Gain1_Gain_h;
          }
        }
      } else {
        rtb_Gain2_p = mpc_tgt_calc_P.Constant_Value_ph;
      }

      rtb_Merge1 = mpc_tgt_calc_P.Constant2_Value;
    } else if (arg_ego->pivot_state == 0.0F) {
      if (std::abs(arg_ego->w) < std::abs(rtb_Switch_ju)) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_c == 1) {
          rtb_Gain2_p = arg_tgt->alpha;
        } else {
          rtb_Gain2_p = (mpc_tgt_calc_P.Constant3_Value - rt_powf_snf(arg_ego->w
            / rtb_Switch_ju, mpc_tgt_calc_P.Constant4_Value_i)) * arg_tgt->alpha;
        }

        rtb_Switch_e = rtb_Gain2_p;
        rtb_Merge1 = mpc_tgt_calc_P.Constant1_Value_p;
      } else {
        rtb_Switch_e = mpc_tgt_calc_P.Constant_Value;
        rtb_Merge1 = mpc_tgt_calc_P.Constant2_Value_f;
      }

      rtb_Gain2_p = static_cast<real32_T>(rtb_Switch_e);
    } else {
      rtb_Gain2_p = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_n);
      rtb_Merge1 = mpc_tgt_calc_P.Constant2_Value_g;
    }

    rtb_BusAssignment_m = *arg_ego;
    rtb_BusAssignment_m.w = mpc_tgt_calc_P.Gain_Gain_d * rtb_Gain2_p *
      static_cast<real32_T>(arg_time_step) + arg_ego->w;
    rtb_BusAssignment_m.alpha = rtb_Gain2_p;
    rtb_BusAssignment_m.pivot_state = rtb_Merge1;
    rtb_BusAssignment1_o = rtb_BusAssignment_m;
    rtb_BusAssignment1_o.w = mpc_tgt_calc_P.Constant_Value_pr;
    rtb_BusAssignment1_o.alpha = mpc_tgt_calc_P.Constant_Value_pr;
    rtb_BusAssignment1_o.pivot_state = mpc_tgt_calc_P.Constant1_Value_c;
    if (rtb_Product2_h < rtb_Gain2_n) {
      rtb_BusAssignment1_o = rtb_BusAssignment_m;
    }
  } else if (arg_mode == 4) {
    t_ego rtb_BusAssignment_m;
    int32_T rtb_Merge1;
    rtb_Product2_h = std::abs(arg_ego->img_ang);
    rtb_Gain2_n = mpc_tgt_calc_P.Gain2_Gain_i * arg_tgt->alpha;
    rtb_Gain2_p = std::abs(arg_tgt->tgt_angle);
    rtb_Switch_ju = arg_ego->w * arg_ego->w - arg_tgt->end_w * arg_tgt->end_w;
    if (arg_ego->pivot_state == 2.0F || std::abs(rtb_Switch_ju) /
        (mpc_tgt_calc_P.Gain1_Gain_hy * std::abs(rtb_Gain2_n)) + rtb_Product2_h >=
        mpc_tgt_calc_P.Gain_Gain_m * rtb_Gain2_p || rtb_Product2_h >=
        mpc_tgt_calc_P.Gain4_Gain * rtb_Gain2_p) {
      boolean_T rtb_RelationalOperator_a;
      if (rtb_Gain2_n > mpc_tgt_calc_P.Switch2_Threshold_m) {
        rtb_RelationalOperator_a = arg_ego->w < arg_tgt->end_w;
      } else {
        rtb_RelationalOperator_a = arg_ego->w > arg_tgt->end_w;
      }

      if (rtb_RelationalOperator_a) {
        if (mpc_tgt_calc_P.ManualSwitch_CurrentSetting_j != 1) {
          if (rtb_Gain2_n > mpc_tgt_calc_P.Switch1_Threshold_i) {
            rtb_Gain2_n = static_cast<real32_T>(std::abs(rtb_Switch_ju / (std::
              fmax(mpc_tgt_calc_P.Constant1_Value_m, static_cast<real_T>
                   (rtb_Gain2_p - rtb_Product2_h)) * mpc_tgt_calc_P.Gain_Gain_g)));
          } else {
            rtb_Gain2_n = static_cast<real32_T>(std::abs(rtb_Switch_ju / (std::
              fmax(mpc_tgt_calc_P.Constant1_Value_m, static_cast<real_T>
                   (rtb_Gain2_p - rtb_Product2_h)) * mpc_tgt_calc_P.Gain_Gain_g)))
              * mpc_tgt_calc_P.Gain1_Gain_n;
          }
        }
      } else {
        rtb_Gain2_n = mpc_tgt_calc_P.Constant_Value_h;
      }

      rtb_Merge1 = mpc_tgt_calc_P.Constant2_Value_f2;
    } else if (arg_ego->pivot_state == 0.0F && rtb_Product2_h <
               mpc_tgt_calc_P.Gain5_Gain * rtb_Gain2_p) {
      rtb_Gain2_n = arg_tgt->alpha;
      rtb_Merge1 = mpc_tgt_calc_P.Constant1_Value_oj;
    } else {
      rtb_Gain2_n = static_cast<real32_T>(mpc_tgt_calc_P.Constant_Value_p);
      rtb_Merge1 = mpc_tgt_calc_P.Constant2_Value_j;
    }

    rtb_BusAssignment_m = *arg_ego;
    rtb_BusAssignment_m.w = mpc_tgt_calc_P.Gain_Gain_c * rtb_Gain2_n *
      static_cast<real32_T>(arg_time_step) + arg_ego->w;
    rtb_BusAssignment_m.alpha = rtb_Gain2_n;
    rtb_BusAssignment_m.pivot_state = rtb_Merge1;
    rtb_BusAssignment1_o = rtb_BusAssignment_m;
    rtb_BusAssignment1_o.w = mpc_tgt_calc_P.Constant_Value_m;
    rtb_BusAssignment1_o.alpha = mpc_tgt_calc_P.Constant_Value_m;
    rtb_BusAssignment1_o.pivot_state = mpc_tgt_calc_P.Constant1_Value_e;
    if (rtb_Product2_h < rtb_Gain2_p) {
      rtb_BusAssignment1_o = rtb_BusAssignment_m;
    }
  } else {
    rtb_BusAssignment1_o = *arg_ego;
    rtb_BusAssignment1_o.w = mpc_tgt_calc_P.Constant_Value_e;
    rtb_BusAssignment1_o.alpha = (mpc_tgt_calc_P.Constant_Value_e - arg_ego->w) /
      (mpc_tgt_calc_P.Gain_Gain_b * static_cast<real32_T>(arg_time_step));
  }

  rtb_Product2_h = rtb_BusAssignment1_o.w * static_cast<real32_T>(arg_time_step)
    * mpc_tgt_calc_P.Gain1_Gain_g;
  if (arg_mode == 1) {
    rtb_Abs6 = mpc_tgt_calc_P.Gain_Gain_p * static_cast<real32_T>(arg_time_step);
    Merge_k = (mpc_tgt_calc_P.Constant_Value_o / arg_ego1->mass +
               rtb_BusAssignment1_o.w * arg_ego->slip.vy) * rtb_Abs6 +
      arg_ego->slip.vx;
    rtb_Gain2_n = (mpc_tgt_calc_P.Gain3_Gain_d * arg_tgt->slip_gain_K1 *
                   arg_ego->slip.beta / arg_ego1->mass - rtb_BusAssignment1_o.w *
                   arg_ego->slip.vx) * rtb_Abs6 + arg_ego->slip.vy;
    rtb_Gain2_p = std::sqrt(Merge_k * Merge_k + rtb_Gain2_n * rtb_Gain2_n);
    rtb_BusAssignment1_o.slip.beta = (arg_ego->slip.beta / rtb_Abs6 -
      rtb_BusAssignment1_o.w) / (mpc_tgt_calc_P.Constant1_Value_i / rtb_Abs6 +
      arg_tgt->slip_gain_K2 / rtb_Gain2_p);
    rtb_BusAssignment1_o.slip.v = rtb_Gain2_p;
    rtb_Gain2_p *= mpc_tgt_calc_P.Gain1_Gain_o;
    rtb_BusAssignment1_o.dist = arg_ego->dist + rtb_Abs7;
    rtb_BusAssignment1_o.ang = arg_ego->ang + rtb_Product2_h;
    rtb_BusAssignment1_o.img_dist = arg_ego->img_dist + rtb_Abs7;
    rtb_BusAssignment1_o.img_ang = arg_ego->img_ang + rtb_Product2_h;
    rtb_BusAssignment1_o.sla_param.base_alpha = arg_ego->sla_param.base_alpha;
    rtb_BusAssignment1_o.sla_param.base_time = arg_ego->sla_param.base_time;
    rtb_BusAssignment1_o.sla_param.limit_time_count =
      arg_ego->sla_param.limit_time_count;
    rtb_BusAssignment1_o.sla_param.pow_n = arg_ego->sla_param.pow_n;
    rtb_BusAssignment1_o.sla_param.state = arg_ego->sla_param.state;
    rtb_BusAssignment1_o.state = rtb_pivot_state;
    rtb_BusAssignment1_o.ideal_point = arg_ego->ideal_point;
    rtb_BusAssignment1_o.slip_point = arg_ego->slip_point;
    rtb_BusAssignment1_o.kanayama_point = arg_ego->kanayama_point;
    rtb_BusAssignment1_o.trj_diff = arg_ego->trj_diff;
    rtb_BusAssignment1_o.delay_accl = arg_ego->delay_accl;
    rtb_BusAssignment1_o.delay_v = arg_ego->delay_v;
    rtb_BusAssignment1_o.cnt_delay_accl_ratio = arg_ego->cnt_delay_accl_ratio;
    rtb_BusAssignment1_o.cnt_delay_decel_ratio = arg_ego->cnt_delay_decel_ratio;
    rtb_BusAssignment1_o.slip.vx = Merge_k;
    rtb_BusAssignment1_o.slip.vy = rtb_Gain2_n;
    rtb_BusAssignment1_o.slip.accl = arg_ego->slip.accl;
    rtb_BusAssignment1_o.v = rtb_Gain2_p;
    rtb_BusAssignment1_o.accl = (rtb_Gain2_p - arg_ego->v) / rtb_Abs6;
  } else {
    rtb_BusAssignment1_o.v = rtb_Abs6;
    rtb_BusAssignment1_o.accl = Merge_k;
    rtb_BusAssignment1_o.dist = arg_ego->dist + rtb_Abs7;
    rtb_BusAssignment1_o.ang = arg_ego->ang + rtb_Product2_h;
    rtb_BusAssignment1_o.img_dist = arg_ego->img_dist + rtb_Abs7;
    rtb_BusAssignment1_o.img_ang = arg_ego->img_ang + rtb_Product2_h;
    rtb_BusAssignment1_o.sla_param.base_alpha = arg_ego->sla_param.base_alpha;
    rtb_BusAssignment1_o.sla_param.base_time = arg_ego->sla_param.base_time;
    rtb_BusAssignment1_o.sla_param.limit_time_count =
      arg_ego->sla_param.limit_time_count;
    rtb_BusAssignment1_o.sla_param.pow_n = arg_ego->sla_param.pow_n;
    rtb_BusAssignment1_o.sla_param.state = arg_ego->sla_param.state;
    rtb_BusAssignment1_o.state = rtb_pivot_state;
    rtb_BusAssignment1_o.ideal_point = arg_ego->ideal_point;
    rtb_BusAssignment1_o.slip_point = arg_ego->slip_point;
    rtb_BusAssignment1_o.kanayama_point = arg_ego->kanayama_point;
    rtb_BusAssignment1_o.trj_diff = arg_ego->trj_diff;
    rtb_BusAssignment1_o.delay_accl = arg_ego->delay_accl;
    rtb_BusAssignment1_o.delay_v = arg_ego->delay_v;
    rtb_BusAssignment1_o.cnt_delay_accl_ratio = arg_ego->cnt_delay_accl_ratio;
    rtb_BusAssignment1_o.cnt_delay_decel_ratio = arg_ego->cnt_delay_decel_ratio;
    rtb_BusAssignment1_o.slip.beta = mpc_tgt_calc_P.Constant_Value_nu;
    rtb_BusAssignment1_o.slip.vx = mpc_tgt_calc_P.Gain1_Gain_i * rtb_Abs6;
    rtb_BusAssignment1_o.slip.vy = mpc_tgt_calc_P.Constant_Value_nu;
    rtb_BusAssignment1_o.slip.v = mpc_tgt_calc_P.Gain_Gain_h * rtb_Abs6;
    rtb_BusAssignment1_o.slip.accl = arg_ego->slip.accl;
  }

  rtb_Abs7 = arg_ego1->gear_ratio * arg_ego1->km;
  rtb_Gain2_n = mpc_tgt_calc_P.Gain4_Gain_c * rtb_BusAssignment1_o.accl *
    arg_ego1->mass * (mpc_tgt_calc_P.Gain1_Gain_b * arg_ego1->tire) *
    arg_ego1->resist / rtb_Abs7;
  Merge_k = rtb_BusAssignment1_o.alpha * arg_ego1->lm *
    (mpc_tgt_calc_P.Gain1_Gain_it * arg_ego1->tire) * arg_ego1->resist /
    rtb_Abs7 / (mpc_tgt_calc_P.Gain2_Gain_j * arg_ego1->tread);
  rtb_Product2_h = mpc_tgt_calc_P.Gain4_Gain_m * rtb_BusAssignment1_o.v;
  rtb_Abs7 = mpc_tgt_calc_P.Gain5_Gain_c * arg_ego1->tread *
    mpc_tgt_calc_P.Gain_Gain_kc * rtb_BusAssignment1_o.w;
  rtb_Abs6 = mpc_tgt_calc_P.Gain6_Gain * arg_ego1->tire *
    mpc_tgt_calc_P.Gain1_Gain_i5;
  rtb_BusAssignment1_o.ff_duty_l = (rtb_Product2_h - rtb_Abs7) * arg_ego1->ke *
    mpc_tgt_calc_P.Gain2_Gain_d / rtb_Abs6 + (rtb_Gain2_n - Merge_k);
  rtb_BusAssignment1_o.ff_duty_r = (rtb_Product2_h + rtb_Abs7) * arg_ego1->ke *
    mpc_tgt_calc_P.Gain3_Gain_k / rtb_Abs6 + (rtb_Gain2_n + Merge_k);
  *arg_next_ego = rtb_BusAssignment1_o;
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
