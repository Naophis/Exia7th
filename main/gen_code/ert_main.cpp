//
// Sponsored License - for use in support of a program or activity
// sponsored by MathWorks.  Not for government, commercial or other
// non-sponsored organizational use.
//
// File: ert_main.cpp
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
#include <stddef.h>
#include <stdio.h>                // This ert_main.c example uses printf/fflush
#include "mpc_tgt_calc.h"              // Model's header file
#include "rtwtypes.h"

static mpc_tgt_calcModelClass mpc_tgt_calc_Obj;// Instance of model class

// '<Root>/tgt'
static t_tgt arg_tgt = {
  0.0F,                                // v_max
  0.0F,                                // end_v
  0.0F,                                // accl
  0.0F,                                // decel
  0.0F,                                // w_max
  0.0F,                                // end_w
  0.0F,                                // alpha
  0.0F,                                // tgt_dist
  0.0F,                                // tgt_angle
  0,                                   // trajectory_point_size

  {
    0.0F,                              // k_x
    0.0F,                              // k_y
    0.0F                               // k_theta
  },                                   // kanayama_gain

  {
    0.0F,                              // limit
    0.0F                               // n
  },                                   // accl_param
  0.0F,                                // slip_gain
  0.0F,                                // limit_accl_ratio_cnt
  0.0F                                 // limit_decel_ratio_cnt
} ;

// '<Root>/ego'
static t_ego arg_ego = {
  0.0F,                                // v
  0.0F,                                // accl
  0.0F,                                // w
  0.0F,                                // alpha
  0.0F,                                // dist
  0.0F,                                // ang
  0.0F,                                // img_dist
  0.0F,                                // img_ang

  {
    0.0F,                              // base_alpha
    0.0F,                              // base_time
    0.0F,                              // limit_time_count
    0.0F,                              // pow_n
    0,                                 // state
    0                                  // counter
  },                                   // sla_param
  0,                                   // state
  0,                                   // pivot_state

  {
    0.0F,                              // x
    0.0F,                              // y
    0.0F,                              // theta
    0.0F,                              // v
    0.0F,                              // w
    0.0F                               // slip_angle
  },                                   // ideal_point

  {
    0.0F,                              // x
    0.0F,                              // y
    0.0F,                              // theta
    0.0F,                              // v
    0.0F,                              // w
    0.0F                               // slip_angle
  },                                   // slip_point

  {
    0.0F,                              // x
    0.0F,                              // y
    0.0F,                              // theta
    0.0F,                              // v
    0.0F                               // w
  },                                   // kanayama_point

  {
    0.0F,                              // x
    0.0F,                              // y
    0.0F                               // theta
  },                                   // trj_diff
  0.0F,                                // delay_accl
  0.0F,                                // delay_v
  0.0F,                                // cnt_delay_accl_ratio
  0.0F                                 // cnt_delay_decel_ratio
} ;

// '<Root>/mode'
static int32_T arg_mode = 0;

// '<Root>/time_step'
static int32_T arg_time_step = 0;

// '<Root>/next_ego'
static t_ego arg_next_ego;

//
// Associating rt_OneStep with a real-time clock or interrupt service routine
// is what makes the generated code "real-time".  The function rt_OneStep is
// always associated with the base rate of the model.  Subrates are managed
// by the base rate from inside the generated code.  Enabling/disabling
// interrupts and floating point context switches are target specific.  This
// example code indicates where these should take place relative to executing
// the generated code step function.  Overrun behavior should be tailored to
// your application needs.  This example simply sets an error status in the
// real-time model and returns from rt_OneStep.
//
void rt_OneStep(void);
void rt_OneStep(void)
{
  static boolean_T OverrunFlag = false;

  // Disable interrupts here

  // Check for overrun
  if (OverrunFlag) {
    rtmSetErrorStatus(mpc_tgt_calc_Obj.getRTM(), "Overrun");
    return;
  }

  OverrunFlag = true;

  // Save FPU context here (if necessary)
  // Re-enable timer or interrupt here
  // Set model inputs here

  // Step the model
  mpc_tgt_calc_Obj.step(&arg_tgt, &arg_ego, arg_mode, arg_time_step,
                        &arg_next_ego);

  // Get model outputs here

  // Indicate task complete
  OverrunFlag = false;

  // Disable interrupts here
  // Restore FPU context here (if necessary)
  // Enable interrupts here
}

//
// The example "main" function illustrates what is required by your
// application code to initialize, execute, and terminate the generated code.
// Attaching rt_OneStep to a real-time clock is target specific.  This example
// illustrates how you do this relative to initializing the model.
//
int_T main(int_T argc, const char *argv[])
{
  // Unused arguments
  (void)(argc);
  (void)(argv);

  // Initialize model
  mpc_tgt_calc_Obj.initialize();

  // Attach rt_OneStep to a timer or interrupt service routine with
  //  period 0.1 seconds (the model's base sample time) here.  The
  //  call syntax for rt_OneStep is
  //
  //   rt_OneStep();

  printf("Warning: The simulation will run forever. "
         "Generated ERT main won't simulate model step behavior. "
         "To change this behavior select the 'MAT-file logging' option.\n");
  fflush((NULL));
  while (rtmGetErrorStatus(mpc_tgt_calc_Obj.getRTM()) == (NULL)) {
    //  Perform other application tasks here
  }

  // Disable rt_OneStep() here

  // Terminate model
  mpc_tgt_calc_Obj.terminate();
  return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//
