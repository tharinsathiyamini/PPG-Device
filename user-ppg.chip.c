// USER PPG Breakout - Task 1
//
// Simulates the optical/tissue path of a reflectance PPG sensor:
// two "photodiode-equivalent" wavelength channels (RED ~660nm, IR ~940nm),
// each with a large non-pulsatile DC term and a small pulsatile AC term
// locked to the cardiac cycle.
//
// The chip does NOT do the LED time-multiplexing itself - that is the AFE's
// job (Task 2). This chip just watches RED_EN / IR_EN (driven by the AFE)
// and puts the correct instantaneous voltage on PD_OUT:
//   RED_EN=1, IR_EN=0  -> RED channel value
//   IR_EN=1, RED_EN=0  -> IR channel value
//   both 0             -> ambient / dark leakage
//
// HR sets the beat-to-beat interval, SpO2 sets the relative AC/DC ratio
// between RED and IR (via the standard ratio-of-ratios relationship
// SpO2 = 110 - 25*R  =>  R = (110-SpO2)/25), and PI scales the absolute
// AC amplitude of the IR channel.
//
// See https://docs.wokwi.com/chips-api/getting-started for the API.

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define TICK_US 200          // internal update tick (200 us -> 5 kHz)
#define DC_RED_V 1.40f       // baseline reflectance DC level, RED channel
#define DC_IR_V  1.80f       // baseline reflectance DC level, IR channel
#define AMBIENT_V 0.05f      // residual ambient / dark-current leakage
#define NOISE_V   0.004f     // small optical/electrical noise (helps show filtering)

typedef struct {
  pin_t red_en;
  pin_t ir_en;
  pin_t pd_out;
  pin_t motion_out;

  uint32_t attr_hr;
  uint32_t attr_spo2;
  uint32_t attr_pi;
  uint32_t attr_motion;

  uint64_t sim_time_us;
  timer_t  timer;
} chip_state_t;

// Simple analytic PPG pulse template: fast systolic upstroke + smaller
// dicrotic-notch hump, normalized so the peak is ~1.0. `phase` is in [0,1).
static float ppg_template(float phase) {
  float systolic = expf(-powf((phase - 0.15f) / 0.07f, 2.0f));
  float dicrotic = 0.35f * expf(-powf((phase - 0.45f) / 0.09f, 2.0f));
  return systolic + dicrotic;
}

static float frand_pm1(void) {
  return ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
}

static void update_output(chip_state_t *chip) {
  float hr   = attr_read_float(chip->attr_hr);     // bpm
  float spo2 = attr_read_float(chip->attr_spo2);   // %
  float pi   = attr_read_float(chip->attr_pi);     // %

  if (hr < 30) hr = 30;
  if (spo2 > 100) spo2 = 100;
  if (spo2 < 50) spo2 = 50;

  float period_s = 60.0f / hr;
  float t_s = chip->sim_time_us / 1e6f;
  float phase = fmodf(t_s, period_s) / period_s;
  float wave = ppg_template(phase); // ~0..1.35

  // Ratio-of-ratios relation (same equation the ESP32 firmware will invert):
  // SpO2 = 110 - 25*R  =>  R = (110 - SpO2) / 25
  float R = (110.0f - spo2) / 25.0f;
  if (R < 0.05f) R = 0.05f;

  float ac_ir_over_dc  = pi / 100.0f;         // PI directly sets IR AC/DC
  float ac_red_over_dc = R * ac_ir_over_dc;   // RED AC/DC derived via R

  float ac_red_v = DC_RED_V * ac_red_over_dc;
  float ac_ir_v  = DC_IR_V  * ac_ir_over_dc;

  int red_on = pin_read(chip->red_en);
  int ir_on  = pin_read(chip->ir_en);

  float v;
  if (red_on && !ir_on) {
    v = DC_RED_V + ac_red_v * wave;
  } else if (ir_on && !red_on) {
    v = DC_IR_V + ac_ir_v * wave;
  } else {
    v = AMBIENT_V; // both off (or both on, which shouldn't happen) -> dark phase
  }

  v += NOISE_V * frand_pm1();

  // ---- motion artifact injection (Bonus b) ----
  // A real accelerometer would flag hand/arm movement; here the "motion"
  // slider stands in for that. Motion couples into the optical path as a
  // large-amplitude disturbance (tremor + slow baseline wander) that is
  // much bigger than the genuine cardiac AC signal - this is what makes
  // motion artifacts so disruptive to naive peak detectors/ratio-of-ratios.
  float motion = attr_read_float(chip->attr_motion); // 0-100
  if (motion > 1.0f) {
    float amp = (motion / 100.0f) * 0.6f; // up to 0.6V, versus ~10-50mV of real AC
    float tremor = amp * sinf(2.0f * 3.14159265f * 3.0f * t_s);   // ~3 Hz hand tremor
    float wander = 0.5f * amp * sinf(2.0f * 3.14159265f * 0.3f * t_s); // slow baseline shift
    v += tremor + wander;
  }
  pin_write(chip->motion_out, motion > 20.0f ? HIGH : LOW);

  if (v < 0) v = 0;
  if (v > 3.3f) v = 3.3f;

  pin_dac_write(chip->pd_out, v);
}

static void chip_timer_tick(void *user_data) {
  chip_state_t *chip = (chip_state_t *)user_data;
  chip->sim_time_us += TICK_US;
  update_output(chip);
}

void chip_init(void) {
  chip_state_t *chip = malloc(sizeof(chip_state_t));

  chip->red_en = pin_init("RED_EN", INPUT_PULLDOWN);
  chip->ir_en  = pin_init("IR_EN", INPUT_PULLDOWN);
  chip->pd_out = pin_init("PD_OUT", ANALOG);
  chip->motion_out = pin_init("MOTION_OUT", OUTPUT);

  chip->attr_hr   = attr_init_float("hr", 75.0f);
  chip->attr_spo2 = attr_init_float("spo2", 98.0f);
  chip->attr_pi   = attr_init_float("pi", 2.0f);
  chip->attr_motion = attr_init_float("motion", 0.0f);

  chip->sim_time_us = 0;

  const timer_config_t timer_config = {
    .callback = chip_timer_tick,
    .user_data = chip,
  };
  chip->timer = timer_init(&timer_config);
  timer_start(chip->timer, TICK_US, true);

  printf("USER PPG chip initialized\n");
}
