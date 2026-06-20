/*
   smartknob - Copyright 2022 Scott Bezek

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
#include "smart_knob.h"
#include "mysys.h"
#include "arm_math.h"
#include "motordriver.h"

#define CONSTRAIN(x,lower,upper)    ((x)<(lower)?(lower):((x)>(upper)?(upper):(x)))

/* Detent strength: the knob PID P-gain = strength * DETENT_STRENGTH_SCALE.
   Higher = stiffer detents. Settable at runtime over I2C (register 0xD3). */
#define DETENT_STRENGTH_SCALE 10
#define DEFAULT_DETENT_STRENGTH 200       /* -> P-gain 2000, firm click at the snap */

#define FOC_PID_P (DEFAULT_DETENT_STRENGTH * DETENT_STRENGTH_SCALE)
#define FOC_PID_I 0
#define FOC_PID_D 1     /* velocity damping: high values feel draggy/stiff between detents */
#define FOC_PID_OUTPUT_RAMP 100000
#define FOC_PID_LIMIT 10

#define MIN_DETENT_COUNT 1
#define MAX_DETENT_COUNT 256

float DEAD_ZONE_DETENT_PERCENT = 0.25;     /* free zone around each detent = smoother between clicks */
float DEAD_ZONE_RAD = 4 * PI / 180;

float IDLE_VELOCITY_EWMA_ALPHA = 0.001;
float IDLE_VELOCITY_RAD_PER_SEC = 0.05;
uint32_t IDLE_CORRECTION_DELAY_MILLIS = 500;
float IDLE_CORRECTION_MAX_ANGLE_RAD = 5 * PI / 180;
float IDLE_CORRECTION_RATE_ALPHA = 0.0005;

int32_t current_position = 0;
float latest_sub_position_unit = 0;

// Detent configuration exposed over I2C/RS485.
// num_detents = number of detents per full revolution (width = 2*PI/num_detents).
// detent_bounded = 0 -> continuous (infinite) detents, 1 -> bounded with endstops.
uint16_t num_detents = DEFAULT_DETENT_COUNT;
uint8_t detent_bounded = 0;
uint8_t detent_strength = DEFAULT_DETENT_STRENGTH;

float idle_check_velocity_ewma = 0;
uint32_t last_idle_start = 0;
uint32_t last_publish = 0;

float current_detent_center = 0;

unsigned long timestamp_prev = 0;
float integral_prev = 0;
float error_prev = 0;
float output_prev;

float torque;

float motor_pid_velocity_p = FOC_PID_P;
float motor_pid_velocity_i = FOC_PID_I;
float motor_pid_velocity_d = FOC_PID_D;
float motor_pid_velocity_output_ramp = FOC_PID_OUTPUT_RAMP;
float motor_pid_velocity_limit = FOC_PID_LIMIT;

float smart_knob_input = 0;
float dead_zone_adjustment = 0;
float angle_to_detent_center = 0;
bool out_of_bounds;
uint32_t idle_delay = 0;
float rad_diff = 0;

PB_SmartKnobConfig config = {
    .position = 0,
    .sub_position_unit = 0,
    .position_nonce = 6,
    .min_position = 0,
    .max_position = -1,                              // continuous (unbounded) by default
    .position_width_radians = 2 * PI / DEFAULT_DETENT_COUNT,
    .detent_strength_unit = 2,
    .endstop_strength_unit = 1,
    .snap_point = 0.5,
    .led_hue = 25,
};

void init_smart_knob(void)
{
    current_detent_center = mechanical_rad;
    timestamp_prev = micros();
    integral_prev = 0;
    output_prev = 0;
}

// General detent config: spacing (detents_per_rev) is independent of the number
// of bounded positions, so e.g. an on/off switch can have two positions close
// together (narrow spacing) rather than half a revolution apart.
//   detents_per_rev - detent spacing; each detent spans 2*PI/detents_per_rev.
//   num_positions   - 0: continuous/infinite; N>0: bounded to positions 0..N-1
//                     with hard endstops just beyond the ends.
// Re-seeds the detent center and zeroes the reported position to avoid a jump.
void set_detent_config_ex(uint16_t detents_per_rev, int32_t num_positions)
{
    if (detents_per_rev < MIN_DETENT_COUNT) detents_per_rev = MIN_DETENT_COUNT;
    if (detents_per_rev > MAX_DETENT_COUNT) detents_per_rev = MAX_DETENT_COUNT;

    num_detents = detents_per_rev;
    config.position_width_radians = 2 * PI / (float)detents_per_rev;

    if (num_positions > 0) {
        detent_bounded = 1;
        config.min_position = 0;
        config.max_position = num_positions - 1;
    } else {
        detent_bounded = 0;
        config.min_position = 0;
        config.max_position = -1;   // bounds disabled -> infinite detents
    }

    current_position = 0;
    init_smart_knob();
}

// I2C/RS485 config: detents-per-rev with an optional bounded flag. When bounded,
// the positions span one full revolution (0 .. detents-1).
void set_detent_config(uint16_t detents, uint8_t bounded)
{
    set_detent_config_ex(detents, bounded ? detents : 0);
}

// Set detent strength (0 = free spinning, higher = stiffer). Maps to the knob
// PID P-gain. Applied live; no re-seed needed.
void set_detent_strength(uint8_t strength)
{
    detent_strength = strength;
    motor_pid_velocity_p = (float)strength * DETENT_STRENGTH_SCALE;
}

// ---- Demo presets: cycled by the button to showcase the haptic range. ----
//                spacing positions p_gain limit  color     name
const detent_preset_t demo_presets[] = {
    { 12,  0, 2000, 500, 0x00FF00, "12 detents" },  // continuous 12/rev (default feel) - green
    { 36,  0, 1800, 400, 0x00FFFF, "36 fine"    },  // fine continuous                  - cyan
    {  4,  0, 2300, 700, 0x0000FF, "4 coarse"   },  // chunky continuous                - blue
    { 12, 12, 2200, 600, 0xFF00FF, "12 bounded" },  // 12 positions over the rev + ends - magenta
    { 12,  2, 3600, 900, 0xFFFF00, "on / off"   },  // 2 positions 30 deg apart, strong - yellow
    {  1,  0,    0,   0, 0xFF0000, "free spin"  },  // no detents                       - red
};
const uint8_t demo_preset_count = sizeof(demo_presets) / sizeof(demo_presets[0]);
uint8_t demo_preset_index = 0;

void apply_detent_preset(uint8_t idx)
{
    if (idx >= demo_preset_count) idx = 0;
    demo_preset_index = idx;
    const detent_preset_t *p = &demo_presets[idx];
    set_detent_config_ex(p->detents_per_rev, p->num_positions);
    motor_pid_velocity_p = (float)p->p_gain;
    motor_pid_velocity_limit = (float)p->torque_limit;
    detent_strength = p->p_gain > 2550 ? 255 : (uint8_t)(p->p_gain / DETENT_STRENGTH_SCALE);
}

// Advance to the next demo preset (wraps). Called on a button click.
void next_detent_preset(void)
{
    apply_detent_preset((demo_preset_index + 1) % demo_preset_count);
}

float Ts;
unsigned long timestamp_now;

float knob_pid(float error) 
{
    // calculate the time from the last call
    timestamp_now = micros();
    Ts = (timestamp_now - timestamp_prev) * 1e-6f;
    // quick fix for strange cases (micros overflow)
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

    // u(s) = (P + I/s + Ds)e(s)
    // Discrete implementations
    // proportional part
    // u_p  = P *e(k)
    float proportional = motor_pid_velocity_p * error;
    // Tustin transform of the integral part
    // u_ik = u_ik_1  + I*Ts/2*(ek + ek_1)
    // method uses the antiwindup Foxboro method : https://core.ac.uk/download/pdf/289952713.pdf
    float integral = integral_prev + motor_pid_velocity_i*Ts*0.5f*(error + error_prev);
    // antiwindup - limit the output
    integral = CONSTRAIN(integral, -motor_pid_velocity_limit, motor_pid_velocity_limit);
    // Discrete derivation
    // u_dk = D(ek - ek_1)/Ts
    float derivative = motor_pid_velocity_d*(error - error_prev)/Ts;

    // sum all the components
    float output = proportional + integral + derivative;
    // antiwindup - limit the output variable
    output = CONSTRAIN(output, -motor_pid_velocity_limit, motor_pid_velocity_limit);

    // if output ramp defined
    if(motor_pid_velocity_output_ramp > 0){
        // limit the acceleration by ramping the output
        float output_rate = (output - output_prev)/Ts;
        if (output_rate > motor_pid_velocity_output_ramp)
            output = output_prev + motor_pid_velocity_output_ramp*Ts;
        else if (output_rate < -motor_pid_velocity_output_ramp)
            output = output_prev - motor_pid_velocity_output_ramp*Ts;
    }
    // saving for the next pass
    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;
    return output;
}

void handle_smart_knob(void)
{
    // If we are not moving and we're close to the center (but not exactly there), slowly adjust the centerpoint to match the current position
    idle_check_velocity_ewma = motor_rps * IDLE_VELOCITY_EWMA_ALPHA + idle_check_velocity_ewma * (1 - IDLE_VELOCITY_EWMA_ALPHA);
    if (fabsf(idle_check_velocity_ewma) > IDLE_VELOCITY_RAD_PER_SEC) {
        last_idle_start = 0;
    } else {
        if (last_idle_start == 0) {
            last_idle_start = HAL_GetTick();
        }
    }
    idle_delay = HAL_GetTick() - last_idle_start;
    rad_diff = fabsf(mechanical_rad - current_detent_center);
    if (last_idle_start > 0 && idle_delay > IDLE_CORRECTION_DELAY_MILLIS && rad_diff < IDLE_CORRECTION_MAX_ANGLE_RAD) {
        current_detent_center = mechanical_rad * IDLE_CORRECTION_RATE_ALPHA + current_detent_center * (1 - IDLE_CORRECTION_RATE_ALPHA);
    }

    // Check where we are relative to the current nearest detent; update our position if we've moved far enough to snap to another detent
    angle_to_detent_center = mechanical_rad - current_detent_center;
    #if SK_INVERT_ROTATION
        angle_to_detent_center = -mechanical_rad - current_detent_center;
    #endif

    float snap_point_radians = config.position_width_radians * config.snap_point;
    float bias_radians = config.position_width_radians * config.snap_point_bias;
    float snap_point_radians_decrease = snap_point_radians + (current_position <= 0 ? bias_radians : -bias_radians);
    float snap_point_radians_increase = -snap_point_radians + (current_position >= 0 ? -bias_radians : bias_radians); 

    int32_t num_positions = config.max_position - config.min_position + 1;
    if (angle_to_detent_center > snap_point_radians_decrease && (num_positions <= 0 || current_position > config.min_position)) {
        current_detent_center += config.position_width_radians;
        angle_to_detent_center -= config.position_width_radians;
        current_position--;
    } else if (angle_to_detent_center < snap_point_radians_increase && (num_positions <= 0 || current_position < config.max_position)) {
        current_detent_center -= config.position_width_radians;
        angle_to_detent_center += config.position_width_radians;
        current_position++;
    }

    latest_sub_position_unit = -angle_to_detent_center / config.position_width_radians;

    dead_zone_adjustment = CONSTRAIN(
        angle_to_detent_center,
        fmaxf(-config.position_width_radians*DEAD_ZONE_DETENT_PERCENT, -DEAD_ZONE_RAD),
        fminf(config.position_width_radians*DEAD_ZONE_DETENT_PERCENT, DEAD_ZONE_RAD));

    out_of_bounds = num_positions > 0 && ((angle_to_detent_center > 0 && current_position == config.min_position) || (angle_to_detent_center < 0 && current_position == config.max_position));
    // motor_pid_velocity_limit is set per preset (apply_detent_preset) / via the
    // strength register, not overridden here.


    // Apply motor torque based on our angle to the nearest detent (detent strength, etc is handled by the PID_velocity parameters)
    if (fabsf(motor_rps) > 8) {
        // Don't apply torque if velocity is too high (helps avoid positive feedback loop/runaway)
        MotorDriverSetCurrentReal(0.0f);
    } else {
        smart_knob_input = -angle_to_detent_center + dead_zone_adjustment;
        if (!out_of_bounds && config.detent_positions_count > 0) {
            bool in_detent = false;
            for (uint8_t i = 0; i < config.detent_positions_count; i++) {
                if (config.detent_positions[i] == current_position) {
                    in_detent = true;
                    break;
                }
            }
            if (!in_detent) {
                smart_knob_input = 0;
            }
        }
        torque = knob_pid(smart_knob_input);
        #if SK_INVERT_ROTATION
            torque = -torque;
        #endif
        // Negated: on this hardware positive iq drives the rotor in the same
        // direction as increasing encoder angle, so the raw detent torque was
        // positive feedback (knob ran away). Invert it to a restoring force.
        MotorDriverSetCurrentReal(-torque);
    }    
}