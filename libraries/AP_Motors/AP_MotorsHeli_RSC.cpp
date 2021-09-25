/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>
#include "AP_MotorsHeli_RSC.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_MotorsHeli_RSC::var_info[] = {

    // @Param: SETPOINT
    // @DisplayName: External Motor Governor Setpoint
    // @Description: Throttle (HeliRSC Servo) output in percent to the external motor governor when motor interlock enabled (throttle hold off).
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SETPOINT", 1, AP_MotorsHeli_RSC, _rsc_setpoint, AP_MOTORS_HELI_RSC_SETPOINT),

    // @Param: MODE
    // @DisplayName: Rotor Speed Control Mode
    // @Description: Selects the type of rotor speed control used to determine throttle output to the HeliRSC servo channel when motor interlock is enabled (throttle hold off). RC Passthrough sends the input from the RC Motor Interlock channel as throttle output.  External Gov SetPoint sends the RSC SetPoint parameter value as throttle output.  Throttle Curve uses the 5 point throttle curve to determine throttle output based on the collective output.  Governor is ArduCopter's built-in governor that uses the throttle curve for a feed forward throttle command to determine throttle output.
    // @Values: 1:RC Passthrough, 2:External Gov SetPoint, 3:Throttle Curve, 4:Governor
    // @User: Standard
    AP_GROUPINFO("MODE", 2, AP_MotorsHeli_RSC, _rsc_mode, (int8_t)ROTOR_CONTROL_MODE_SPEED_PASSTHROUGH),

    // @Param: RAMP_TIME
    // @DisplayName: Throttle Ramp Time
    // @Description: Time in seconds for throttle output (HeliRSC servo) to ramp from ground idle (RSC_IDLE) to flight idle throttle setting when motor interlock is enabled (throttle hold off).
    // @Range: 0 60
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("RAMP_TIME", 3, AP_MotorsHeli_RSC, _ramp_time, AP_MOTORS_HELI_RSC_RAMP_TIME),

    // @Param: RUNUP_TIME
    // @DisplayName: Rotor Runup Time
    // @Description: Actual time in seconds for the main rotor to reach full speed after motor interlock is enabled (throttle hold off). Must be at least one second longer than the Throttle Ramp Time that is set with RSC_RAMP_TIME.
    // @Range: 0 60
    // @Units: s
    // @User: Standard
    AP_GROUPINFO("RUNUP_TIME", 4, AP_MotorsHeli_RSC, _runup_time, AP_MOTORS_HELI_RSC_RUNUP_TIME),

    // @Param: CRITICAL
    // @DisplayName: Critical Rotor Speed
    // @Description: Percentage of normal rotor speed where flight is no longer possible. However currently the rotor runup/rundown is estimated using the RSC_RUNUP_TIME parameter.   Estimated rotor speed increases/decreases between 0 (rotor stopped) to 1 (rotor at normal speed) in the RSC_RUNUP_TIME in seconds. This parameter should be set so that the estimated rotor speed goes below critical in approximately 3 seconds.  So if you had a 10 second runup time then set RSC_CRITICAL to 70%.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("CRITICAL", 5, AP_MotorsHeli_RSC, _critical_speed, AP_MOTORS_HELI_RSC_CRITICAL),

    // @Param: IDLE
    // @DisplayName: Throttle Output at Idle
    // @Description: Throttle output (HeliRSC Servo) in percent while armed but motor interlock is disabled (throttle hold on). FOR COMBUSTION ENGINES. Sets the engine ground idle throttle percentage with clutch disengaged. This must be set to zero for electric helicopters under most situations. If the ESC has an autorotation window this can be set to keep the autorotation window open in the ESC. Consult the operating manual for your ESC to set it properly for this purpose
    // @Range: 0 50
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("IDLE", 6, AP_MotorsHeli_RSC, _idle_output, AP_MOTORS_HELI_RSC_IDLE_DEFAULT),

    // @Param: SLEWRATE
    // @DisplayName: Throttle Slew Rate
    // @Description: This controls the maximum rate at which the throttle output (HeliRSC servo) can change, as a percentage per second. A value of 100 means the throttle can change over its full range in one second. A value of zero gives unlimited slew rate.
    // @Range: 0 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("SLEWRATE", 7, AP_MotorsHeli_RSC, _power_slewrate, 0),

    // @Param: THRCRV_0
    // @DisplayName: Throttle Curve at 0% Coll
    // @Description: Sets the throttle output (HeliRSC servo) in percent for the throttle curve at the minimum collective pitch position. The 0 percent collective is defined by H_COL_MIN. Example: if the setup has -2 degree to +10 degree collective pitch setup, this setting would correspond to -2 degree of pitch.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THRCRV_0", 8, AP_MotorsHeli_RSC, _thrcrv[0], AP_MOTORS_HELI_RSC_THRCRV_0_DEFAULT),

    // @Param: THRCRV_25
    // @DisplayName: Throttle Curve at 25% Coll
    // @Description: Sets the throttle output (HeliRSC servo) in percent for the throttle curve at 25% of full collective travel where he 0 percent collective is defined by H_COL_MIN and 100 percent collective is defined by H_COL_MAX.  Example: if the setup has -2 degree to +10 degree collective pitch setup, the total range is 12 degrees. 25% of 12 degrees is 3 degrees, so this setting would correspond to +1 degree of pitch.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THRCRV_25", 9, AP_MotorsHeli_RSC, _thrcrv[1], AP_MOTORS_HELI_RSC_THRCRV_25_DEFAULT),

    // @Param: THRCRV_50
    // @DisplayName: Throttle Curve at 50% Coll
    // @Description: Sets the throttle output (HeliRSC servo) in percent for the throttle curve at 50% of full collective travel where he 0 percent collective is defined by H_COL_MIN and 100 percent collective is defined by H_COL_MAX.  Example: if the setup has -2 degree to +10 degree collective pitch setup, the total range is 12 degrees. 50% of 12 degrees is 6 degrees, so this setting would correspond to +4 degree of pitch.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THRCRV_50", 10, AP_MotorsHeli_RSC, _thrcrv[2], AP_MOTORS_HELI_RSC_THRCRV_50_DEFAULT),

    // @Param: THRCRV_75
    // @DisplayName: Throttle Curve at 75% Coll
    // @Description: Sets the throttle output (HeliRSC servo) in percent for the throttle curve at 75% of full collective travel where he 0 percent collective is defined by H_COL_MIN and 100 percent collective is defined by H_COL_MAX.  Example: if the setup has -2 degree to +10 degree collective pitch setup, the total range is 12 degrees. 75% of 12 degrees is 9 degrees, so this setting would correspond to +7 degree of pitch.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THRCRV_75", 11, AP_MotorsHeli_RSC, _thrcrv[3], AP_MOTORS_HELI_RSC_THRCRV_75_DEFAULT),

    // @Param: THRCRV_100
    // @DisplayName: Throttle Curve at 100% Coll
    // @Description: Sets the throttle output (HeliRSC servo) in percent for the throttle curve at the minimum collective pitch position. The 100 percent collective is defined by H_COL_MAX.  Example: if the setup has -2 degree to +10 degree collective pitch setup, this setting would correspond to +10 degree of pitch.
    // @Range: 0 100
    // @Units: %
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("THRCRV_100", 12, AP_MotorsHeli_RSC, _thrcrv[4], AP_MOTORS_HELI_RSC_THRCRV_100_DEFAULT),

    AP_GROUPEND
};

// init_servo - servo initialization on start-up
void AP_MotorsHeli_RSC::init_servo()
{
    // setup RSC on specified channel by default
    SRV_Channels::set_aux_channel_default(_aux_fn, _default_channel);

    // set servo range 
    SRV_Channels::set_range(SRV_Channels::get_motor_function(_aux_fn), 1000);

}

// set_power_output_range
// TODO: Look at possibly calling this at a slower rate.  Doesn't need to be called every cycle.
void AP_MotorsHeli_RSC::set_throttle_curve()
{
    float thrcrv[5];
    // Ensure user inputs are within parameter limits
    // Scale throttle curve parameters
    for (uint8_t i = 0; i < 5; i++) {
        thrcrv[i] = constrain_float(_thrcrv[i] * 0.01f, 0.0f, 1.0f);
    }
    // Calculate the spline polynomials for the throttle curve
    splinterp5(thrcrv,_thrcrv_poly);
}

// output - update value to send to ESC/Servo
void AP_MotorsHeli_RSC::output(RotorControlState state)
{
    float dt;
    uint64_t now = AP_HAL::micros64();
    float last_control_output = _control_output;

    if (_last_update_us == 0) {
        _last_update_us = now;
        dt = 0.001f;
    } else {
        dt = 1.0e-6f * (now - _last_update_us);
        _last_update_us = now;
    }

    switch (state){
        case ROTOR_CONTROL_STOP:
            // set rotor ramp to decrease speed to zero, this happens instantly inside update_rotor_ramp()
            update_rotor_ramp(0.0f, dt);

            // control output forced to zero
            _control_output = 0.0f;
			
			//turbine start internal flag on
			_starting = true;
            break;

        case ROTOR_CONTROL_IDLE:
            // set rotor ramp to decrease speed to zero
            update_rotor_ramp(0.0f, dt);

           // set rotor control speed to idle speed parameter, this happens instantly and ignore ramping
             if (_turbine_start && _starting == true ) {			
			_control_output += 0.001f;
			
			         if(_control_output >= 1.0f) {
						_control_output = get_idle_output();
						gcs().send_text(MAV_SEVERITY_INFO, "Turbine startup");
				       _starting = false;
				      }
             } else{
			 _control_output = get_idle_output();			 
			 } 	 
            break;

        case ROTOR_CONTROL_ACTIVE:
            // set main rotor ramp to increase to full speed
            update_rotor_ramp(1.0f, dt);

            if ((_control_mode == ROTOR_CONTROL_MODE_SPEED_PASSTHROUGH) || (_control_mode == ROTOR_CONTROL_MODE_SPEED_SETPOINT)) {
                // set control rotor speed to ramp slewed value between idle and desired speed
                _control_output = get_idle_output() + (_rotor_ramp_output * (_desired_speed - get_idle_output()));
            } else if (_control_mode == ROTOR_CONTROL_MODE_OPEN_LOOP_POWER_OUTPUT) {
                // throttle output from throttle curve based on collective position
                float desired_throttle = calculate_desired_throttle(_collective_in);
                _control_output = get_idle_output() + (_rotor_ramp_output * (desired_throttle - get_idle_output()));
            } else if (_control_mode == ROTOR_CONTROL_MODE_CLOSED_LOOP_POWER_OUTPUT) {
               _throttle_output = calculate_desired_throttle(_collective_in);
                _control_output = constrain_float(get_idle_output() + (_rotor_ramp_output * (_throttle_output - get_idle_output() + _governor_output)), get_idle_output(), 1.0f);
            }
            break;
    }

    // update rotor speed run-up estimate
    update_rotor_runup(dt);

    if (_power_slewrate > 0) {
        // implement slew rate for throttle
        float max_delta = dt * _power_slewrate * 0.01f;
        _control_output = constrain_float(_control_output, last_control_output-max_delta, last_control_output+max_delta);
    }

    // output to rsc servo
    write_rsc(_control_output);
}

// update_rotor_ramp - slews rotor output scalar between 0 and 1, outputs float scalar to _rotor_ramp_output
void AP_MotorsHeli_RSC::update_rotor_ramp(float rotor_ramp_input, float dt)
{
    int8_t ramp_time;
    // sanity check ramp time and enable bailout if set
    if (_use_bailout_ramp || _ramp_time <= 0) {
        ramp_time = 1;
    } else {
        ramp_time = _ramp_time;
    }

    // ramp output upwards towards target
    if (_rotor_ramp_output < rotor_ramp_input) {
        // allow control output to jump to estimated speed
        if (_rotor_ramp_output < _rotor_runup_output) {
            _rotor_ramp_output = _rotor_runup_output;
        }
        // ramp up slowly to target
        _rotor_ramp_output += (dt / ramp_time);
        if (_rotor_ramp_output > rotor_ramp_input) {
            _rotor_ramp_output = rotor_ramp_input;
        }
    }else{
        // ramping down happens instantly
        _rotor_ramp_output = rotor_ramp_input;
    }
}

// update_rotor_runup - function to slew rotor runup scalar, outputs float scalar to _rotor_runup_ouptut
void AP_MotorsHeli_RSC::update_rotor_runup(float dt)
{
    // sanity check runup time
    if (_runup_time < _ramp_time) {
        _runup_time = _ramp_time;
    }
    if (_runup_time <= 0 ) {
        _runup_time = 1;
    }

    // ramp speed estimate towards control out
    float runup_increment = dt / _runup_time;
    if (_rotor_runup_output < _rotor_ramp_output) {
        _rotor_runup_output += runup_increment;
        if (_rotor_runup_output > _rotor_ramp_output) {
            _rotor_runup_output = _rotor_ramp_output;
        }
    }else{
        _rotor_runup_output -= runup_increment;
        if (_rotor_runup_output < _rotor_ramp_output) {
            _rotor_runup_output = _rotor_ramp_output;
        }
    }

    // update run-up complete flag

    // if control mode is disabled, then run-up complete always returns true
    if ( _control_mode == ROTOR_CONTROL_MODE_DISABLED ){
        _runup_complete = true;
        return;
    }

    // if rotor ramp and runup are both at full speed, then run-up has been completed
    if (!_runup_complete && (_rotor_ramp_output >= 1.0f) && (_rotor_runup_output >= 1.0f)) {
        _runup_complete = true;
    }
    // if rotor speed is less than critical speed, then run-up is not complete
    // this will prevent the case where the target rotor speed is less than critical speed
    if (_runup_complete && (get_rotor_speed() <= get_critical_speed())) {
        _runup_complete = false;
    }
}

// get_rotor_speed - gets rotor speed either as an estimate, or (ToDO) a measured value
float AP_MotorsHeli_RSC::get_rotor_speed() const
{
    // if no actual measured rotor speed is available, estimate speed based on rotor runup scalar.
    return _rotor_runup_output;
}

// write_rsc - outputs pwm onto output rsc channel
// servo_out parameter is of the range 0 ~ 1
void AP_MotorsHeli_RSC::write_rsc(float servo_out)
{
    if (_control_mode == ROTOR_CONTROL_MODE_DISABLED){
        // do not do servo output to avoid conflicting with other output on the channel
        // ToDo: We should probably use RC_Channel_Aux to avoid this problem
        return;
    } else {
        SRV_Channels::set_output_scaled(_aux_fn, (uint16_t) (servo_out * 1000));
    }
}

    // calculate_desired_throttle - uses throttle curve and collective input to determine throttle setting
float AP_MotorsHeli_RSC::calculate_desired_throttle(float collective_in)
{

    const float inpt = collective_in * 4.0f + 1.0f;
    uint8_t idx = constrain_int16(int8_t(collective_in * 4), 0, 3);
    const float a = inpt - (idx + 1.0f);
    const float b = (idx + 1.0f) - inpt + 1.0f;
    float throttle = _thrcrv_poly[idx][0] * a + _thrcrv_poly[idx][1] * b + _thrcrv_poly[idx][2] * (powf(a,3.0f) - a) / 6.0f + _thrcrv_poly[idx][3] * (powf(b,3.0f) - b) / 6.0f;

    throttle = constrain_float(throttle, 0.0f, 1.0f);
    return throttle;

}
