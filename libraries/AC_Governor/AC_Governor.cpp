#include "AC_Governor.h"
#include <AP_Math/AP_Math.h>
#include <AP_RPM/AP_RPM.h>

const AP_Param::GroupInfo AC_Governor::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable settings for RSC Setpoint
    // @Description: Allows you to enable (1) or disable (0) the autonomous autorotation capability.
    // @Values: 0:Disabled,1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AC_Governor, _param_enable, 0, AP_PARAM_FLAG_ENABLE),
	
	// @Param: _RATE_FF
    // @DisplayName: Wheel rate control feed forward gain
    // @Description: Wheel rate control feed forward gain.  Desired rate (in radians/sec) is multiplied by this constant and output to output (in the range -1 to +1)
    // @Range: 0.100 2.000
    // @User: Standard

    // @Param: _RATE_P
    // @DisplayName: Wheel rate control P gain
    // @Description: Wheel rate control P gain.  Converts rate error (in radians/sec) to output (in the range -1 to +1)
    // @Range: 0.100 2.000
    // @User: Standard

    // @Param: _RATE_I
    // @DisplayName: Wheel rate control I gain
    // @Description: Wheel rate control I gain.  Corrects long term error between the desired rate (in rad/s) and actual
    // @Range: 0.000 2.000
    // @User: Standard

    // @Param: _RATE_IMAX
    // @DisplayName: Wheel rate control I gain maximum
    // @Description: Wheel rate control I gain maximum.  Constrains the output (range -1 to +1) that the I term will generate
    // @Range: 0.000 1.000
    // @User: Standard

    // @Param: _RATE_D
    // @DisplayName: Wheel rate control D gain
    // @Description: Wheel rate control D gain.  Compensates for short-term change in desired rate vs actual
    // @Range: 0.000 0.400
    // @User: Standard

    // @Param: _RATE_FILT
    // @DisplayName: Wheel rate control filter frequency
    // @Description: Wheel rate control input filter.  Lower values reduce noise but add delay.
    // @Range: 1.000 100.000
    // @Units: Hz
    // @User: Standard
    AP_SUBGROUPINFO(_gov_pid, "_", 2,  AC_Governor, AC_PID),
	
	// @Param: SETPOINT
    // @DisplayName: setpoint for governor
    // @Description: 
    // @Values: 0.000 2.0000
    // @User: Advanced
    AP_GROUPINFO("RPM_SETPOINT", 3, AC_Governor, _param_setpoint, AP_GOV_SETPOINT ),
	
	// @Param: RANGE
    // @DisplayName: range for governor operation
    // @Description: 
    // @Values: 0.000 2.0000
    // @User: Advanced
    AP_GROUPINFO("_RANGE", 4, AC_Governor, _param_governor_range, AP_GOV_RANGE ),
	
	// @Param: SOURCE
    // @DisplayName: range for governor operation
    // @Description: 
    // @Values: 0 1
    // @User: Advanced
    AP_GROUPINFO("_SRC", 5, AC_Governor, _param_rpm_instance, AP_GOV_RPM_SRC ),
	
	
    AP_GROUPEND
};

// Constructor
AC_Governor::AC_Governor()
 
    {
        AP_Param::setup_object_defaults(this, var_info);
    }


void AC_Governor::governor_init()
{
	_governor_engage = false;
	_governor_fault = false;
	_counter=0.0f;
	_governor_fault_count = 0.0f;
	_output = 0.0f;
}	

	
float AC_Governor::get_governor_output()
{	
	 // Get singleton for RPM library
    const AP_RPM *rpm = AP_RPM::get_singleton();

    
    //Get current rpm, checking to ensure no nullptr
    if (rpm != nullptr) {
        //Check requested rpm instance to ensure either 0 or 1.  Always defaults to 0.
        if ((_param_rpm_instance > 1) || (_param_rpm_instance < 0)) {
            _param_rpm_instance = 0;
        }

        //Get RPM value
        uint8_t instance = _param_rpm_instance;
        _current_rpm = rpm->get_rpm(instance);
		
    
if (_current_rpm >= (_param_setpoint - _param_governor_range) && _current_rpm <= (_param_setpoint + _param_governor_range) && !_governor_fault){
	
	AC_PID& rate_pid = _gov_pid;
	
	float dt;
    uint32_t now = AP_HAL::millis();

    if (_last_update_ms == 0) {
        _last_update_ms = now;
        dt = 0.001f;
    } else {
        dt = 1.0e-3* (now - _last_update_ms);
        _last_update_ms = now;
    }
	
   if (now - _last_update_ms >  AP_GOV_TIMEOUT_MS) {
        rate_pid.reset_filter();
        rate_pid.reset_I();
        _limit.lower = false;
        _limit.upper = false;
    }
	
	
	if(_counter <= 400.0f) {
	_desired_rpm =linear_interpolate((_param_setpoint - _param_governor_range), _param_setpoint, _counter, 0.0f, 400.0f ) ;
	_counter +=1.0f; 
	rate_pid.reset_I();
	} else {
	_desired_rpm = _param_setpoint;	
      if (!_governor_engage){
						gcs().send_text(MAV_SEVERITY_INFO, "Governor on");
					}
    _governor_engage = true;	
	}
	// set PID's dt
    rate_pid .set_dt(dt);
    float actual = _current_rpm/_desired_rpm;
    // constrain and set limit flags
	_output = rate_pid.update_all(1.0000f, actual,  (_limit.lower || _limit.upper));
	
   // set limits for next iteration
    _limit.upper = _output >=0.2f;
    _limit.lower =_output <= -0.2f;
 }else{
	 _output = 0.0f;
  }
   gov_fault_detector();  
 }
  return _output;
}

void AC_Governor::gov_fault_detector()
{
	if (_governor_engage){
	if (_current_rpm <= (_param_setpoint - _param_governor_range) || _current_rpm >= (_param_setpoint + _param_governor_range)) {
                _governor_fault_count += 1.0f;
                if (_governor_fault_count > 200.0f) {
                    _governor_fault = true;
					_output = 0.0f;
					if (_governor_engage){
						gcs().send_text(MAV_SEVERITY_INFO, "Governor fault");
						_governor_engage = false;
					}
                }
			}
	 }	
}	
