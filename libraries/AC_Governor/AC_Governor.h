#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Motors/AP_MotorsHeli_RSC.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter.h>
#include <AC_PID/AC_PID.h>
#include <AP_Common/AP_Common.h>



#define AP_GOV_FF    0.00f
#define AP_GOV_P     0.5f
#define AP_GOV_I     0.25f
#define AP_GOV_IMAX  1.00f
#define AP_GOV_D     0.018f
#define AP_GOV_FILT  0.00f
#define AP_GOV_DT    0.02f
#define AP_GOV_TIMEOUT_MS 200
#define AP_GOV_SETPOINT  1200.0f
#define AP_GOV_RANGE  200.0f
#define AP_GOV_RPM_SRC  0

class AC_Governor
{
public:

    //Constructor
    AC_Governor();
	

    //--------Functions--------
   bool is_enable(void) { return _param_enable; }	

    // User Settable Parameters
    static const struct AP_Param::GroupInfo var_info[];
	void governor_init();
	float get_governor_output();
	void gov_fault_detector();
	float _output;
	bool _governor_engage;
	AC_PID& get_pid();

private:

    //--------Internal Variables--------
    float _current_rpm;
	float _pid_output;
	float _governor_fault_count;
	bool _governor_fault;
	bool _governor_ramp_up;
	float _counter;
	float _desired_rpm;
	uint32_t _last_update_ms;

    //--------Parameter Values--------
    AP_Int8  _param_enable;
	AC_PID          _gov_pid = AC_PID(AP_GOV_P, AP_GOV_I, AP_GOV_D, AP_GOV_FF, AP_GOV_IMAX, AP_GOV_FILT, AP_GOV_FILT, AP_GOV_FILT, AP_GOV_DT);	
	AP_Float   _param_setpoint;
	AP_Float   _param_governor_range;
	AP_Int8    _param_rpm_instance;

    //--------Internal Flags--------
    // limit flags
    struct AC_Governor_limit {
        bool    lower;  
        bool    upper;  
    } _limit;

    //--------Internal Functions--------
   

    // low pass filter for collective trim
    

    //--------internal variables--------
	
  

};
