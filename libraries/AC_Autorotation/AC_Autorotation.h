#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Motors/AP_MotorsHeli_RSC.h>
#include <Filter/Filter.h>
#include <Filter/LowPassFilter.h>
#include <AC_PID/AC_P.h>
#include <AP_InertialNav/AP_InertialNav.h>  // Inertial Navigation library

class AC_Autorotation
{
public:

    //Constructor
    AC_Autorotation(AP_InertialNav& inav);

    //--------Functions--------
    void init_hs_controller(void);  // Initialise head speed controller
	void guided_input_safety_check();
    void init_fwd_spd_controller(void);  // Initialise forward speed controller
    bool update_hs_glide_controller(float dt);  // Update head speed controller
    float get_rpm(void) const { return _current_rpm; }  // Function just returns the rpm as last read in this library
    float get_rpm(bool update_counter);  // Function fetches fresh rpm update and continues sensor health monitoring
    void set_target_head_speed(float ths) { _target_head_speed = ths; }  // Sets the normalised target head speed
    void set_col_cutoff_freq(float freq) { _col_cutoff_freq = freq; }  // Sets the collective low pass filter cut off frequency
    int16_t get_hs_set_point(void) { return _param_head_speed_set_point; }
    float get_col_entry_freq(void) { return _param_col_entry_cutoff_freq; }
    float get_col_glide_freq(void) { return _param_col_glide_cutoff_freq; }
	float get_col_cushion_freq(void) { return _param_col_cushion_cutoff_freq; }
    float get_bail_time(void) { return _param_bail_time; }
    float get_last_collective() const { return _collective_out; }
    bool is_enable(void) { return _param_enable; }
    void Log_Write_Autorotation(void) const;
    void update_forward_speed_controller(void);  // Update foward speed controller
    void set_desired_fwd_speed(void) { _vel_target = _param_target_speed; } // Overloaded: Set desired speed for forward controller to parameter value
    void set_desired_fwd_speed(float speed) { _vel_target = speed; } // Overloaded: Set desired speed to argument value
    int32_t get_pitch(void) const { return _pitch_target; }  // Get pitch target
    float calc_speed_forward(void);  // Calculates the forward speed in the horizontal plane
    void set_dt(float delta_sec);
	void flare_controller();
	void touchdown_controller();
	void set_ground_distance(float radalt) { _radar_alt = radalt; }
	void get_entry_speed();
	float get_ground_distance() const { return _radar_alt; }
	float get_time_to_ground() const { return _time_to_ground; }
	void time_to_ground();
	void set_collective_minimum_drag(float col_mid )const;
	void get_collective_minimum_drag(float col_mid )  { _col_mid = col_mid; }
    void set_entry_sink_rate (float sink_rate) { _entry_sink_rate = sink_rate; }
    void set_entry_alt (float entry_alt) { _entry_alt = entry_alt; }
	void set_ground_clearance(float ground_clearance) { _ground_clearance = ground_clearance; }

    // User Settable Parameters
    static const struct AP_Param::GroupInfo var_info[];
	AP_Int16 _param_target_speed;
	AP_Float _param_flr_alt;
	AP_Float _param_time_to_ground;
	AP_Int16 _param_head_speed_set_point;	
	AP_Int8  _param_guided;

private:

    //--------Internal Variables--------
    float _current_rpm;
    float _collective_out;
    float _head_speed_error;         // Error between target head speed and current head speed.  Normalised by head speed set point RPM.
    float _col_cutoff_freq;          // Lowpass filter cutoff frequency (Hz) for collective.
    uint8_t _unhealthy_rpm_counter;  // Counter used to track RPM sensor unhealthy signal.
    uint8_t _healthy_rpm_counter;    // Counter used to track RPM sensor healthy signal.
    float _target_head_speed;        // Normalised target head speed.  Normalised by head speed set point RPM.
    float _p_term_hs;                // Proportional contribution to collective setting.
    float _ff_term_hs;               // Following trim feed forward contribution to collective setting.
    float _vel_target;               // Forward velocity target.
    float _pitch_target;             // Pitch angle target.
    float _accel_max;                // Maximum acceleration limit.
    int16_t _speed_forward_last;       // The forward speed calculated in the previous cycle.
    bool _flag_limit_accel;          // Maximum acceleration limit reached flag.
    float _accel_out_last;           // Acceleration value used to calculate pitch target in previous cycle.
    float _cmd_vel;                  // Command velocity, used to get PID values for acceleration calculation.
    float _accel_target;             // Acceleration target, calculated from PID.
    float _delta_speed_fwd;          // Change in forward speed between computation cycles.
    float _dt;                       // Time step.
    int16_t _speed_forward;            // Measured forward speed.
    float _vel_p;                    // Forward velocity P term.
    float _vel_ff;                   // Forward velocity Feed Forward term.
    float _accel_out;                // Acceleration value used to calculate pitch target.
	float _entry_sink_rate;
    float _entry_alt;
	float  _radar_alt;
	float  _rpm_decay;
	float _flare_entry_speed;
	float _desired_speed;
	float _time_to_ground;
	float _distance_to_ground;
	float _desired_sink_rate;
	float _col_mid;
	float _ground_clearance;

    LowPassFilterFloat _accel_target_filter; // acceleration target filter

    //--------Parameter Values--------
    AP_Int8  _param_enable;
    AC_P _p_hs;
    AC_P _p_fw_vel;
	AC_P _p_coll_tch;
    AP_Float _param_col_entry_cutoff_freq;
    AP_Float _param_col_glide_cutoff_freq;
	AP_Float _param_col_cushion_cutoff_freq;
    AP_Int16 _param_accel_max;
    AP_Float _param_bail_time;
    AP_Int8  _param_rpm_instance;
    AP_Float _param_fwd_k_ff;

    //--------Internal Flags--------
    struct controller_flags {
            bool bad_rpm             : 1;
            bool bad_rpm_warning     : 1;
    } _flags;

    //--------Internal Functions--------
    void set_collective(float _collective_filter_cutoff) const;

    // low pass filter for collective trim
    LowPassFilterFloat col_trim_lpf;
	
	//--------References to Other Libraries--------
    AP_InertialNav&    _inav;
};
