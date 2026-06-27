#pragma once
#ifdef _WIN32
  #ifdef LINESIM_EXPORTS
    #define LINESIM_API __declspec(dllexport)
  #else
    #define LINESIM_API __declspec(dllimport)
  #endif
#else
  #define LINESIM_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef struct { double x, y; } Pt;

LINESIM_API double dist_point_to_polyline_mm(
    double px, double py, const Pt* poly, int npts);

LINESIM_API int envelope_contacts_tape_C(
    double cx, double cy, double heading_rad,
    double env_w, double env_h,
    const Pt* poly, int npts,
    double tape_half_with_margin,
    int grid_n);

LINESIM_API double estimate_sensor_coverage_C(
    double px, double py,
    const Pt* poly, int npts,
    double tape_half,
    double sensor_size,
    int n_grid);

LINESIM_API void estimate_sensors_coverage_batch_C(
    const double* px, const double* py, int n,
    const Pt* poly, int npts,
    double tape_half,
    const double* sensor_sizes,
    double sensor_size_default,
    int n_grid,
    double* out_cov);

LINESIM_API int segments_intersect_C(
    double ax, double ay, double bx, double by,
    double cx, double cy, double dx, double dy);

LINESIM_API int crossed_finish_C(
    double x0, double y0, double x1, double y1,
    double fx0, double fy0, double fx1, double fy1);

LINESIM_API void poly_copy_from_xy(
    const double* xs, const double* ys, int n, Pt* out);

LINESIM_API int envelope_contacts_raster_C(
    double cx, double cy, double heading_rad,
    double env_w, double env_h,
    const unsigned char* mask, int W, int H,
    double origin_x, double origin_y, double pixel_mm);

LINESIM_API void step_motor_drivetrain_C(
    double x_m, double y_m, double heading_rad,
    double v_mps, double w_radps, double I_L_A, double I_R_A,
    int pwmL, int pwmR,
    double pwm_min, double pwm_max, double pwm_center, double deadband_01,
    double V_batt, double R_batt, double R_wiring, double V_driver_drop,
    double Rm, double Lm, double Kt, double Ke,
    double b_visc, double tau_coulomb,
    double gear, double eta_drive,
    double mass, double track, double wheel_r, double Jz,
    double Crr, double rho, double CdA,
    double mu_static, double mu_kinetic,
    double I_max,
    double dt_s,
    double* out_x_m, double* out_y_m, double* out_heading_rad,
    double* out_v_mps, double* out_w_radps,
    double* out_I_L_A, double* out_I_R_A);


#define LINESIM_ABI_VERSION 4

typedef struct {
    double pwm_left;
    double pwm_right;
    double max_pwm;
    double ocv_voltage_v;
} PhysicsInputC;

typedef struct {
    int use_dc_motor_model;
    int use_kinematic_model;
    int use_acceleration_limit;
    int use_battery_model;
    int use_wheel_slip;
    int use_encoder_model;
    int use_imu_model;
    double dt_s;
    double max_wheel_accel_mm_s2;
    double max_wheel_speed_mm_s;
    double battery_voltage_v;
    double battery_nominal_voltage_v;
    double battery_min_voltage_v;
    double battery_capacity_mah;
    double battery_internal_resistance_ohm;
    double battery_soc;
    double slip_ratio_left;
    double slip_ratio_right;
    int encoder_ticks_per_rev;
    int encoder_quantization;
    double track_m;
    double wheel_radius_m;
    double pwm_min;
    double pwm_max;
    double pwm_center;
    double motor_deadzone_pwm;
    double current_limit_a;
    double drivetrain_efficiency;
    double viscous_friction;
    double coulomb_friction;
    double driver_drop_v;
    double rm_ohm;
    double lm_h;
    double kt_nm_per_a;
    double ke_v_per_rad;
    double gear_ratio;
    double mass_kg;
    double jz_kgm2;
    double crr;
    double rho_air;
    double cda;
    double mu_static;
    double mu_kinetic;
    int use_wheel_dynamics;
    int use_continuous_slip;
    int use_lateral_slip;
    int use_combined_friction_limit;
    int use_energy_balance;
    double j_motor_kgm2;
    double j_load_kgm2;
    double wheel_mass_kg;
    double r_batt_ohm;
    double wiring_r_ohm;
    double slip_stiffness_factor;
    double slip_at_limit;
    double slip_max_ratio;
    double mu_static_left;
    double mu_static_right;
    double mu_kinetic_left;
    double mu_kinetic_right;
} PhysicsConfigC;

typedef struct {
    double x_mm;
    double y_mm;
    double heading_deg;
    double v_left_mm_s;
    double v_right_mm_s;
    double v_mm_s;
    double omega_rad_s;
    double a_lin_mm_s2;
    double alpha_rad_s2;
    double battery_voltage_v;
    double battery_soc;
    double enc_left_ticks;
    double enc_right_ticks;
    double enc_left_delta_ticks;
    double enc_right_delta_ticks;
    double imu_omega_rad_s;
    double imu_alpha_rad_s2;
    double imu_accel_x_mm_s2;
    double imu_accel_y_mm_s2;
    double current_left_a;
    double current_right_a;
    double omega_wheel_left_rad_s;
    double omega_wheel_right_rad_s;
    double alpha_wheel_left_rad_s2;
    double alpha_wheel_right_rad_s2;
    double battery_energy_j;
    double copper_loss_energy_j;
    double driver_loss_energy_j;
    double battery_internal_loss_energy_j;
    double wiring_loss_energy_j;
    double mechanical_friction_loss_energy_j;
    double rolling_resistance_energy_j;
    double tire_slip_loss_energy_j;
    double brake_dissipated_energy_j;
} PhysicsStateC;

typedef struct {
    double duty_left, duty_right;
    double current_left_a, current_right_a, battery_current_a;
    double tau_motor_em_left_nm, tau_motor_em_right_nm;
    double tau_motor_viscous_left_nm, tau_motor_viscous_right_nm;
    double tau_motor_coulomb_left_nm, tau_motor_coulomb_right_nm;
    double tau_motor_net_left_nm, tau_motor_net_right_nm;
    double tau_wheel_drive_left_nm, tau_wheel_drive_right_nm;
    double tau_rolling_left_nm, tau_rolling_right_nm;
    double tau_bearing_left_nm, tau_bearing_right_nm;
    double tau_ground_left_nm, tau_ground_right_nm;
    double tau_slip_loss_left_nm, tau_slip_loss_right_nm;
    double force_longitudinal_command_left_n, force_longitudinal_command_right_n;
    double force_longitudinal_ground_left_n, force_longitudinal_ground_right_n;
    double force_longitudinal_max_left_n, force_longitudinal_max_right_n;
    double force_longitudinal_saturation_left, force_longitudinal_saturation_right;
    double lambda_long_left, lambda_long_right;
    double lateral_accel_mm_s2;
    double lateral_force_total_n, lateral_force_left_n, lateral_force_right_n;
    double lateral_slip_left, lateral_slip_right;
    double friction_usage_left, friction_usage_right;
    double combined_friction_limit_left_n, combined_friction_limit_right_n;
    double slip_ratio_left, slip_ratio_right;
    double wheel_left_surface_speed_mm_s, wheel_right_surface_speed_mm_s;
    double ground_left_speed_mm_s, ground_right_speed_mm_s;
    double j_eq_left_kgm2, j_eq_right_kgm2;
    double battery_power_w;
    double copper_loss_left_w, copper_loss_right_w;
    double driver_loss_left_w, driver_loss_right_w;
    double battery_internal_loss_w, wiring_loss_w;
    double mechanical_friction_loss_left_w, mechanical_friction_loss_right_w;
    double rolling_resistance_loss_w;
    double tire_slip_loss_left_w, tire_slip_loss_right_w;
    double brake_dissipated_power_w;
    double kinetic_power_delta_w;
    double kinetic_energy_linear_j, kinetic_energy_angular_j, kinetic_energy_wheels_j, total_kinetic_energy_j;
    double total_loss_energy_j;
    double energy_balance_error_j, energy_balance_error_percent;
    int step_executed_in_c;
} PhysicsTelemetryC;

LINESIM_API int linesim_abi_version_C(void);
LINESIM_API const char* linesim_backend_name_C(void);

LINESIM_API int step_physics_modular_C(
    const PhysicsInputC* input,
    const PhysicsConfigC* config,
    PhysicsStateC* state,
    PhysicsTelemetryC* telemetry);

#ifdef __cplusplus
}
#endif
