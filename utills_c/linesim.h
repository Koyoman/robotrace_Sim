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

#ifdef __cplusplus
}
#endif
