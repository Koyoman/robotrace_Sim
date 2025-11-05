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

LINESIM_API void step_dynamics_C(
    double x, double y, double heading_deg,
    double vL, double vR,
    int pwmL, int pwmR,
    double v_final, double tau, double trackW, double dt_s,
    double* out_x, double* out_y, double* out_heading_deg,
    double* out_vL, double* out_vR,
    double* out_v, double* out_w);

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

#ifdef __cplusplus
}
#endif
