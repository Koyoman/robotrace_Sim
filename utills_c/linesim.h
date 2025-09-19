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

/* Distância ponto→polilinha (mm) */
LINESIM_API double dist_point_to_polyline_mm(
    double px, double py, const Pt* poly, int npts);

/* Envelope retangular do robô toca a “fita” (polilinha com half-width)? */
LINESIM_API int envelope_contacts_tape_C(
    double cx, double cy, double heading_rad,
    double env_w, double env_h,
    const Pt* poly, int npts,
    double tape_half_with_margin,
    int grid_n);

/* Cobertura (0..1) de um sensor quadrado sobre a fita */
LINESIM_API double estimate_sensor_coverage_C(
    double px, double py,
    const Pt* poly, int npts,
    double tape_half,
    double sensor_size,
    int n_grid);

/* *** NOVO: Cobertura (0..1) para N sensores em lote ***
   px[i], py[i] → out_cov[i], i=0..(n-1)  */
LINESIM_API void estimate_sensors_coverage_batch_C(
    const double* px, const double* py, int n,
    const Pt* poly, int npts,
    double tape_half,
    const double* sensor_sizes, /* opcional: se NULL, usa sensor_size_default */
    double sensor_size_default,
    int n_grid,
    double* out_cov);

/* Interseção de segmentos AB x CD (1=intersecta, 0=não) */
LINESIM_API int segments_intersect_C(
    double ax, double ay, double bx, double by,
    double cx, double cy, double dx, double dy);

/* Cruzou a linha de chegada? (segmento de movimento P0→P1 x linha F0→F1) */
LINESIM_API int crossed_finish_C(
    double x0, double y0, double x1, double y1,
    double fx0, double fy0, double fx1, double fy1);

/* Copia X/Y para vetor de Pt (facilita a partir de arrays Python) */
LINESIM_API void poly_copy_from_xy(
    const double* xs, const double* ys, int n, Pt* out);

#ifdef __cplusplus
}
#endif
