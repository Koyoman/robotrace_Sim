#define LINESIM_EXPORTS
#include "linesim.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

static inline double dot(double ax,double ay,double bx,double by)
{
    return ax*bx + ay*by;
}
static inline double clamp01(double t)
{
    return t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t);
}

static inline void rot(double x, double y, double ang, double* rx, double* ry)
{
    double c = cos(ang), s = sin(ang);
    *rx =  c*x - s*y;
    *ry =  s*x + c*y;
}

static inline double dist_point_seg(double px,double py,
                                    double ax,double ay,double bx,double by)
{
    double vx = bx-ax, vy = by-ay;
    double wx = px-ax, wy = py-ay;
    double L2 = vx*vx + vy*vy;
    if (L2 <= 1e-18) {
        double dx = px-ax, dy = py-ay;
        return sqrt(dx*dx + dy*dy);
    }
    double t = clamp01((wx*vx + wy*vy)/L2);
    double cx = ax + t*vx, cy = ay + t*vy;
    double dx = px - cx, dy = py - cy;
    return sqrt(dx*dx + dy*dy);
}

LINESIM_API double dist_point_to_polyline_mm(
    double px, double py, const Pt* poly, int npts)
{
    double best = 1e300;
    if (!poly || npts < 2) return best;
    for (int i=0; i<npts-1; ++i) {
        double d = dist_point_seg(px,py, poly[i].x,poly[i].y, poly[i+1].x,poly[i+1].y);
        if (d < best) best = d;
    }
    return best;
}

LINESIM_API int envelope_contacts_tape_C(
    double cx, double cy, double heading_rad,
    double env_w, double env_h,
    const Pt* poly, int npts,
    double half, int grid_n)
{
    if (grid_n < 3) grid_n = 3;
    double hw = env_w * 0.5, hh = env_h * 0.5;

    for (int iy=0; iy<grid_n; ++iy) {
        double ly = -hh + (2.0*hh) * ((double)iy / (double)(grid_n-1));
        for (int ix=0; ix<grid_n; ++ix) {
            double lx = -hw + (2.0*hw) * ((double)ix / (double)(grid_n-1));
            double rx, ry; rot(lx, ly, heading_rad, &rx, &ry);
            double px = cx + rx, py = cy + ry;
            if (dist_point_to_polyline_mm(px,py,poly,npts) <= half) return 1;
        }
    }
    return 0;
}

LINESIM_API double estimate_sensor_coverage_C(
    double px, double py,
    const Pt* poly, int npts,
    double tape_half, double sensor_size, int n){
    if (n < 2) n = 2;
    int inside = 0, tot = n*n;
    double start = -0.5 * sensor_size;
    double step = (n==1) ? 0.0 : (sensor_size / (double)(n-1));
    for (int iy=0; iy<n; ++iy) {
        for (int ix=0; ix<n; ++ix) {
            double sx = px + start + ix*step;
            double sy = py + start + iy*step;
            if (dist_point_to_polyline_mm(sx,sy,poly,npts) <= tape_half) inside++;
        }
    }
    return ((double)inside / (double)tot);
}

LINESIM_API void estimate_sensors_coverage_batch_C(
    const double* px, const double* py, int n,
    const Pt* poly, int npts,
    double tape_half,
    const double* sensor_sizes,
    double sensor_size_default,
    int n_grid,
    double* out_cov)
{
    if (!px || !py || !out_cov || n <= 0) return;
    if (n_grid < 2) n_grid = 2;
    for (int i=0;i<n;i++) {
        double sz = sensor_sizes ? sensor_sizes[i] : sensor_size_default;
        double start = -0.5 * sz;
        double step = (n_grid==1) ? 0.0 : (sz / (double)(n_grid-1));
        int inside = 0, tot = n_grid*n_grid;
        for (int iy=0; iy<n_grid; ++iy) {
            for (int ix=0; ix<n_grid; ++ix) {
                double sx = px[i] + start + ix*step;
                double sy = py[i] + start + iy*step;
                if (dist_point_to_polyline_mm(sx,sy,poly,npts) <= tape_half) inside++;
            }
        }
        out_cov[i] = (double)inside / (double)tot;
    }
}

static inline int orient(double ax,double ay,double bx,double by,double cx,double cy)
{
    double v = (bx-ax)*(cy-ay) - (by-ay)*(cx-ax);
    return (v > 0) - (v < 0);
}
static inline int on_segment(double ax,double ay,double bx,double by,double px,double py)
{
    if (fmin(ax,bx) - 1e-12 <= px && px <= fmax(ax,bx) + 1e-12 &&
        fmin(ay,by) - 1e-12 <= py && py <= fmax(ay,by) + 1e-12) {
        double cross = (bx-ax)*(py-ay) - (by-ay)*(px-ax);
        return fabs(cross) <= 1e-12;
    }
    return 0;
}

static inline int sample_px(const unsigned char* mask, int W, int H,
                            double origin_x, double origin_y, double pixel_mm,
                            double wx, double wy)
{
    int px = (int)floor((wx - origin_x) / pixel_mm);
    int py = (int)floor((wy - origin_y) / pixel_mm);
    if ((unsigned)px >= (unsigned)W || (unsigned)py >= (unsigned)H) return 0;
    return mask[py*W + px] ? 1 : 0;
}

LINESIM_API int segments_intersect_C(
    double ax,double ay,double bx,double by,
    double cx,double cy,double dx,double dy)
{
    int o1 = orient(ax,ay,bx,by,cx,cy);
    int o2 = orient(ax,ay,bx,by,dx,dy);
    int o3 = orient(cx,cy,dx,dy,ax,ay);
    int o4 = orient(cx,cy,dx,dy,bx,by);

    if (o1 != o2 && o3 != o4) return 1;

    if (o1 == 0 && on_segment(ax,ay,bx,by,cx,cy)) return 1;
    if (o2 == 0 && on_segment(ax,ay,bx,by,dx,dy)) return 1;
    if (o3 == 0 && on_segment(cx,cy,dx,dy,ax,ay)) return 1;
    if (o4 == 0 && on_segment(cx,cy,dx,dy,bx,by)) return 1;
    return 0;
}

LINESIM_API int crossed_finish_C(
    double x0,double y0,double x1,double y1,
    double fx0,double fy0,double fx1,double fy1)
{
    return segments_intersect_C(x0,y0,x1,y1, fx0,fy0,fx1,fy1);
}

LINESIM_API void poly_copy_from_xy(const double* xs, const double* ys, int n, Pt* out)
{
    for (int i=0;i<n;i++){ out[i].x = xs[i]; out[i].y = ys[i]; }
}

LINESIM_API void step_dynamics_C(
    double x, double y, double heading_deg,
    double vL, double vR,
    int pwmL, int pwmR,
    double v_final, double tau, double trackW, double dt_s,
    double* out_x, double* out_y, double* out_heading_deg,
    double* out_vL, double* out_vR,
    double* out_v, double* out_w)
{
    if (tau < 1e-9) tau = 1e-9;
    if (trackW < 1e-9) trackW = 1e-9;
    if (dt_s < 0.0) dt_s = 0.0;

    if (pwmL > 4095) pwmL = 4095; else if (pwmL < -4095) pwmL = -4095;
    if (pwmR > 4095) pwmR = 4095; else if (pwmR < -4095) pwmR = -4095;

    double vL_cmd = ( (double)pwmL / 4095.0 ) * v_final;
    double vR_cmd = ( (double)pwmR / 4095.0 ) * v_final;
    double alpha = 1.0 - exp(-dt_s / tau);

    vL += (vL_cmd - vL) * alpha;
    vR += (vR_cmd - vR) * alpha;

    double v = 0.5 * (vL + vR);
    double w = (vR - vL) / trackW;

    double h_rad = heading_deg * (M_PI / 180.0);
    h_rad += w * dt_s;

    double x2 = x + v * dt_s * cos(h_rad);
    double y2 = y + v * dt_s * sin(h_rad);
    double h_deg2 = h_rad * (180.0 / M_PI);

    if (out_x) *out_x = x2;
    if (out_y) *out_y = y2;
    if (out_heading_deg) *out_heading_deg = h_deg2;
    if (out_vL) *out_vL = vL;
    if (out_vR) *out_vR = vR;
    if (out_v) *out_v = v;
    if (out_w) *out_w = w;
}

LINESIM_API int envelope_contacts_raster_C(
    double cx, double cy, double heading_rad,
    double env_w, double env_h,
    const unsigned char* mask, int W, int H,
    double origin_x, double origin_y, double pixel_mm)
{
    if (!mask || W <= 0 || H <= 0) return 0;
    if (pixel_mm <= 0.0) pixel_mm = 1.0;

    const double ca = cos(heading_rad);
    const double sa = sin(heading_rad);
    const double ux = ca,  uy = sa;
    const double vx = -sa, vy = ca;
    const double halfL = env_h * 0.5;
    const double halfW = env_w * 0.5;

    const double C[4][2] = {
        { cx - vx*halfW - ux*halfL, cy - vy*halfW - uy*halfL },
        { cx + vx*halfW - ux*halfL, cy + vy*halfW - uy*halfL },
        { cx + vx*halfW + ux*halfL, cy + vy*halfW + uy*halfL },
        { cx - vx*halfW + ux*halfL, cy - vy*halfW + uy*halfL }
    };

    for (int e = 0; e < 4; ++e) {
        int e2 = (e + 1) & 3;
        double x1 = C[e][0],  y1 = C[e][1];
        double x2 = C[e2][0], y2 = C[e2][1];
        double dx = x2 - x1,  dy = y2 - y1;
        double len = sqrt(dx*dx + dy*dy);
        int steps = (int)ceil(len / pixel_mm);
        if (steps < 1) steps = 1;
        double inv = 1.0 / (double)steps;
        for (int i = 0; i <= steps; ++i) {
            double t = i * inv;
            double wx = x1 + t*dx;
            double wy = y1 + t*dy;
            if (sample_px(mask, W, H, origin_x, origin_y, pixel_mm, wx, wy)) return 1;
        }
    }
    return 0;
}
