#define LINESIM_EXPORTS
#include "linesim.h"
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

static inline double dot(double ax,double ay,double bx,double by){ return ax*bx + ay*by; }
static inline double clamp01(double t){ return t < 0.0 ? 0.0 : (t > 1.0 ? 1.0 : t); }

static inline void rot(double x, double y, double ang, double* rx, double* ry){
    double c = cos(ang), s = sin(ang);
    *rx =  c*x - s*y;
    *ry =  s*x + c*y;
}

static inline double dist_point_seg(double px,double py,
                                    double ax,double ay,double bx,double by){
    double vx = bx-ax, vy = by-ay;
    double wx = px-ax, wy = py-ay;
    double L2 = vx*vx + vy*vy;
    if (L2 <= 1e-18){
        double dx = px-ax, dy = py-ay;
        return sqrt(dx*dx + dy*dy);
    }
    double t = clamp01((wx*vx + wy*vy)/L2);
    double cx = ax + t*vx, cy = ay + t*vy;
    double dx = px - cx, dy = py - cy;
    return sqrt(dx*dx + dy*dy);
}

double dist_point_to_polyline_mm(
    double px, double py, const Pt* poly, int npts){
    double best = 1e300;
    if (!poly || npts < 2) return best;
    for (int i=0; i<npts-1; ++i){
        double d = dist_point_seg(px,py, poly[i].x,poly[i].y, poly[i+1].x,poly[i+1].y);
        if (d < best) best = d;
    }
    return best;
}

int envelope_contacts_tape_C(
    double cx, double cy, double heading_rad,
    double env_w, double env_h,
    const Pt* poly, int npts,
    double half, int grid_n){
    if (grid_n < 3) grid_n = 3;
    double hw = env_w * 0.5, hh = env_h * 0.5;

    for (int iy=0; iy<grid_n; ++iy){
        double ly = -hh + (2.0*hh) * ((double)iy / (double)(grid_n-1));
        for (int ix=0; ix<grid_n; ++ix){
            double lx = -hw + (2.0*hw) * ((double)ix / (double)(grid_n-1));
            double rx, ry; rot(lx, ly, heading_rad, &rx, &ry);
            double px = cx + rx, py = cy + ry;
            if (dist_point_to_polyline_mm(px,py,poly,npts) <= half) return 1;
        }
    }
    return 0;
}

double estimate_sensor_coverage_C(
    double px, double py,
    const Pt* poly, int npts,
    double tape_half, double sensor_size, int n){
    if (n < 2) n = 2;
    int inside = 0, tot = n*n;
    double start = -0.5 * sensor_size;
    double step = (n==1) ? 0.0 : (sensor_size / (double)(n-1));
    for (int iy=0; iy<n; ++iy){
        for (int ix=0; ix<n; ++ix){
            double sx = px + start + ix*step;
            double sy = py + start + iy*step;
            if (dist_point_to_polyline_mm(sx,sy,poly,npts) <= tape_half) inside++;
        }
    }
    return (double)inside / (double)tot;
}

/* ===== NOVO: versão em lote para coberturas de sensores ===== */
void estimate_sensors_coverage_batch_C(
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
    for (int i=0;i<n;i++){
        double sz = sensor_sizes ? sensor_sizes[i] : sensor_size_default;
        double start = -0.5 * sz;
        double step = (n_grid==1) ? 0.0 : (sz / (double)(n_grid-1));
        int inside = 0, tot = n_grid*n_grid;
        for (int iy=0; iy<n_grid; ++iy){
            for (int ix=0; ix<n_grid; ++ix){
                double sx = px[i] + start + ix*step;
                double sy = py[i] + start + iy*step;
                if (dist_point_to_polyline_mm(sx,sy,poly,npts) <= tape_half) inside++;
            }
        }
        out_cov[i] = (double)inside / (double)tot;
    }
}

static inline int orient(double ax,double ay,double bx,double by,double cx,double cy){
    double v = (bx-ax)*(cy-ay) - (by-ay)*(cx-ax);
    return (v > 0) - (v < 0); // 1, 0, -1
}
static inline int on_segment(double ax,double ay,double bx,double by,double px,double py){
    if (fmin(ax,bx) - 1e-12 <= px && px <= fmax(ax,bx) + 1e-12 &&
        fmin(ay,by) - 1e-12 <= py && py <= fmax(ay,by) + 1e-12){
        double cross = (bx-ax)*(py-ay) - (by-ay)*(px-ax);
        return fabs(cross) <= 1e-12;
    }
    return 0;
}

int segments_intersect_C(
    double ax,double ay,double bx,double by,
    double cx,double cy,double dx,double dy){
    int o1 = orient(ax,ay,bx,by,cx,cy);
    int o2 = orient(ax,ay,bx,by,dx,dy);
    int o3 = orient(cx,cy,dx,dy,ax,ay);
    int o4 = orient(cx,cy,dx,dy,bx,by);

    if (o1 != o2 && o3 != o4) return 1; // geral

    // casos colineares
    if (o1 == 0 && on_segment(ax,ay,bx,by,cx,cy)) return 1;
    if (o2 == 0 && on_segment(ax,ay,bx,by,dx,dy)) return 1;
    if (o3 == 0 && on_segment(cx,cy,dx,dy,ax,ay)) return 1;
    if (o4 == 0 && on_segment(cx,cy,dx,dy,bx,by)) return 1;
    return 0;
}

int crossed_finish_C(
    double x0,double y0,double x1,double y1,
    double fx0,double fy0,double fx1,double fy1){
    return segments_intersect_C(x0,y0,x1,y1, fx0,fy0,fx1,fy1);
}

void poly_copy_from_xy(const double* xs, const double* ys, int n, Pt* out){
    for (int i=0;i<n;i++){ out[i].x = xs[i]; out[i].y = ys[i]; }
}
