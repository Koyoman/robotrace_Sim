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

static inline double clamp(double v, double a, double b){ return v < a ? a : (v > b ? b : v); }
static inline double sgn(double v){ return (v>0) - (v<0); }

static inline double pwm_to_duty(int pwm, double pmin, double pmax, double pcenter, double deadband)
{
    double p = (double)pwm;
    if (p < pmin) p = pmin; else if (p > pmax) p = pmax;
    double span_pos = (pmax - pcenter);
    double span_neg = (pcenter - pmin);
    double duty;
    if (p >= pcenter) duty = (span_pos>1e-12) ? ((p - pcenter)/span_pos) : 0.0;
    else              duty = (span_neg>1e-12) ? ((p - pcenter)/span_neg) : 0.0;
    if (fabs(duty) < deadband) return 0.0;
    if (duty > 0.0) duty = (duty - deadband)/fmax(1e-12, 1.0 - deadband);
    else            duty = (duty + deadband)/fmax(1e-12, 1.0 - deadband);
    return clamp(duty, -1.0, 1.0);
}

typedef struct {
    double dx, dy, dh, dv, dw, dIL, dIR;
} Deriv;

static Deriv deriv_dc(
    double x, double y, double h, double v, double w, double IL, double IR,
    int pwmL, int pwmR,
    double pmin, double pmax, double pcenter, double deadband,
    double Vb, double Rb, double Rw, double Vdrop,
    double Rm, double Lm, double Kt, double Ke, double b, double tau_c,
    double gear, double eta, double mass, double track, double r, double Jz,
    double Crr, double rho, double CdA,
    double mu_s, double mu_k)
{

    double track_m = track;
    double r_m = r;
    if (track_m > 2.0) track_m *= 0.001;
    if (r_m > 1.0)     r_m     *= 0.001;
    if (mass < 1e-6)   mass = 1e-6;
    if (Jz   < 1e-9)   Jz   = 1e-9;

    double vL = v - 0.5*w*track_m;
    double vR = v + 0.5*w*track_m;
    double omg_wL = vL / fmax(1e-12, r_m);
    double omg_wR = vR / fmax(1e-12, r_m);
    double omg_mL = gear * omg_wL;
    double omg_mR = gear * omg_wR;

    double dutyL = pwm_to_duty(pwmL, pmin, pmax, pcenter, deadband);
    double dutyR = pwm_to_duty(pwmR, pmin, pmax, pcenter, deadband);

    double Ibatt = fabs(IL) + fabs(IR);
    double Vbus  = fmax(0.0, Vb - Ibatt*(Rb + Rw) - Vdrop);
    double VapplL = dutyL * Vbus;
    double VapplR = dutyR * Vbus;

    double dIL = (VapplL - Rm*IL - Ke*omg_mL) / fmax(1e-12, Lm);
    double dIR = (VapplR - Rm*IR - Ke*omg_mR) / fmax(1e-12, Lm);

    double TmL = Kt*IL - b*omg_mL - (tau_c * ( (omg_mL>0)-(omg_mL<0) ));
    double TmR = Kt*IR - b*omg_mR - (tau_c * ( (omg_mR>0)-(omg_mR<0) ));

    double TwL = eta * gear * TmL;
    double TwR = eta * gear * TmR;

    double FwL = TwL / fmax(1e-12, r_m);
    double FwR = TwR / fmax(1e-12, r_m);

    double sign_v = (fabs(v) < 1e-6) ? 0.0 : sgn(v);
    double F_roll_each = Crr * mass * 9.81 * 0.5 * sign_v;
    double F_drag_total = 0.5 * rho * CdA * v * fabs(v);
    double F_drag_each  = 0.5 * F_drag_total;

    double N_each = 0.5 * mass * 9.81;
    double Fmax_each = mu_s * N_each;
    if (fabs(FwL) > Fmax_each) FwL = mu_k * N_each * sgn(FwL);
    if (fabs(FwR) > Fmax_each) FwR = mu_k * N_each * sgn(FwR);

    double FnetL = FwL - F_roll_each - F_drag_each;
    double FnetR = FwR - F_roll_each - F_drag_each;

    double a = (FnetL + FnetR) / fmax(1e-12, mass);
    double alpha = ((FnetR - FnetL) * (0.5*track_m)) / fmax(1e-12, Jz);

    double dx = v * cos(h);
    double dy = v * sin(h);
    double dh = w;

    Deriv out = { dx, dy, dh, a, alpha, dIL, dIR };
    return out;
}

LINESIM_API void step_motor_drivetrain_C(
    double x, double y, double h,
    double v, double w, double IL, double IR,
    int pwmL, int pwmR,
    double pmin, double pmax, double pcenter, double deadband,
    double Vb, double Rb, double Rw, double Vdrop,
    double Rm, double Lm, double Kt, double Ke,
    double b, double tau_c,
    double gear, double eta,
    double mass, double track, double r, double Jz,
    double Crr, double rho, double CdA,
    double mu_s, double mu_k,
    double Imax,
    double dt,
    double* ox, double* oy, double* oh,
    double* ov, double* ow, double* oIL, double* oIR)
{
    if (dt <= 0.0) dt = 1e-3;

    Deriv k1 = deriv_dc(x,y,h,v,w,IL,IR,
                        pwmL,pwmR,pmin,pmax,pcenter,deadband,
                        Vb,Rb,Rw,Vdrop,
                        Rm,Lm,Kt,Ke,b,tau_c,
                        gear,eta,mass,track,r,Jz,
                        Crr,rho,CdA,mu_s,mu_k);

    double xp = x + dt*k1.dx;
    double yp = y + dt*k1.dy;
    double hp = h + dt*k1.dh;
    double vp = v + dt*k1.dv;
    double wp = w + dt*k1.dw;
    double ILp = IL + dt*k1.dIL;
    double IRp = IR + dt*k1.dIR;
    if (Imax > 0.0) {
        if (ILp > Imax) ILp = Imax; else if (ILp < -Imax) ILp = -Imax;
        if (IRp > Imax) IRp = Imax; else if (IRp < -Imax) IRp = -Imax;
    }

    Deriv k2 = deriv_dc(xp,yp,hp,vp,wp,ILp,IRp,
                        pwmL,pwmR,pmin,pmax,pcenter,deadband,
                        Vb,Rb,Rw,Vdrop,
                        Rm,Lm,Kt,Ke,b,tau_c,
                        gear,eta,mass,track,r,Jz,
                        Crr,rho,CdA,mu_s,mu_k);

    x  += 0.5*dt*(k1.dx  + k2.dx);
    y  += 0.5*dt*(k1.dy  + k2.dy);
    h  += 0.5*dt*(k1.dh  + k2.dh);
    v  += 0.5*dt*(k1.dv  + k2.dv);
    w  += 0.5*dt*(k1.dw  + k2.dw);
    IL += 0.5*dt*(k1.dIL + k2.dIL);
    IR += 0.5*dt*(k1.dIR + k2.dIR);

{
    const double tau_e = Lm / fmax(1e-12, Rm);
    const double v_mid = v;
    const double w_mid = w;
    const double vL_mid = v_mid - 0.5*w_mid*track;
    const double vR_mid = v_mid + 0.5*w_mid*track;
    const double omg_mL_mid = gear * (vL_mid / fmax(1e-12, r));
    const double omg_mR_mid = gear * (vR_mid / fmax(1e-12, r));

    const double dutyL = pwm_to_duty(pwmL, pmin, pmax, pcenter, deadband);
    const double dutyR = pwm_to_duty(pwmR, pmin, pmax, pcenter, deadband);

    const double Ibatt_mid = fabs(IL) + fabs(IR);
    const double Vbus_mid  = fmax(0.0, Vb - Ibatt_mid*(Rb + Rw) - Vdrop);

    const double VapplL_mid = dutyL * Vbus_mid;
    const double VapplR_mid = dutyR * Vbus_mid;

    const double i_inf_L = (VapplL_mid - Ke*omg_mL_mid) / fmax(1e-12, Rm);
    const double i_inf_R = (VapplR_mid - Ke*omg_mR_mid) / fmax(1e-12, Rm);

    const double decay = exp(-dt / fmax(1e-12, tau_e));
    IL = i_inf_L + (IL - i_inf_L) * decay;
    IR = i_inf_R + (IR - i_inf_R) * decay;
}

    if (Imax > 0.0) {
        if (IL > Imax) IL = Imax; else if (IL < -Imax) IL = -Imax;
        if (IR > Imax) IR = Imax; else if (IR < -Imax) IR = -Imax;
    }

    if (ox)  *ox = x;
    if (oy)  *oy = y;
    if (oh)  *oh = h;
    if (ov)  *ov = v;
    if (ow)  *ow = w;
    if (oIL) *oIL = IL;
    if (oIR) *oIR = IR;
}
