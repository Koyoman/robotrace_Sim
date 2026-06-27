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

    /* Keep the public function tolerant of both SI units and legacy mm inputs.
       deriv_dc already normalizes these, but the exact current update below also
       needs the normalized values. */
    double track_m = track;
    double r_m = r;
    if (track_m > 2.0) track_m *= 0.001;
    if (r_m > 1.0)     r_m     *= 0.001;
    track_m = fmax(1e-12, track_m);
    r_m = fmax(1e-12, r_m);

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
    const double vL_mid = v_mid - 0.5*w_mid*track_m;
    const double vR_mid = v_mid + 0.5*w_mid*track_m;
    const double omg_mL_mid = gear * (vL_mid / r_m);
    const double omg_mR_mid = gear * (vR_mid / r_m);

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

LINESIM_API int linesim_abi_version_C(void)
{
    return LINESIM_ABI_VERSION;
}

LINESIM_API const char* linesim_backend_name_C(void)
{
    return "linesim_c_modular_phase4_4";
}

static inline double clamp_abs(double v, double limit)
{
    if (limit <= 0.0) return v;
    return clamp(v, -limit, limit);
}

static inline double round_half_away(double v)
{
    return (v >= 0.0) ? floor(v + 0.5) : ceil(v - 0.5);
}

typedef struct {
    double duty;
    double omega_wheel0;
    double omega_wheel;
    double alpha_wheel;
    double current_signed;
    double current_abs;
    double tau_em;
    double tau_visc;
    double tau_coul;
    double tau_net;
    double tau_wheel_drive;
    double tau_rolling;
    double tau_bearing;
    double tau_ground;
    double tau_slip_loss;
    double f_cmd;
    double f_ground;
    double f_max;
    double saturation;
    double lambda_long;
    double lateral_force;
    double lateral_slip;
    double friction_usage;
    double combined_limit;
    double slip;
    double surface_speed_mm_s;
    double ground_speed_mm_s;
    double j_eq;
    double copper_loss;
    double driver_loss;
    double mech_friction_loss;
    double rolling_loss;
    double tire_slip_loss;
    double brake_loss;
    double motor_voltage;
    double back_emf;
} WheelCalc;

static WheelCalc wheel_step(
    double pwm, double prev_current, double prev_omega_wheel,
    double prev_ground_speed_mm_s, double other_prev_ground_speed_mm_s,
    const PhysicsConfigC* cfg, double terminal_v, double fz, double f_lat,
    double mu_static, double mu_kinetic, double dt)
{
    WheelCalc w;
    for (unsigned i=0; i<sizeof(WheelCalc); ++i) ((unsigned char*)&w)[i] = 0;

    double r = fmax(1e-12, cfg->wheel_radius_m);
    double gear = fmax(1e-12, cfg->gear_ratio);
    double eta = fmax(1e-12, cfg->drivetrain_efficiency);
    double Rm = fmax(1e-12, cfg->rm_ohm);
    double Lm = fmax(0.0, cfg->lm_h);
    double Ke = cfg->ke_v_per_rad;
    double Kt = cfg->kt_nm_per_a;
    double pmin = cfg->pwm_min, pmax = cfg->pwm_max, pcenter = cfg->pwm_center;
    if (pmax <= pmin) { pmin = -4095.0; pmax = 4095.0; pcenter = 0.0; }
    double span = fmax(fabs(pmax - pcenter), fabs(pcenter - pmin));
    double deadband = cfg->motor_deadzone_pwm > 0.0 ? clamp(cfg->motor_deadzone_pwm / fmax(1.0, span), 0.0, 0.95) : 0.0;
    w.duty = pwm_to_duty((int)pwm, pmin, pmax, pcenter, deadband);

    w.omega_wheel0 = prev_omega_wheel;
    if (fabs(w.omega_wheel0) < 1e-12 && fabs(prev_ground_speed_mm_s) > 1e-12) {
        w.omega_wheel0 = (prev_ground_speed_mm_s * 0.001) / r;
    }

    double omega_motor = w.omega_wheel0 * gear;
    w.back_emf = Ke * omega_motor;
    double driver_available_v = fmax(0.0, terminal_v - fmax(0.0, cfg->driver_drop_v));
    w.motor_voltage = w.duty * driver_available_v;
    double i_inf = (w.motor_voltage - w.back_emf) / Rm;
    if (Lm > 1e-12) {
        double tau_e = Lm / Rm;
        double decay = exp(-dt / fmax(1e-12, tau_e));
        w.current_signed = i_inf + (prev_current - i_inf) * decay;
    } else {
        w.current_signed = i_inf;
    }
    w.current_signed = clamp_abs(w.current_signed, cfg->current_limit_a);
    w.current_abs = fabs(w.current_signed);

    w.tau_em = Kt * w.current_signed;
    w.tau_visc = cfg->viscous_friction * omega_motor;
    if (fabs(omega_motor) > 1e-9) {
        w.tau_coul = cfg->coulomb_friction * sgn(omega_motor);
    } else {
        w.tau_coul = fmin(fabs(w.tau_em), fabs(cfg->coulomb_friction)) * sgn(w.tau_em);
    }
    w.tau_net = w.tau_em - w.tau_visc - w.tau_coul;
    w.tau_wheel_drive = w.tau_net * gear * eta;
    w.f_cmd = w.tau_wheel_drive / r;

    double frr = fmax(0.0, cfg->crr) * fz;
    w.tau_rolling = frr * r * (fabs(w.omega_wheel0) > 1e-9 ? sgn(w.omega_wheel0) : sgn(w.f_cmd));
    w.tau_bearing = 0.0;

    w.f_max = fmax(0.0, mu_static) * fz;
    double f_lat_abs = fabs(f_lat);
    double combined = w.f_max;
    if (cfg->use_combined_friction_limit && f_lat_abs < w.f_max) {
        combined = sqrt(fmax(0.0, w.f_max*w.f_max - f_lat_abs*f_lat_abs));
    } else if (cfg->use_combined_friction_limit && f_lat_abs >= w.f_max) {
        combined = 0.0;
    }
    w.combined_limit = combined;
    double denom_limit = fmax(1e-12, cfg->use_combined_friction_limit ? combined : w.f_max);
    double force_after_rr = w.f_cmd - frr * (fabs(w.omega_wheel0) > 1e-9 ? sgn(w.omega_wheel0) : sgn(w.f_cmd));
    w.lambda_long = fabs(force_after_rr) / denom_limit;

    double slip_max = clamp(cfg->slip_max_ratio > 0.0 ? cfg->slip_max_ratio : 0.95, 0.0, 0.99);
    if (cfg->use_wheel_slip) {
        if (cfg->use_continuous_slip) {
            if (w.lambda_long <= 1.0) {
                w.slip = cfg->slip_stiffness_factor * w.lambda_long * w.lambda_long;
            } else {
                w.slip = cfg->slip_at_limit + (1.0 - denom_limit / fmax(fabs(force_after_rr), 1e-12));
            }
            w.slip = clamp(w.slip, 0.0, slip_max);
        } else {
            w.slip = clamp(cfg->slip_ratio_left, 0.0, slip_max);
        }
    }
    if (cfg->use_lateral_slip) {
        double lat_usage = f_lat_abs / fmax(w.f_max, 1e-12);
        if (lat_usage > 1.0) w.lateral_slip = clamp(lat_usage - 1.0, 0.0, slip_max);
        else w.lateral_slip = 0.03 * lat_usage * lat_usage;
        w.slip = clamp(w.slip + 0.25 * w.lateral_slip, 0.0, slip_max);
    }

    double ground_limit = cfg->use_combined_friction_limit ? combined : w.f_max;
    double kinetic_limit = fmax(0.0, mu_kinetic) * fz;
    if (ground_limit <= 0.0) ground_limit = 0.0;
    if (fabs(force_after_rr) <= ground_limit) {
        w.f_ground = force_after_rr;
    } else {
        double lim = fmin(fmax(0.0, ground_limit), fmax(0.0, kinetic_limit));
        if (lim <= 0.0) lim = ground_limit;
        w.f_ground = sgn(force_after_rr) * lim;
    }
    w.saturation = fabs(force_after_rr) > ground_limit + 1e-9 ? 1.0 : 0.0;
    w.friction_usage = sqrt(w.f_cmd*w.f_cmd + f_lat*f_lat) / fmax(w.f_max, 1e-12);
    w.lateral_force = f_lat;
    w.tau_ground = w.f_ground * r;
    w.tau_slip_loss = (w.f_cmd - w.f_ground) * r;

    double wheel_mass = cfg->wheel_mass_kg > 0.0 ? cfg->wheel_mass_kg : fmax(0.001, 0.03 * cfg->mass_kg);
    double j_wheel = 0.5 * wheel_mass * r * r;
    w.j_eq = fmax(1e-12, cfg->j_load_kgm2 + cfg->j_motor_kgm2 * gear * gear + j_wheel);
    double torque_for_alpha = cfg->use_wheel_dynamics ? (w.tau_wheel_drive - w.tau_rolling - w.tau_bearing) : (w.tau_ground);
    w.alpha_wheel = torque_for_alpha / w.j_eq;
    w.omega_wheel = w.omega_wheel0 + w.alpha_wheel * dt;
    if (!cfg->use_wheel_dynamics) {
        double target = (prev_ground_speed_mm_s * 0.001) / r;
        w.omega_wheel = target;
        w.alpha_wheel = (w.omega_wheel - w.omega_wheel0) / fmax(1e-12, dt);
    }
    w.surface_speed_mm_s = w.omega_wheel * r * 1000.0;
    w.ground_speed_mm_s = w.surface_speed_mm_s * (1.0 - w.slip);

    w.copper_loss = w.current_abs * w.current_abs * Rm;
    w.driver_loss = w.current_abs * fmax(0.0, cfg->driver_drop_v);
    w.mech_friction_loss = fabs(w.tau_visc * omega_motor) + fabs(w.tau_coul * omega_motor);
    w.rolling_loss = fabs(frr * (w.ground_speed_mm_s * 0.001));
    w.tire_slip_loss = fabs(w.f_cmd - w.f_ground) * fabs((w.surface_speed_mm_s - w.ground_speed_mm_s) * 0.001);
    double mech_power = w.tau_wheel_drive * w.omega_wheel;
    w.brake_loss = (w.tau_wheel_drive * w.omega_wheel < 0.0) ? fabs(mech_power) : 0.0;

    return w;
}

LINESIM_API int step_physics_modular_C(
    const PhysicsInputC* input,
    const PhysicsConfigC* cfg,
    PhysicsStateC* st,
    PhysicsTelemetryC* telem)
{
    if (!input || !cfg || !st || !telem) return 1;
    for (unsigned i=0; i<sizeof(PhysicsTelemetryC); ++i) ((unsigned char*)telem)[i] = 0;

    double dt = cfg->dt_s > 0.0 ? cfg->dt_s : 1e-3;
    double track_m = cfg->track_m > 1e-12 ? cfg->track_m : 0.07;
    double wheel_r_m = cfg->wheel_radius_m > 1e-12 ? cfg->wheel_radius_m : 0.011;
    double mass = fmax(1e-9, cfg->mass_kg);
    double jz = cfg->jz_kgm2 > 1e-12 ? cfg->jz_kgm2 : fmax(1e-6, mass * track_m * track_m / 12.0);
    double prev_v = st->v_mm_s;
    double prev_w = st->omega_rad_s;
    double prev_ke = 0.5 * mass * pow(prev_v * 0.001, 2.0) + 0.5 * jz * prev_w * prev_w;

    double source_v = input->ocv_voltage_v > 0.0 ? input->ocv_voltage_v : cfg->battery_voltage_v;
    if (source_v <= 0.0) source_v = cfg->battery_nominal_voltage_v;
    if (source_v <= 0.0) source_v = 7.4;
    double soc = st->battery_soc;
    if (soc < 0.0 || soc > 1.0) soc = clamp(cfg->battery_soc, 0.0, 1.0);
    double min_v = fmax(0.0, cfg->battery_min_voltage_v);
    double ocv = cfg->use_battery_model ? (min_v + soc * fmax(0.0, source_v - min_v)) : source_v;
    double prev_current_abs = fabs(st->current_left_a) + fabs(st->current_right_a);
    double terminal_v_est = fmax(min_v, ocv - prev_current_abs * fmax(0.0, cfg->r_batt_ohm + cfg->wiring_r_ohm));

    double fz = mass * 9.81 * 0.5;
    double v_body_mps = st->v_mm_s * 0.001;
    double lateral_accel_mps2 = v_body_mps * st->omega_rad_s;
    double f_lat_total = cfg->use_lateral_slip ? mass * lateral_accel_mps2 : 0.0;
    double f_lat_each = 0.5 * f_lat_total;

    double mu_sl = cfg->mu_static_left > 0.0 ? cfg->mu_static_left : cfg->mu_static;
    double mu_sr = cfg->mu_static_right > 0.0 ? cfg->mu_static_right : cfg->mu_static;
    double mu_kl = cfg->mu_kinetic_left > 0.0 ? cfg->mu_kinetic_left : cfg->mu_kinetic;
    double mu_kr = cfg->mu_kinetic_right > 0.0 ? cfg->mu_kinetic_right : cfg->mu_kinetic;

    WheelCalc L, R;
    if (cfg->use_dc_motor_model) {
        L = wheel_step(input->pwm_left, st->current_left_a, st->omega_wheel_left_rad_s, st->v_left_mm_s, st->v_right_mm_s, cfg, terminal_v_est, fz, f_lat_each, mu_sl, mu_kl, dt);
        R = wheel_step(input->pwm_right, st->current_right_a, st->omega_wheel_right_rad_s, st->v_right_mm_s, st->v_left_mm_s, cfg, terminal_v_est, fz, f_lat_each, mu_sr, mu_kr, dt);
    } else {
        double pmin=cfg->pwm_min, pmax=cfg->pwm_max, pc=cfg->pwm_center;
        if (pmax <= pmin) { pmin=-4095.0; pmax=4095.0; pc=0.0; }
        double max_speed = cfg->max_wheel_speed_mm_s > 0.0 ? cfg->max_wheel_speed_mm_s : 500.0;
        for (unsigned i=0; i<sizeof(WheelCalc); ++i) { ((unsigned char*)&L)[i]=0; ((unsigned char*)&R)[i]=0; }
        L.duty = pwm_to_duty((int)input->pwm_left, pmin, pmax, pc, 0.0);
        R.duty = pwm_to_duty((int)input->pwm_right, pmin, pmax, pc, 0.0);
        L.surface_speed_mm_s = L.ground_speed_mm_s = L.duty * max_speed;
        R.surface_speed_mm_s = R.ground_speed_mm_s = R.duty * max_speed;
        L.omega_wheel = L.surface_speed_mm_s * 0.001 / wheel_r_m;
        R.omega_wheel = R.surface_speed_mm_s * 0.001 / wheel_r_m;
        L.alpha_wheel = (L.omega_wheel - st->omega_wheel_left_rad_s)/fmax(1e-12,dt);
        R.alpha_wheel = (R.omega_wheel - st->omega_wheel_right_rad_s)/fmax(1e-12,dt);
        double wheel_mass = cfg->wheel_mass_kg > 0.0 ? cfg->wheel_mass_kg : fmax(0.001, 0.03 * mass);
        L.j_eq = R.j_eq = fmax(1e-12, cfg->j_load_kgm2 + cfg->j_motor_kgm2*cfg->gear_ratio*cfg->gear_ratio + 0.5*wheel_mass*wheel_r_m*wheel_r_m);
    }

    if (cfg->use_acceleration_limit && cfg->max_wheel_accel_mm_s2 > 0.0) {
        double max_delta = cfg->max_wheel_accel_mm_s2 * dt;
        double gl0 = st->v_left_mm_s;
        double gr0 = st->v_right_mm_s;
        L.ground_speed_mm_s = gl0 + clamp(L.ground_speed_mm_s - gl0, -max_delta, max_delta);
        R.ground_speed_mm_s = gr0 + clamp(R.ground_speed_mm_s - gr0, -max_delta, max_delta);
        L.surface_speed_mm_s = L.ground_speed_mm_s / fmax(1e-9, 1.0 - L.slip);
        R.surface_speed_mm_s = R.ground_speed_mm_s / fmax(1e-9, 1.0 - R.slip);
        L.omega_wheel = L.surface_speed_mm_s * 0.001 / wheel_r_m;
        R.omega_wheel = R.surface_speed_mm_s * 0.001 / wheel_r_m;
    }

    double v = 0.5 * (L.ground_speed_mm_s + R.ground_speed_mm_s);
    double omega = (R.ground_speed_mm_s - L.ground_speed_mm_s) / fmax(1e-9, track_m * 1000.0);
    double h0 = st->heading_deg * M_PI / 180.0;
    double h_mid = h0 + 0.5 * omega * dt;
    st->x_mm += v * cos(h_mid) * dt;
    st->y_mm += v * sin(h_mid) * dt;
    st->heading_deg = (h0 + omega * dt) * 180.0 / M_PI;
    st->v_left_mm_s = L.ground_speed_mm_s;
    st->v_right_mm_s = R.ground_speed_mm_s;
    st->v_mm_s = v;
    st->omega_rad_s = omega;
    st->a_lin_mm_s2 = (v - prev_v) / fmax(1e-12, dt);
    st->alpha_rad_s2 = (omega - prev_w) / fmax(1e-12, dt);
    st->current_left_a = L.current_signed;
    st->current_right_a = R.current_signed;
    st->omega_wheel_left_rad_s = L.omega_wheel;
    st->omega_wheel_right_rad_s = R.omega_wheel;
    st->alpha_wheel_left_rad_s2 = L.alpha_wheel;
    st->alpha_wheel_right_rad_s2 = R.alpha_wheel;

    double battery_current = fabs(L.current_signed) + fabs(R.current_signed);
    if (cfg->use_battery_model) {
        double capacity_as = fmax(1e-9, cfg->battery_capacity_mah * 3.6);
        soc = clamp(soc - battery_current * dt / capacity_as, 0.0, 1.0);
        ocv = min_v + soc * fmax(0.0, source_v - min_v);
    }
    double rb = cfg->r_batt_ohm > 0.0 ? cfg->r_batt_ohm : cfg->battery_internal_resistance_ohm;
    double rw = fmax(0.0, cfg->wiring_r_ohm);
    double terminal_v = fmax(min_v, ocv - battery_current * (rb + rw));
    st->battery_soc = soc;
    st->battery_voltage_v = terminal_v;

    if (cfg->use_encoder_model) {
        double ticks_per_rad = ((double)(cfg->encoder_ticks_per_rev > 0 ? cfg->encoder_ticks_per_rev : 1)) / (2.0 * M_PI);
        double dl = L.omega_wheel * dt * ticks_per_rad;
        double dr = R.omega_wheel * dt * ticks_per_rad;
        if (cfg->encoder_quantization) { dl = round_half_away(dl); dr = round_half_away(dr); }
        st->enc_left_delta_ticks = dl;
        st->enc_right_delta_ticks = dr;
        st->enc_left_ticks += dl;
        st->enc_right_ticks += dr;
    } else {
        st->enc_left_delta_ticks = 0.0;
        st->enc_right_delta_ticks = 0.0;
    }

    if (cfg->use_imu_model) {
        st->imu_omega_rad_s = st->omega_rad_s;
        st->imu_alpha_rad_s2 = st->alpha_rad_s2;
        st->imu_accel_x_mm_s2 = st->a_lin_mm_s2;
        st->imu_accel_y_mm_s2 = st->v_mm_s * st->omega_rad_s;
    } else {
        st->imu_omega_rad_s = 0.0;
        st->imu_alpha_rad_s2 = 0.0;
        st->imu_accel_x_mm_s2 = 0.0;
        st->imu_accel_y_mm_s2 = 0.0;
    }

    double battery_power = terminal_v * battery_current;
    double internal_loss = battery_current*battery_current*rb;
    double wiring_loss = battery_current*battery_current*rw;
    double rolling_loss = L.rolling_loss + R.rolling_loss;
    double tire_loss = L.tire_slip_loss + R.tire_slip_loss;
    double copper_loss = L.copper_loss + R.copper_loss;
    double driver_loss = L.driver_loss + R.driver_loss;
    double mech_loss = L.mech_friction_loss + R.mech_friction_loss;
    double brake_loss = L.brake_loss + R.brake_loss;

    st->battery_energy_j += battery_power * dt;
    st->copper_loss_energy_j += copper_loss * dt;
    st->driver_loss_energy_j += driver_loss * dt;
    st->battery_internal_loss_energy_j += internal_loss * dt;
    st->wiring_loss_energy_j += wiring_loss * dt;
    st->mechanical_friction_loss_energy_j += mech_loss * dt;
    st->rolling_resistance_energy_j += rolling_loss * dt;
    st->tire_slip_loss_energy_j += tire_loss * dt;
    st->brake_dissipated_energy_j += brake_loss * dt;

    double ke_linear = 0.5 * mass * pow(st->v_mm_s * 0.001, 2.0);
    double ke_angular = 0.5 * jz * st->omega_rad_s * st->omega_rad_s;
    double ke_wheels = 0.5 * L.j_eq * L.omega_wheel * L.omega_wheel + 0.5 * R.j_eq * R.omega_wheel * R.omega_wheel;
    double total_ke = ke_linear + ke_angular + ke_wheels;
    double total_losses = st->copper_loss_energy_j + st->driver_loss_energy_j + st->battery_internal_loss_energy_j + st->wiring_loss_energy_j + st->mechanical_friction_loss_energy_j + st->rolling_resistance_energy_j + st->tire_slip_loss_energy_j + st->brake_dissipated_energy_j;
    double balance = st->battery_energy_j - (total_losses + total_ke);

    telem->duty_left = L.duty; telem->duty_right = R.duty;
    telem->current_left_a = fabs(L.current_signed); telem->current_right_a = fabs(R.current_signed); telem->battery_current_a = battery_current;
    telem->tau_motor_em_left_nm = L.tau_em; telem->tau_motor_em_right_nm = R.tau_em;
    telem->tau_motor_viscous_left_nm = L.tau_visc; telem->tau_motor_viscous_right_nm = R.tau_visc;
    telem->tau_motor_coulomb_left_nm = L.tau_coul; telem->tau_motor_coulomb_right_nm = R.tau_coul;
    telem->tau_motor_net_left_nm = L.tau_net; telem->tau_motor_net_right_nm = R.tau_net;
    telem->tau_wheel_drive_left_nm = L.tau_wheel_drive; telem->tau_wheel_drive_right_nm = R.tau_wheel_drive;
    telem->tau_rolling_left_nm = L.tau_rolling; telem->tau_rolling_right_nm = R.tau_rolling;
    telem->tau_bearing_left_nm = L.tau_bearing; telem->tau_bearing_right_nm = R.tau_bearing;
    telem->tau_ground_left_nm = L.tau_ground; telem->tau_ground_right_nm = R.tau_ground;
    telem->tau_slip_loss_left_nm = L.tau_slip_loss; telem->tau_slip_loss_right_nm = R.tau_slip_loss;
    telem->force_longitudinal_command_left_n = L.f_cmd; telem->force_longitudinal_command_right_n = R.f_cmd;
    telem->force_longitudinal_ground_left_n = L.f_ground; telem->force_longitudinal_ground_right_n = R.f_ground;
    telem->force_longitudinal_max_left_n = L.f_max; telem->force_longitudinal_max_right_n = R.f_max;
    telem->force_longitudinal_saturation_left = L.saturation; telem->force_longitudinal_saturation_right = R.saturation;
    telem->lambda_long_left = L.lambda_long; telem->lambda_long_right = R.lambda_long;
    telem->lateral_accel_mm_s2 = lateral_accel_mps2 * 1000.0;
    telem->lateral_force_total_n = f_lat_total; telem->lateral_force_left_n = L.lateral_force; telem->lateral_force_right_n = R.lateral_force;
    telem->lateral_slip_left = L.lateral_slip; telem->lateral_slip_right = R.lateral_slip;
    telem->friction_usage_left = L.friction_usage; telem->friction_usage_right = R.friction_usage;
    telem->combined_friction_limit_left_n = L.combined_limit; telem->combined_friction_limit_right_n = R.combined_limit;
    telem->slip_ratio_left = L.slip; telem->slip_ratio_right = R.slip;
    telem->wheel_left_surface_speed_mm_s = L.surface_speed_mm_s; telem->wheel_right_surface_speed_mm_s = R.surface_speed_mm_s;
    telem->ground_left_speed_mm_s = L.ground_speed_mm_s; telem->ground_right_speed_mm_s = R.ground_speed_mm_s;
    telem->j_eq_left_kgm2 = L.j_eq; telem->j_eq_right_kgm2 = R.j_eq;
    telem->battery_power_w = battery_power;
    telem->copper_loss_left_w = L.copper_loss; telem->copper_loss_right_w = R.copper_loss;
    telem->driver_loss_left_w = L.driver_loss; telem->driver_loss_right_w = R.driver_loss;
    telem->battery_internal_loss_w = internal_loss; telem->wiring_loss_w = wiring_loss;
    telem->mechanical_friction_loss_left_w = L.mech_friction_loss; telem->mechanical_friction_loss_right_w = R.mech_friction_loss;
    telem->rolling_resistance_loss_w = rolling_loss;
    telem->tire_slip_loss_left_w = L.tire_slip_loss; telem->tire_slip_loss_right_w = R.tire_slip_loss;
    telem->brake_dissipated_power_w = brake_loss;
    telem->kinetic_power_delta_w = (total_ke - prev_ke) / fmax(1e-12, dt);
    telem->kinetic_energy_linear_j = ke_linear;
    telem->kinetic_energy_angular_j = ke_angular;
    telem->kinetic_energy_wheels_j = ke_wheels;
    telem->total_kinetic_energy_j = total_ke;
    telem->total_loss_energy_j = total_losses;
    telem->energy_balance_error_j = balance;
    telem->energy_balance_error_percent = (fabs(st->battery_energy_j) > 1e-12) ? 100.0 * balance / st->battery_energy_j : 0.0;
    telem->step_executed_in_c = 1;
    return 0;
}
