#include "kepler.h"
#include "corbit_math.h"
#include <math.h>
#include <stdio.h>
#include "constants.h"
#include "../utils/logger.h"
#include "assert.h"

double wrap_angle(double angle) {
    if (angle < 0) {
        return 2 * D_PI + angle;
    } else if (angle > 2 * D_PI) {
        return angle - 2 * D_PI;
    } else {
        return angle;
    }
}

double mean_anom(double mean_anomaly_at_epoch, double time, double time_at_epoch, double orbital_period) {
    // First calculate mean motion
    // Given by n = (2 * D_PI) / T
    double n = (2 * D_PI) / orbital_period;
    // Calculate mean anomaly given time + mean anomaly at epoch + time elapsed since
    // Given by M = M_naught + n ( t - t_naught);
    double M = mean_anomaly_at_epoch + n * (time - time_at_epoch);
    
    return M;
}

// Function to compute the mean anomaly from the hyperbolic anomaly
double hyperbolic_anom_to_mean_anom(double hyperbolic_anom, double eccentricity) {
    // Mean anomaly M = e * sinh(H) - H
    double M = eccentricity * sinh(hyperbolic_anom) - hyperbolic_anom;

    return M;
}

// Function to compute hyperbolic anomaly from true anomaly
double true_anom_to_hyperbolic_anom(double true_anomaly, double eccentricity) {
    double nu = true_anomaly * D_PI / 180.0;

    double tan_nu_2 = tan(nu / 2.0);

    double sqrt_term = sqrt((eccentricity + 1) / (eccentricity - 1));

    double tanh_H_2 = tan_nu_2 / sqrt_term;

    double H_2 = atanh(tanh_H_2);

    return 2.0 * H_2;
}

double true_anom_to_mean_anom(double true_anomaly, double eccentricity) {
    double E = true_anom_to_ecc_anom(true_anomaly, eccentricity);
    return ecc_anom_to_mean_anom(eccentricity, E);
}

double true_anom_to_ecc_anom(double true_anomaly, double eccentricity) {
    // Calculate the factor sqrt((1 - e) / (1 + e))
    double factor = sqrt((1 - eccentricity) / (1 + eccentricity));
    
    // Calculate tan(E / 2)
    double tanE_half = factor * tan(true_anomaly / 2);
    
    // Calculate the eccentric anomaly E
    double E = 2 * atan(tanE_half);
    
    return E;
}

double ecc_anom_to_true_anom(double eccentricity, double eccentric_anomaly) {
    double numerator = 1 + eccentricity;
    double denomenator = 1 - eccentricity;
    
    double true_anomaly = 2 * atan(sqrt(numerator/denomenator) * tan(eccentric_anomaly/2.0));

    return true_anomaly;
}

double ecc_anom_to_mean_anom(double eccentricity, double eccentric_anomaly) {
    return eccentric_anomaly - eccentricity * sin(eccentric_anomaly);
}

double solve_kepler_eq_ellipse(double eccentricity, double mean_anomaly, int max_iters) {
    double eta = 1e-50;
    double e_naught;

    // Choose initial guesses for eccentric anomaly
    if ((-1 * D_PI < mean_anomaly && mean_anomaly < 0) || (mean_anomaly > D_PI)) {
        e_naught = mean_anomaly - eccentricity;
    } else {
        e_naught = mean_anomaly + eccentricity;
    }

    double e_n = e_naught;
    double delta = eta + 1;
    int count = 0;
    double e_np1 = 0;

    // Iterate until we reach convergence or max out our iters
    while (delta > eta) {
        e_np1 = e_n + 
            (mean_anomaly - e_n + eccentricity * sin(e_n)) 
            / 
            (1. - eccentricity * cos(e_n));
        delta = fabs(e_np1 - e_n);
        e_n = e_np1;
        count++;

        if (count > max_iters) {
            /* Warn("Failed to converge for keplers eq after %d iters\n",max_iters); */
            break;
        }
    }


    return e_np1;
}



double kepler_H_newt(double e, double M) {
    double error = 1e-8;

    double F = M;

    double ratio = 1.0;

    while (fabs(ratio) > error) {
        ratio = (e * sin(F) - F - M)/(e*cosh(F) - 1);
        F = F - ratio;
    }

    return F;
}

double stump_c(double z) {
    if (fabs(z) < 1e-8) return 0.5;
    else if (z > 0) return (1 - cos(sqrt(z))) / z;
    else return (cosh(sqrt(-z)) - 1) / (-z);
}

double stump_s(double z) {
    if (fabs(z) < 1e-8) return 1.0 / 6.0;
    else if (z > 0) return (sqrt(z) - sin(sqrt(z))) / (sqrt(z * z * z));
    else return (sinh(sqrt(-z)) - sqrt(-z)) / sqrt((-z) * (-z) * (-z));
}

double clampd(double x, double min, double max) {
    if (x < min) {
        return min;
    } else if (x > max) {
        return max;
    } else {
        return x;
    }   
}

double calculate_sphere_of_influence_r(double a, double mass_of_parent, double mass_of_grandparent) {
    double soi = a * powf(mass_of_parent/(3.0*mass_of_grandparent),1.0/3.0);
    return soi;
}

TimeOfPassage compute_time_until(ClassicalOrbitalElements oe, double grav_param, double desired_true_anomaly, double t) {
    // First check orbit type
    if (oe.eccentricity < 1.0) {
        // t-M/n = t_naught !!
        double M = true_anom_to_mean_anom(desired_true_anomaly, oe.eccentricity);
        return (TimeOfPassage) {
            .current_time = t,
            .time_at_point = (oe.mean_anomaly)/oe.mean_motion,
        };
    } else if (oe.eccentricity == 1.0) { // Parabolic orbits
        // D is helper variable in barkers eq
        double D = tan(desired_true_anomaly/2);

        // Solve barkers equation 
        // Time until periapsis (Δt) in a parabolic orbit:
        // Δt = t0 - t = -(1/2) * sqrt(p^3 / (2 * μ)) * (D + (D^3 / 3))
        // where D = tan(θ / 2) and p is the semi-latus rectum.
        double duration_until_periapsis = -0.5 * sqrt(pow(oe.semilatus_rectum,3)/(2*grav_param)) * (D + pow(D,3)/3);

        return (TimeOfPassage) {
            .time_at_point = t + duration_until_periapsis,
            .current_time = t,
            .duration_until_point = duration_until_periapsis,
        };
    } else { // Hyperbolic orbits
        // Time until periapsis (Δt) in a hyperbolic orbit:
        // Δt = t0 - t = -(sqrt(μ) / (-a)^(3/2)) * (e * sinh(H) - H)

        // Compute the desired true anomaly at the sphere of influence
        double hyperbolic_anomaly = true_anom_to_hyperbolic_anom(desired_true_anomaly, oe.eccentricity);
        double mean_anomaly_at_sof = hyperbolic_anom_to_mean_anom(hyperbolic_anomaly, oe.eccentricity);

        double time_till_sof = ((mean_anomaly_at_sof - oe.mean_anomaly) / oe.mean_motion) + t;

        return (TimeOfPassage) {
            .time_at_point = t+time_till_sof,
            .current_time = t,
            .duration_until_point = time_till_sof,
        };
    }
}

double distance_sphere_coords(double e, double a, double E) {
    return a * (1. - e * cos(E));
}

ClassicalOrbitalElements rv_to_classical_elements(PhysicalState rv, double grav_param) {
   double eps = 1.e-10;

   // Position and velocity vectors
   DVector3 R = rv.r;
   DVector3 V = rv.v;
   double r = DVector3Length(R);
   double v = DVector3Length(V);
   double r_dot_v = DVector3DotProduct(R, V);
   
   // Angular momentum vector H = R × V
   DVector3 H = DVector3CrossProduct(R, V);
   double h = DVector3Length(H);
   DVector3 h_hat = DVector3Scale(H, 1.0/h);

   // Inclination is angle between H and Z axis
   double i = acos(clampd(h_hat.z, -1.0, 1.0));

   // Node vector N = K × H
   DVector3 N = DVector3CrossProduct((DVector3){0, 0, 1}, H);
   double n = DVector3Length(N);
   DVector3 n_hat = n > eps ? DVector3Scale(N, 1.0/n) : (DVector3){1, 0, 0};

   // Eccentricity vector
   double term1 = (v * v - grav_param / r);
   DVector3 E = DVector3Scale(
       DVector3Subtract(
           DVector3Scale(R, term1),
           DVector3Scale(V, r_dot_v)
       ),
       1.0/grav_param
   );
   double e = DVector3Length(E);
   DVector3 e_hat = e > eps ? DVector3Scale(E, 1.0/e) : n_hat;

   // For nearly circular orbits (e < threshold), semi-major axis ≈ r
   // Otherwise calculate from energy equation
   double a;
   if (e < 1e-5) {
       a = r;
   } else {
       double specific_energy = 0.5 * v*v - grav_param/r;
       a = -grav_param/(2.0 * specific_energy);
   }

   // RAAN - keep in [-π, π)
   double Ra = (n > eps) ? atan2(N.y, N.x) : 0.0;
   if (Ra > D_PI) Ra -= 2*D_PI;

   // Argument of periapsis - keep in [-π, π)
   double w = 0.0;
   if (e > eps) {
       if (n > eps) {
           // Non-equatorial orbit: Use node vector (N)
           double cos_w = DVector3DotProduct(N, E) / (n * e);
           cos_w = fmax(-1.0, fmin(1.0, cos_w));  // Clamp to [-1, 1]
           w = acos(cos_w);
           if (E.z < 0) w = 2 * M_PI - w;  // Correct quadrant
       } else {
           // Equatorial orbit: Use x-axis reference
           DVector3 x_ident = {1, 0, 0};  // Reference direction in the equatorial plane
           double cos_w = DVector3DotProduct(x_ident, E) / e;
           cos_w = fmax(-1.0, fmin(1.0, cos_w));  // Clamp to [-1, 1]
           w = acos(cos_w);
           if (E.y < 0) w = 2 * M_PI - w;  // Correct quadrant for equatorial plane
       }
   }

   // True anomaly - keep in [0, 2π)
   double Ta;
   if (e > eps) {
       double sin_Ta = DVector3DotProduct(DVector3CrossProduct(e_hat, DVector3Scale(R, 1.0/r)), h_hat);
       double cos_Ta = DVector3DotProduct(e_hat, DVector3Scale(R, 1.0/r));
       Ta = atan2(sin_Ta, cos_Ta);
   } else {
       double sin_Ta = DVector3DotProduct(DVector3CrossProduct(n_hat, DVector3Scale(R, 1.0/r)), h_hat);
       double cos_Ta = DVector3DotProduct(n_hat, DVector3Scale(R, 1.0/r));
       Ta = atan2(sin_Ta, cos_Ta);
   }
   if (Ta < 0.0) Ta += 2*D_PI;

   // Calculate remaining orbital elements
   double T = (e < 1.0) ? 2.0 * D_PI * sqrt(pow(a, 3)/grav_param) : 0.0;
   double Ea = 0.0, Ha = 0.0, Ma = 0.0;

   if (e >= 1.0) {
       Ha = true_anom_to_hyperbolic_anom(Ta, e);
       Ma = hyperbolic_anom_to_mean_anom(Ha, e);
   } else {
       Ea = true_anom_to_ecc_anom(Ta, e);
       Ma = ecc_anom_to_mean_anom(e, Ea);
   }
   Ma = fmod(Ma, 2*D_PI);
   if (Ma < 0) Ma += 2*D_PI;

   // Derived quantities
   double mean_motion = sqrt(grav_param/pow(fabs(a), 3.0));
   double periapsis = a*(1.0 - e);
   double apoapsis = a*(1.0 + e);
   double p = h*h/grav_param;  // semi-parameter

   return (ClassicalOrbitalElements){
       .semimajor_axis = a,
       .eccentricity = e,
       .eccentric_anomaly = Ea,
       .hyperbolic_anomaly = Ha,
       .mean_anomaly = wrap_angle(Ma),
       .inclination = wrap_angle(i),
       .long_of_asc_node = wrap_angle(Ra),
       .arg_of_periapsis = wrap_angle(w),
       .true_anomaly = wrap_angle(Ta),
       .period = T,
       .ang_momentum = h,
       .ang_momentum_vec = h_hat,
       .semilatus_rectum = p,
       .mean_motion = mean_motion,
       .periapsis_distance = periapsis,
       .apoapsis_distance = apoapsis
   };
}

void print_orbital_elements(char* body_name, ClassicalOrbitalElements elems) {
    Debug("--- %s ---\n",body_name);
    Debug("semi-major axis = %f\n",elems.semimajor_axis);
    Debug("eccentricity = %f\n",elems.eccentricity);
    Debug("orbital period (s) = %f\n",elems.period);
    Debug("mean anomaly = %f degrees\n",elems.mean_anomaly * D_RAD2DEG);
    Debug("eccentric anomaly = %f degrees\n",elems.eccentric_anomaly * D_RAD2DEG);
    Debug("hyperbolic anomaly = %f degrees\n",elems.hyperbolic_anomaly * D_RAD2DEG);
    Debug("true anomaly = %f degrees\n",elems.true_anomaly * D_RAD2DEG);
    Debug("inclination = %f degrees\n",elems.inclination * D_RAD2DEG);
    Debug("arg of periapsis = %f degrees\n", elems.arg_of_periapsis * D_RAD2DEG);
    Debug("long of asc node = %f degrees\n", elems.long_of_asc_node * D_RAD2DEG);
    Debug("angular momentum h = %f\n", elems.ang_momentum * D_RAD2DEG);
    Debug("-------\n");
}

void print_physical_state(PhysicalState rv) {
    Debug("------\n");
    Debug("R = (%.3f,%.3f,%.3f)\n",rv.r.x,rv.r.y,rv.r.z);
    Debug("V = (%.3f,%.3f,%.3f)\n",rv.v.x,rv.v.y,rv.v.z);
    Debug("-------\n");
}

PhysicalState classical_elements_to_rv(ClassicalOrbitalElements elems,double grav_param) {
    double mu = grav_param;
    double h = elems.ang_momentum;

    // Calculate the radius
    double r = (h * h / mu) / (1.0 + elems.eccentricity * cos(elems.true_anomaly));

    // Perifocal position vector
    DVector3 r_perifocal = {
        r * cos(elems.true_anomaly),
        r * sin(elems.true_anomaly),
        0.0
    };

    // Perifocal velocity components
    double v_r = (mu / h) * elems.eccentricity * sin(elems.true_anomaly);
    double v_theta = (mu / h) * (1.0 + elems.eccentricity * cos(elems.true_anomaly));

    // Perifocal velocity vector
    DVector3 v_perifocal = {
        v_r * cos(elems.true_anomaly) - v_theta * sin(elems.true_anomaly),
        v_r * sin(elems.true_anomaly) + v_theta * cos(elems.true_anomaly),
        0.0
    };

    // Transform perifocal to inertial coordinates
    DVector3 r_inertial = perifocal_coords_to_inertial_coords(
        r_perifocal,
        elems.long_of_asc_node,
        elems.arg_of_periapsis,
        elems.inclination
    );

    DVector3 v_inertial = perifocal_coords_to_inertial_coords(
        v_perifocal,
        elems.long_of_asc_node,
        elems.arg_of_periapsis,
        elems.inclination
    );

    // Return the physical state
    return (PhysicalState){
        .r = r_inertial,
        .v = v_inertial,
    };
}


DVector2 vector2_from_physical_to_world(DVector2 vec) {
    DVector2 transformed = {vec.x * KM_TO_RENDER_UNITS_2D, vec.y * KM_TO_RENDER_UNITS_2D};

    return transformed;
}

DVector2 vector2_from_world_to_physical(DVector2 vec) {
    DVector2 transformed = {vec.x * RENDER_UNITS_TO_KM_2D, vec.y * RENDER_UNITS_TO_KM_2D};

    return transformed;
}

DVector3 vector_from_physical_to_world(DVector3 vec) {
    DVector3 transformed = {vec.x * KM_TO_RENDER_UNITS, vec.z * KM_TO_RENDER_UNITS, vec.y * KM_TO_RENDER_UNITS };

    return transformed;
}

DVector3 vector_from_world_to_physical(DVector3 vec) {
    DVector3 transformed = {vec.x * RENDER_UNITS_TO_KM, vec.z * RENDER_UNITS_TO_KM, vec.y * RENDER_UNITS_TO_KM };

    return transformed;
}

DVector3 solve_kepler_ellipse_perifocal(ClassicalOrbitalElements elems, double M_naught, double t_naught, double t) {
    double mean_anomaly = mean_anom(M_naught,t,t_naught, elems.period);

    // NOTE: If your mean anomaly goes higher than 2 D_PI
    // You will fail to converge when you solve keplers eq
    // for certain values of mean_anomaly
    if (mean_anomaly > 2 * D_PI) {
        mean_anomaly = mean_anomaly - 2 * D_PI;
    }

    // Solve keplers eq MA -> E -> TA -> DIST -> (x,y,z)
    double eccentric_anomaly = solve_kepler_eq_ellipse(elems.eccentricity, mean_anomaly, 50);
    double true_anomaly = ecc_anom_to_true_anom(elems.eccentricity, eccentric_anomaly);
    double r = distance_sphere_coords(elems.eccentricity,elems.semimajor_axis,eccentric_anomaly);

    // Spherical coords -> carteesian
    double x = r * cos(true_anomaly);
    double y = r * sin(true_anomaly);

    // Return our handy DVector3
    return (DVector3){.x=x,.y=y,.z=0.0};
}

double solve_universal_anomaly(double dt, double r0, double vr0, double a_inv, double grav_param) {
    double error = 1e-8;  // Tolerance
    int max_iters = 250;  // Maximum iterations

    // Initial guess for x
    double x = sqrt(grav_param) * dt / r0;

    double lambda = 1e-3;  // Levenberg-Marquardt damping parameter
    double nu = 2.0;       // Multiplicative factor for lambda adjustment

    int n = 0;
    double ratio = 1.0;
    double C, S, F, dFdx = 0.0;

    while (fabs(ratio) > error && n <= max_iters) {
        C = stump_c(a_inv * x * x);
        S = stump_s(a_inv * x * x);

        F = ((r0 * vr0) / sqrt(grav_param)) * x * x * C + (1.0 - a_inv * r0) * x * x * x * S + r0 * x - sqrt(grav_param) * dt;

        dFdx = (r0 * vr0 / sqrt(grav_param)) * x * (1.0 - a_inv * x * x * S) + (1.0 - a_inv * r0) * x * x * C + r0;

        // Levenberg-Marquardt step size control
        double dFdx_mod = dFdx + lambda * dFdx;  // Modify gradient with damping
        ratio = F / dFdx_mod;  // Update ratio

        // Check if the new guess is better
        double x_new = x - ratio;
        double F_new = ((r0 * vr0) / sqrt(grav_param)) * x_new * x_new * C + (1.0 - a_inv * r0) * x_new * x_new * x_new * S + r0 * x_new - sqrt(grav_param) * dt;

        if (fabs(F_new) < fabs(F)) {
            // If improved, decrease lambda
            lambda /= nu;
            x = x_new;  // Accept new x
        } else {
            // If not, increase lambda
            lambda *= nu;
        }

        n++;
    }

    if (n > max_iters) {
        Warn("solve_universal_anomaly() reached max iterations without convergence, n = %i\n", n);
    }

    return x;
}


LagrangeCoefs compute_lagrange_f_g(double univ_anomaly, double t, double r0, double a_inv, double grav_param) {
    double z = a_inv * univ_anomaly * univ_anomaly;

    double f = 1 - (univ_anomaly*univ_anomaly/r0)*stump_c(z);
    double g = t - (1.0/sqrt(grav_param))*univ_anomaly*univ_anomaly*univ_anomaly*stump_s(z);

    return (LagrangeCoefs){
        .f = f,
        .g = g,
    };
}

LagrangeTimeDerivs compute_lagrange_fdot_gdot(double univ_anomaly, double r, double r0, double a_inv, double grav_param) {
    double z = a_inv * univ_anomaly * univ_anomaly;

    // sqrt(mu)/r/ro*(z*stumpS(z) - 1)*x
    double f_dot = (sqrt(grav_param)/(r*r0)) * ((z*stump_s(z) - 1)*univ_anomaly);
    // 1 - xˆ2/r*stumpC(z);
    double g_dot = 1 - (univ_anomaly*univ_anomaly/r)*stump_c(z);

    return (LagrangeTimeDerivs){
        .f_dot = f_dot,
        .g_dot = g_dot,
    };
}

PhysicalState rv_from_r0v0(PhysicalState rv, double grav_param, double t) {
    double r0 = DVector3Length(rv.r);
    double v0 = DVector3Length(rv.v);

    double vr0 = DVector3DotProduct(rv.r, rv.v) / r0;

    // %...Reciprocal of the semimajor axis (from the energy equation):
    double alpha = 2/r0 - v0*v0/grav_param;

    // Compute universal anomaly
    double x = solve_universal_anomaly(t, r0, vr0, alpha, grav_param);

    // Grab our lagrange coefficients here
    LagrangeCoefs fg = compute_lagrange_f_g(x, t, r0, alpha, grav_param);

    // Final position vector
    DVector3 R = DVector3Add(DVector3Scale(rv.r,fg.f), DVector3Scale(rv.v,fg.g));

    // Compute length of R
    double r = DVector3Length(R);

    // Compute time derivatives of Lagrange Coefs
    LagrangeTimeDerivs fdotgdot = compute_lagrange_fdot_gdot(x, r, r0, alpha, grav_param);

    // Final velocity vector
    DVector3 V = DVector3Add(DVector3Scale(rv.r,fdotgdot.f_dot), DVector3Scale(rv.v,fdotgdot.g_dot));

    return (PhysicalState){
        .r = R,
        .v = V,
        .mass = rv.mass,
    };
}

DVector3 perifocal_coords_to_inertial_coords(DVector3 pq, double long_of_asc_node, double arg_of_periapsis, double inclination) {
    // Calculate trigonometric values
    double cos_Omega = cos(long_of_asc_node); // cos(Ω)
    double sin_Omega = sin(long_of_asc_node); // sin(Ω)
    double cos_omega = cos(arg_of_periapsis); // cos(ω)
    double sin_omega = sin(arg_of_periapsis); // sin(ω)
    double cos_i = cos(inclination);          // cos(i)
    double sin_i = sin(inclination);          // sin(i)

    // Rotation matrix elements (from perifocal to inertial)
    double i_1_1 = cos_Omega * cos_omega - sin_Omega * sin_omega * cos_i;
    double i_1_2 = -cos_Omega * sin_omega - sin_Omega * cos_omega * cos_i;
    double i_1_3 = sin_Omega * sin_i;

    double i_2_1 = sin_Omega * cos_omega + cos_Omega * sin_omega * cos_i;
    double i_2_2 = -sin_Omega * sin_omega + cos_Omega * cos_omega * cos_i;
    double i_2_3 = -cos_Omega * sin_i;

    double i_3_1 = sin_omega * sin_i;
    double i_3_2 = cos_omega * sin_i;
    double i_3_3 = cos_i;

    // Perform the transformation
    double x_inertial = i_1_1 * pq.x + i_1_2 * pq.y + i_1_3 * pq.z;
    double y_inertial = i_2_1 * pq.x + i_2_2 * pq.y + i_2_3 * pq.z;
    double z_inertial = i_3_1 * pq.x + i_3_2 * pq.y + i_3_3 * pq.z;

    return (DVector3){.x = x_inertial, .y = y_inertial, .z = z_inertial};
}

DVector3 inertial_coords_to_perifocal_coords(DVector3 eci, double long_of_asc_node, double arg_of_periapsis, double inclination) {
    // Calculate trigonometric values
    double cos_Omega = cos(long_of_asc_node);
    double sin_Omega = sin(long_of_asc_node);
    double cos_omega = cos(arg_of_periapsis);
    double sin_omega = sin(arg_of_periapsis);
    double cos_i = cos(inclination);
    double sin_i = sin(inclination);

    // Transpose of the perifocal to inertial matrix
    double p_1_1 = cos_Omega * cos_omega - sin_Omega * sin_omega * cos_i;
    double p_1_2 = sin_Omega * cos_omega + cos_Omega * sin_omega * cos_i;
    double p_1_3 = sin_omega * sin_i;

    double p_2_1 = -cos_Omega * sin_omega - sin_Omega * cos_omega * cos_i;
    double p_2_2 = -sin_Omega * sin_omega + cos_Omega * cos_omega * cos_i;
    double p_2_3 = cos_omega * sin_i;

    double p_3_1 = sin_Omega * sin_i;
    double p_3_2 = -cos_Omega * sin_i;
    double p_3_3 = cos_i;

    // Perform the transformation
    double x_perifocal = p_1_1 * eci.x + p_1_2 * eci.y + p_1_3 * eci.z;
    double y_perifocal = p_2_1 * eci.x + p_2_2 * eci.y + p_2_3 * eci.z;
    double z_perifocal = p_3_1 * eci.x + p_3_2 * eci.y + p_3_3 * eci.z;

    return (DVector3){.x = x_perifocal, .y = y_perifocal, .z = z_perifocal};
}

DVector3 solve_kepler_ellipse_inertial(ClassicalOrbitalElements elems, double M_naught, double t_naught, double t) {
    DVector3 perifocal_coords = solve_kepler_ellipse_perifocal(elems, M_naught, t_naught, t);
    DVector3 result = perifocal_coords_to_inertial_coords(perifocal_coords, elems.long_of_asc_node, elems.arg_of_periapsis, elems.inclination);
    return result;
}

double periapsis_distance(ClassicalOrbitalElements oe) {
    return oe.semimajor_axis * (1.0 - oe.eccentricity);
}


OrbitalNodes compute_nodes(ClassicalOrbitalElements oe) {
    // True anomalies at nodes
    double nu_ascending = 0.0 - oe.arg_of_periapsis;       // Ascending node
    double nu_descending = M_PI - oe.arg_of_periapsis;    // Descending node

    // Distance at ascending and descending nodes
    double r_ascending = oe.semimajor_axis * (1.0 - oe.eccentricity * oe.eccentricity) /
                         (1.0 + oe.eccentricity * cos(nu_ascending));
    double r_descending = oe.semimajor_axis * (1.0 - oe.eccentricity * oe.eccentricity) /
                          (1.0 + oe.eccentricity * cos(nu_descending));

    // Perifocal coordinates at nodes
    DVector3 perifocal_ascending = {
        .x = r_ascending * cos(nu_ascending),
        .y = r_ascending * sin(nu_ascending),
        .z = 0.0
    };

    DVector3 perifocal_descending = {
        .x = r_descending * cos(nu_descending),
        .y = r_descending * sin(nu_descending),
        .z = 0.0
    };

    // Convert perifocal to inertial coordinates
    DVector3 ascending_node = perifocal_coords_to_inertial_coords(
        perifocal_ascending,
        oe.long_of_asc_node,
        oe.arg_of_periapsis,
        oe.inclination
    );

    DVector3 descending_node = perifocal_coords_to_inertial_coords(
        perifocal_descending,
        oe.long_of_asc_node,
        oe.arg_of_periapsis,
        oe.inclination
    );

    OrbitalNodes n = {
        ascending_node,
        descending_node,
    };

    return n;
}
