#include "kepler.h"
#include "math.h"
#include "raymath.h"
#include <X11/X.h>
#include <math.h>
#include "constants.h"
#include "../utils/logger.h"
#include "assert.h"

float mean_anom(float mean_anomaly_at_epoch, float time, float time_at_epoch, float orbital_period) {
    // First calculate mean motion
    // Given by n = (2 * PI) / T
    float n = (2 * PI) / orbital_period;
    // Calculate mean anomaly given time + mean anomaly at epoch + time elapsed since
    // Given by M = M_naught + n ( t - t_naught);
    float M = mean_anomaly_at_epoch + n * (time - time_at_epoch);
    
    return M;
}

// Function to compute the mean anomaly from the hyperbolic anomaly
float hyperbolic_anom_to_mean_anom(float hyperbolic_anom, float eccentricity) {
    // Mean anomaly M = e * sinh(H) - H
    float M = eccentricity * sinh(hyperbolic_anom) - hyperbolic_anom;

    return M;
}

// Function to compute hyperbolic anomaly from true anomaly
float true_anom_to_hyperbolic_anom(float true_anomaly, float eccentricity) {
    float nu = true_anomaly * M_PI / 180.0;

    float tan_nu_2 = tan(nu / 2.0);

    float sqrt_term = sqrt((eccentricity + 1) / (eccentricity - 1));

    float tanh_H_2 = tan_nu_2 / sqrt_term;

    float H_2 = atanh(tanh_H_2);

    return 2.0 * H_2;
}

float true_anom_to_mean_anom(float true_anomaly, float eccentricity) {
    float E = true_anom_to_ecc_anom(true_anomaly, eccentricity);
    return ecc_anom_to_mean_anom(eccentricity, E);
}

float true_anom_to_ecc_anom(float true_anomaly, float eccentricity) {
    // Calculate the factor sqrt((1 - e) / (1 + e))
    float factor = sqrt((1 - eccentricity) / (1 + eccentricity));
    
    // Calculate tan(E / 2)
    float tanE_half = factor * tan(true_anomaly / 2);
    
    // Calculate the eccentric anomaly E
    float E = 2 * atan(tanE_half);
    
    return E;
}

float ecc_anom_to_true_anom(float eccentricity, float eccentric_anomaly) {
    float numerator = 1 + eccentricity;
    float denomenator = 1 - eccentricity;
    
    float true_anomaly = 2 * atan(sqrt(numerator/denomenator) * tan(eccentric_anomaly/2.0));

    return true_anomaly;
}

float ecc_anom_to_mean_anom(float eccentricity, float eccentric_anomaly) {
    return eccentric_anomaly - eccentricity * sin(eccentric_anomaly);
}

float solve_kepler_eq_ellipse(float eccentricity, float mean_anomaly, int max_iters) {
    float eta = 1e-50;
    float e_naught;

    // Choose initial guesses for eccentric anomaly
    if ((-1 * PI < mean_anomaly && mean_anomaly < 0) || (mean_anomaly > PI)) {
        e_naught = mean_anomaly - eccentricity;
    } else {
        e_naught = mean_anomaly + eccentricity;
    }

    float e_n = e_naught;
    float delta = eta + 1;
    int count = 0;
    float e_np1 = 0;

    // Iterate until we reach convergence or max out our iters
    while (delta > eta) {
        e_np1 = e_n + 
            (mean_anomaly - e_n + eccentricity * sin(e_n)) 
            / 
            (1. - eccentricity * cos(e_n));
        delta = fabsf(e_np1 - e_n);
        e_n = e_np1;
        count++;

        if (count > max_iters) {
            //Warn("Failed to converge for keplers eq after %d iters\n",max_iters);
            break;
        }
    }


    return e_np1;
}


float kepler_H_newt(float e, float M) {
    float error = 1e-8;

    float F = M;

    float ratio = 1.0;

    while (fabs(ratio) > error) {
        ratio = (e * sin(F) - F - M)/(e*cosh(F) - 1);
        F = F - ratio;
    }

    return F;
}

float stump_c(float z) {
    if (fabs(z) < 1e-8) return 0.5;
    else if (z > 0) return (1 - cos(sqrt(z))) / z;
    else return (cosh(sqrt(-z)) - 1) / (-z);
}

float stump_s(float z) {
    if (fabs(z) < 1e-8) return 1.0 / 6.0;
    else if (z > 0) return (sqrt(z) - sin(sqrt(z))) / (sqrt(z * z * z));
    else return (sinh(sqrt(-z)) - sqrt(-z)) / sqrt((-z) * (-z) * (-z));
}


float clampf(float x, float min, float max) {
    if (x < min) {
        return min;
    } else if (x > max) {
        return max;
    } else {
        return x;
    }   
}

float calculate_sphere_of_influence_r(float a, float mass_of_parent, float mass_of_grandparent) {
    return a * powf(mass_of_parent/mass_of_grandparent,2.0/5.0);
}

TimeOfPassage compute_time_until(OrbitalElements oe, float desired_true_anomaly, float t) {
    // First check orbit type
    if (oe.eccentricity < 1.0) {
        // t-M/n = t_naught !!
        float M = true_anom_to_mean_anom(desired_true_anomaly, oe.eccentricity);
        return (TimeOfPassage) {
            .current_time = t,
            .time_at_point = (oe.mean_anomaly)/oe.mean_motion,
        };
    } else if (oe.eccentricity == 1.0) { // Parabolic orbits
        // D is helper variable in barkers eq
        float D = tan(desired_true_anomaly/2);

        // Solve barkers equation 
        // Time until periapsis (Δt) in a parabolic orbit:
        // Δt = t0 - t = -(1/2) * sqrt(p^3 / (2 * μ)) * (D + (D^3 / 3))
        // where D = tan(θ / 2) and p is the semi-latus rectum.
        float duration_until_periapsis = -0.5 * sqrt(pow(oe.semilatus_rectum,3)/(2*oe.grav_param)) * (D + pow(D,3)/3);

        return (TimeOfPassage) {
            .time_at_point = t + duration_until_periapsis,
            .current_time = t,
            .duration_until_point = duration_until_periapsis,
        };
    } else { // Hyperbolic orbits
        // Time until periapsis (Δt) in a hyperbolic orbit:
        // Δt = t0 - t = -(sqrt(μ) / (-a)^(3/2)) * (e * sinh(H) - H)

        // Compute the desired true anomaly at the sphere of influence
        float hyperbolic_anomaly = true_anom_to_hyperbolic_anom(desired_true_anomaly, oe.eccentricity);
        float mean_anomaly_at_sof = hyperbolic_anom_to_mean_anom(hyperbolic_anomaly, oe.eccentricity);

        float time_till_sof = ((mean_anomaly_at_sof - oe.mean_anomaly) / oe.mean_motion) + t;

        return (TimeOfPassage) {
            .time_at_point = t+time_till_sof,
            .current_time = t,
            .duration_until_point = time_till_sof,
        };
    }
}

float distance_sphere_coords(float e, float a, float E) {
    return a * (1. - e * cos(E));
}

OrbitalElements orb_elems_from_rv(PhysicalState rv, float mean_anomaly_at_epoch, float time_at_epoch) {
    float eps = 1.e-10;

    float grav_param = G * rv.mass_of_parent;

    // R,V position velocity vectors
    Vector3 R = rv.r;
    Vector3 V = rv.v;

    // r,v magnitudes of R and V
    float r = Vector3Length(R); 
    float v = Vector3Length(V);

    // R dot V / r
    float vr = Vector3DotProduct(R,V)/r;

    // Angular momentum vector
    Vector3 H = Vector3CrossProduct(R,V);

    // Angular momentum
    float h = Vector3Length(H);

    // inclination
    float i = acos(H.z/h);

    Vector3 z_ident = { 0, 0, 1 };

    // Only cross with Z component of H
    Vector3 N = Vector3CrossProduct(z_ident,H);

    float n = Vector3Length(N);

    float Ra;

    if (n == 0) {
        Ra = 0.0;
    } else {
        Ra = acos(N.x/n);
        if (N.y < 0.0) {
            Ra = 2*PI - Ra;
        }
    }

    // Eccentricity Vector
    float term1 = (v*v - grav_param/r);

    float inv_μ = 1.0f / grav_param;

    Vector3 E = Vector3Scale((Vector3Subtract(Vector3Scale(R,term1),Vector3Scale(V,r*vr))),inv_μ);

    // Orbital eccentricity = Vector3Length(E);
    float e = Vector3Length(E);

    float w;

    // Compute argument of periapsis
    if (n == 0) {
        w = 0;
    } else {
        if (e > eps) {
            w = acos(Vector3DotProduct(N,E)/n/e);
            if (E.z < 0) {
                w = 2 * PI - w;
            }
        } else {
            w = 0;
        }
    }

    float Ta;

    // Compute true anomaly
    if (e > eps) {
        Ta = acos(Vector3DotProduct(E,R)/e/r);

        if (vr < 0) {
            Ta = 2 * PI - Ta;
        }
    } else {
        Vector3 cp = Vector3CrossProduct(N,R);

        if (cp.z >= 0) {
            Ta = acos(Vector3DotProduct(N,R)/n/r);
        } else {
            Ta = 2 * PI - acos(Vector3DotProduct(N,R)/n/r);
        }
    }

    // Semi major axis
    float a = h * h / grav_param / (1 - e * e);
    
    // Compute eccentric anomaly + mean anomaly using helper functions

    float Ha, Ma, Ea;

    if (e >= 1.0) {
        Ha = true_anom_to_hyperbolic_anom(Ta, e);
        Ma = hyperbolic_anom_to_mean_anom(Ha, e);
    } else {
        Ea = true_anom_to_ecc_anom(Ta, e);
        Ma = ecc_anom_to_mean_anom(Ea, e);
    }
    float T = 2.0 * PI * sqrt(pow(a, 3) / grav_param); 

    // Return elements
    OrbitalElements elems = { 
        .semimajor_axis = a,
        .eccentricity = e,
        .eccentric_anomaly = Ea,
        .hyperbolic_anomaly = Ha,
        .mean_anomaly = Ma,
        .mean_anomaly_at_epoch = mean_anomaly_at_epoch,
        .time_at_epoch = time_at_epoch,
        .inclination = i,
        .long_of_asc_node = Ra,
        .grav_param = grav_param,
        .true_anomaly = Ta,
        .arg_of_periapsis = w,
        .period = T,
        .ang_momentum = h,
        .ang_momentum_vec = H,
        .semilatus_rectum = h * h / grav_param,
        .mean_motion = sqrt(grav_param/(powf(fabs(a),3.0))),
        .mass_of_parent = rv.mass_of_parent,
        .mass_of_grandparent = rv.mass_of_grandparent,
    };

    return elems;
}

void print_orbital_elements(OrbitalElements elems) {
    Debug("------\n");
    Debug("semi-major axis = %f\n",elems.semimajor_axis);
    Debug("eccentricity = %f\n",elems.eccentricity);
    Debug("orbital period (s) = %f\n",elems.period);
    Debug("grav param = %f\n",elems.grav_param);
    Debug("mean anomaly = %f degrees\n",elems.mean_anomaly * RAD2DEG);
    Debug("eccentric anomaly = %f degrees\n",elems.eccentric_anomaly * RAD2DEG);
    Debug("hyperbolic anomaly = %f degrees\n",elems.hyperbolic_anomaly * RAD2DEG);
    Debug("true anomaly = %f degrees\n",elems.true_anomaly * RAD2DEG);
    Debug("inclination = %f degrees\n",elems.inclination * RAD2DEG);
    Debug("arg of periapsis = %f degrees\n", elems.arg_of_periapsis * RAD2DEG);
    Debug("long of asc node = %f degrees\n", elems.long_of_asc_node * RAD2DEG);
    Debug("angular momentum h = %f\n", elems.ang_momentum * RAD2DEG);
    Debug("-------\n");
}

void print_physical_state(PhysicalState rv) {
    Debug("------\n");
    Debug("R = (%.3f,%.3f,%.3f)\n",rv.r.x,rv.r.y,rv.r.z);
    Debug("V = (%.3f,%.3f,%.3f)\n",rv.v.x,rv.v.y,rv.v.z);
    Debug("-------\n");
}

PhysicalState rv_from_orb_elems(OrbitalElements elems) {
    Vector3 i_hat = (Vector3){1.0,0.0,0.0};
    Vector3 j_hat = (Vector3){0.0,1.0,0.0};
    
    // rp = (hˆ2/mu) * (1/(1 + e*cos(TA))) * (cos(TA)*[1;0;0] + sin(TA)*[0;1;0]);
    //       rp_term_a    rp_term_b                rp_term_c             rp_term_d

    float rp_term_a = ((elems.ang_momentum * elems.ang_momentum)/elems.grav_param);
    float rp_term_b = 1/(1 + elems.eccentricity*cos(elems.true_anomaly));
    Vector3 rp_term_c = Vector3Scale(i_hat,cos(elems.true_anomaly));
    Vector3 rp_term_d = Vector3Scale(j_hat,sin(elems.true_anomaly));
    Vector3 c_plus_d = Vector3Add(rp_term_c, rp_term_d);

    Vector3 rp = Vector3Scale(c_plus_d, rp_term_a * rp_term_b);

    // vp = (mu/h) * (-sin(TA)*[1;0;0] + (e + cos(TA))*[0;1;0]);
    //        ^             ^                  ^
    //    vp_term_a     vp_term_b           vp_term_c
    float vp_term_a = (elems.grav_param/elems.ang_momentum);
    float vp_term_b = (-sin(elems.true_anomaly));
    float vp_term_c = (elems.eccentricity + cos(elems.true_anomaly));

    Vector3 vp = Vector3Scale(Vector3Add(Vector3Scale(i_hat,vp_term_b),Vector3Scale(j_hat,vp_term_c)),vp_term_a);

    // Explicit Vector3 -> Vector2 code
    Vector2 vp_pq = (Vector2){.x=vp.x,.y=vp.y};
    Vector2 rp_pq = (Vector2){.x=rp.x,.y=rp.y};

    return (PhysicalState){
        .r = perifocal_coords_to_inertial_coords(rp_pq, elems.long_of_asc_node, elems.arg_of_periapsis, elems.inclination),
        .v = perifocal_coords_to_inertial_coords(vp_pq, elems.long_of_asc_node,elems.arg_of_periapsis,elems.inclination)
    };
}

Vector3 vector_from_physical_to_world(Vector3 vec) {
    Vector3 transformed = {vec.x * KM_TO_RENDER_UNITS, vec.z * KM_TO_RENDER_UNITS, vec.y * KM_TO_RENDER_UNITS };

    return transformed;
}

Vector3 vector_from_world_to_physical(Vector3 vec) {
    Vector3 transformed = {vec.x * RENDER_UNITS_TO_KM, vec.z * RENDER_UNITS_TO_KM, vec.y * RENDER_UNITS_TO_KM };

    return transformed;
}

Vector2 solve_kepler_ellipse_perifocal(OrbitalElements elems, float M_naught, float t_naught, float t) {
    float mean_anomaly = mean_anom(M_naught,t,t_naught, elems.period);

    // NOTE: If your mean anomaly goes higher than 2 PI
    // You will fail to converge when you solve keplers eq
    // for certain values of mean_anomaly
    if (mean_anomaly > 2 * PI) {
        mean_anomaly = mean_anomaly - 2 * PI;
    }

    // Solve keplers eq MA -> E -> TA -> DIST -> (x,y,z)
    float eccentric_anomaly = solve_kepler_eq_ellipse(elems.eccentricity, mean_anomaly, 50);
    float true_anomaly = ecc_anom_to_true_anom(elems.eccentricity, eccentric_anomaly);
    float r = distance_sphere_coords(elems.eccentricity,elems.semimajor_axis,eccentric_anomaly);

    // Spherical coords -> carteesian
    float x = r * cos(true_anomaly);
    float y = r * sin(true_anomaly);

    // Return our handy Vector2
    return (Vector2){.x=x,.y=y};
}

float solve_universal_anomaly(float dt, float r0, float vr0, float a_inv, float grav_param) {
    float error = 1e-8;  // Tolerance
    int max_iters = 150;  // Maximum iterations

    // Initial guess for x
    float x = sqrt(grav_param) * dt / r0;

    float lambda = 1e-3;  // Levenberg-Marquardt damping parameter
    float nu = 2.0;       // Multiplicative factor for lambda adjustment

    int n = 0;
    float ratio = 1.0;
    float C, S, F, dFdx = 0.0;

    while (fabs(ratio) > error && n <= max_iters) {
        C = stump_c(a_inv * x * x);
        S = stump_s(a_inv * x * x);

        F = ((r0 * vr0) / sqrt(grav_param)) * x * x * C + (1.0 - a_inv * r0) * x * x * x * S + r0 * x - sqrt(grav_param) * dt;

        dFdx = (r0 * vr0 / sqrt(grav_param)) * x * (1.0 - a_inv * x * x * S) + (1.0 - a_inv * r0) * x * x * C + r0;

        // Levenberg-Marquardt step size control
        float dFdx_mod = dFdx + lambda * dFdx;  // Modify gradient with damping
        ratio = F / dFdx_mod;  // Update ratio

        // Check if the new guess is better
        float x_new = x - ratio;
        float F_new = ((r0 * vr0) / sqrt(grav_param)) * x_new * x_new * C + (1.0 - a_inv * r0) * x_new * x_new * x_new * S + r0 * x_new - sqrt(grav_param) * dt;

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


LagrangeCoefs compute_lagrange_f_g(float univ_anomaly, float t, float r0, float a_inv, float grav_param) {
    float z = a_inv * univ_anomaly * univ_anomaly;

    float f = 1 - (univ_anomaly*univ_anomaly/r0)*stump_c(z);
    float g = t - (1.0/sqrt(grav_param))*univ_anomaly*univ_anomaly*univ_anomaly*stump_s(z);

    return (LagrangeCoefs){
        .f = f,
        .g = g,
    };
}

LagrangeTimeDerivs compute_lagrange_fdot_gdot(float univ_anomaly, float r, float r0, float a_inv, float grav_param) {
    float z = a_inv * univ_anomaly * univ_anomaly;

    // sqrt(mu)/r/ro*(z*stumpS(z) - 1)*x
    float f_dot = (sqrt(grav_param)/(r*r0)) * ((z*stump_s(z) - 1)*univ_anomaly);
    // 1 - xˆ2/r*stumpC(z);
    float g_dot = 1 - (univ_anomaly*univ_anomaly/r)*stump_c(z);

    return (LagrangeTimeDerivs){
        .f_dot = f_dot,
        .g_dot = g_dot,
    };
}

PhysicalState rv_from_r0v0(PhysicalState rv, float t) {
    float grav_param = G * rv.mass_of_parent;

    float r0 = Vector3Length(rv.r);
    float v0 = Vector3Length(rv.v);

    float vr0 = Vector3DotProduct(rv.r, rv.v) / r0;

    // %...Reciprocal of the semimajor axis (from the energy equation):
    float alpha = 2/r0 - v0*v0/grav_param;

    // Compute universal anomaly
    float x = solve_universal_anomaly(t, r0, vr0, alpha, grav_param);

    // Grab our lagrange coefficients here
    LagrangeCoefs fg = compute_lagrange_f_g(x, t, r0, alpha, grav_param);

    // Final position vector
    Vector3 R = Vector3Add(Vector3Scale(rv.r,fg.f), Vector3Scale(rv.v,fg.g));

    // Compute length of R
    float r = Vector3Length(R);

    LagrangeTimeDerivs fdotgdot = compute_lagrange_fdot_gdot(x, r, r0, alpha, grav_param);

    Vector3 V = Vector3Add(Vector3Scale(rv.r,fdotgdot.f_dot), Vector3Scale(rv.v,fdotgdot.g_dot));

    return (PhysicalState){
        .r = R,
        .v = V,
        .mass_of_parent = rv.mass_of_parent,
        .mass_of_grandparent = rv.mass_of_grandparent,
    };
}

Vector3 perifocal_coords_to_inertial_coords(Vector2 pq,float long_of_asc_node,float arg_of_periapsis, float inclination) {
    // Perform a single combined linear transformation on these perifocal 
    // coords to convert to inertial 3d space
    // cos Ω cos ω − sin Ω sin ω cos i 
    float i_1_1 = cos(long_of_asc_node) * cos(arg_of_periapsis) - sin(long_of_asc_node) * sin(arg_of_periapsis) * cos(inclination);
    // − cos Ω sin ω − sin Ω cos ω cos i
    float i_1_2  = - cos(long_of_asc_node) * sin(arg_of_periapsis) - sin(long_of_asc_node) * cos(arg_of_periapsis) * cos(inclination);
    // sin Ω sin i
    float i_1_3 = sin(long_of_asc_node) * sin(inclination);

    // sin Ω cos ω + cos Ω sin ω cos i
    float i_2_1 = sin(long_of_asc_node) * cos(arg_of_periapsis) + cos(long_of_asc_node) * sin(arg_of_periapsis) * cos(inclination);
    // − sin Ω sin ω + cos Ω cos ω cos i
    float i_2_2 = - sin(long_of_asc_node) * sin(arg_of_periapsis) + cos(long_of_asc_node) * cos(arg_of_periapsis) * cos(inclination);
    // − cos Ω sin i
    float i_2_3 = - cos(long_of_asc_node) * sin(inclination);

    // sin ω sin i
    float i_3_1 = sin(arg_of_periapsis) * sin(inclination);
    // cos ω sin i
    float i_3_2 = cos(arg_of_periapsis) * sin(inclination);
    // cos i
    float i_3_3 = cos(inclination);

    // Create 3x3 matrix
    Matrix mat = {
        i_1_1, i_1_2, i_1_3, 0,
        i_2_1, i_2_2, i_2_3, 0,
        i_3_1, i_3_2, i_3_3, 0,
        0,     0,     0,     1,
    };

    // Transform the point and return
    return Vector3Transform((Vector3){.x = pq.x, .y = pq.y, .z = 0}, mat);
}

Vector3 solve_kepler_ellipse_inertial(OrbitalElements elems, float M_naught, float t_naught, float t) {
    Vector2 perifocal_coords = solve_kepler_ellipse_perifocal(elems, M_naught, t_naught, t);
    return perifocal_coords_to_inertial_coords(perifocal_coords, elems.long_of_asc_node, elems.arg_of_periapsis, elems.inclination);
} 
