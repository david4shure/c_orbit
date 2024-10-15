#include "kepler.h"
#include "math.h"
#include "raymath.h"
#include <math.h>
#include "constants.h"
#include "../utils/logger.h"

float mean_anom(float mean_anomaly_at_epoch, float time, float time_at_epoch, float orbital_period) {
    // First calculate mean motion
    // Given by n = (2 * PI) / T
    float n = (2 * PI) / orbital_period;
    // Calculate mean anomaly given time + mean anomaly at epoch + time elapsed since
    // Given by M = M_naught + n ( t - t_naught);
    float M = mean_anomaly_at_epoch + n * (time - time_at_epoch);
    
    return M;
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
            Warn("Failed to converge for keplers eq after %d iters\n",max_iters);
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

// Stumpff function C(z)
float stump_c(float z) {
    if (z > 1e-6) {
        return (1.0 - cos(sqrt(z))) / z;
    } else if (z < -1e-6) {
        return (cosh(sqrt(-z)) - 1.0) / (-z);
    } else {
        return 0.5;
    }
}

// Stumpff function S(z)
float stump_s(float z) {
    if (z > 1e-6) {
        return (sqrt(z) - sin(sqrt(z))) / (sqrt(z * z * z));
    } else if (z < -1e-6) {
        return (sinh(sqrt(-z)) - sqrt(-z)) / sqrt((-z) * (-z) * (-z));
    } else {
        return 1.0 / 6.0;
    }
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

float distance_sphere_coords(float e, float a, float E) {
    return a * (1. - e * cos(E));
}

OrbitalElements orb_elems_from_rv(PhysicalState rv, float μ) {
    float eps = 1.e-10;

    // R,V position velocity vectors
    Vector3 R = rv.r;
    Vector3 V = rv.v;

    // r,v magnitudes of R and V
    float r = Vector3Length(R); 
    float v = Vector3Length(V);

    // R dot V / r
    float vr = Vector3DotProduct(R,V)/r;

    Vector3 H = Vector3CrossProduct(R,V);

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
    float term1 = (v*v - μ/r);

    float inv_μ = 1.0f / μ;

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
    float a = h * h / μ / (1 - e * e);
    
    // Compute eccentric anomaly + mean anomaly using helper functions
    float Ea = true_anom_to_ecc_anom(Ta, e);
    float Ma = ecc_anom_to_mean_anom(Ea, e);
    float T = 2.0 * PI * sqrt(pow(a, 3) / μ); 

    // Return elements
    OrbitalElements elems = { 
        .semimajor_axis = a,
        .eccentricity = e,
        .eccentric_anomaly = Ea,
        .mean_anomaly = Ma,
        .inclination = i,
        .long_of_asc_node = Ra,
        .grav_param = μ,
        .true_anomaly = Ta,
        .arg_of_periapsis = w,
        .period = T,
        .ang_momentum = h,
        .ang_momentum_vec = H,
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
    Debug("true anomaly = %f degrees\n",elems.true_anomaly * RAD2DEG);
    Debug("inclination = %f degrees\n",elems.inclination * RAD2DEG);
    Debug("arg of periapsis = %f degrees\n", elems.arg_of_periapsis * RAD2DEG);
    Debug("long of asc node = %f degrees\n", elems.long_of_asc_node * RAD2DEG);
    Debug("angular momentum h = %f\n", elems.ang_momentum * RAD2DEG);
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

float solve_universal_anomaly(float dt, float r0, float v0, float a, float grav_param) {
    // User is passing in real semi-major axis, we calculate inv_a
    float a_inv = 1/a;
    float error = 1e-10;
    int max_iters = 1000;

    // ... Starting value for x:
    float x = sqrt(grav_param) * fabs(a_inv) * dt;

    int n = 0;
    float ratio = 1.0;
    float C, S, F, dFdx = 0.0;

    while (fabs(ratio) > error && n <= max_iters) {
        // Calculate stumps and stumpc
        C = stump_c(a_inv*x*x);
        S = stump_s(a_inv*x*x);

        // Estimate F
        F = ((r0 * v0) / sqrt(grav_param)) * x * x * C + (1.0 - a_inv * r0) * x * x * x * S + r0 * x - sqrt(grav_param) * dt;
        // Estimate dF / dt
        dFdx = (r0 * v0 / sqrt(grav_param)) * x * (1.0 - a_inv * x * x * S) + (1.0 - a_inv * r0) * x * x * C + r0;

        // Update ratio
        ratio = F/dFdx;

        // Update x
        x = x - ratio;

        // Increment n
        n++;
    }

    if (n > max_iters) {
        Warn("solve_universal_anomaly() reached max iterations without convergance, n = %i\n",n);
    }

    return x;
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
