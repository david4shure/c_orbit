#include "kepler.h"
#include "math.h"
#include "raymath.h"
#include <math.h>
#include <stdio.h>
#include "constants.h"

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
            printf("Failed to converge after %d iters\n",max_iters);
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
    if (z > 0) {
        return (1 - cos(sqrt(z)))/z;
    } else if (z < 0) {
        return (cosh(sqrt(-z)) - 1) / -z;
    } else {
        return 1.0/2.0;
    }
}

float stump_s(float z) {
    if (z > 0) {
        return (sqrt(z) - sin(sqrt(z)))/(powf(sqrt(-z),3));
    } else if (z < 0) {
        return (sinh(sqrt(-z)) - (sqrt(-z)))/(powf(sqrt(-z),3));
    } else {
        return 1.0/6.0;
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
    };

    return elems;
}

void print_orbital_elements(OrbitalElements elems) {
    printf("------\n");
    printf("semi-major axis = %.2f\n",elems.semimajor_axis);
    printf("eccentricity = %.2f\n",elems.eccentricity);
    printf("orbital period (s) = %.2f\n",elems.period);
    printf("grav param = %.2f\n",elems.grav_param);
    printf("mean anomaly = %.2f\n",elems.mean_anomaly);
    printf("eccentric anomaly = %.2f\n",elems.eccentric_anomaly);
    printf("true anomaly = %.2f\n",elems.true_anomaly);
    printf("inclination = %.2f\n",elems.inclination);
    printf("arg of periapsis = %.2f\n", elems.arg_of_periapsis);
    printf("long of asc node = %.2f\n", elems.long_of_asc_node);
    printf("-------\n");
}

// TODO implement me
/* PhysicalState rv_from_orb_elems(OrbitalElements elems) { */
    
/* } */

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


Vector3 solve_kepler_ellipse_inertial(OrbitalElements elems, float M_naught, float t_naught, float t) {
    Vector2 perifocal_coords = solve_kepler_ellipse_perifocal(elems, M_naught, t_naught, t);

    // Perform a single combined linear transformation on these perifocal 
    // coords to convert to inertial 3d space
    // cos Ω cos ω − sin Ω sin ω cos i 
    float i_1_1 = cos(elems.long_of_asc_node) * cos(elems.arg_of_periapsis) - sin(elems.long_of_asc_node) * sin(elems.arg_of_periapsis) * cos(elems.inclination);
    // − cos Ω sin ω − sin Ω cos ω cos i
    float i_1_2  = - cos(elems.long_of_asc_node) * sin(elems.arg_of_periapsis) - sin(elems.long_of_asc_node) * cos(elems.arg_of_periapsis) * cos(elems.inclination);
    // sin Ω sin i
    float i_1_3 = sin(elems.long_of_asc_node) * sin(elems.inclination);

    // sin Ω cos ω + cos Ω sin ω cos i
    float i_2_1 = sin(elems.long_of_asc_node) * cos(elems.arg_of_periapsis) + cos(elems.long_of_asc_node) * sin(elems.arg_of_periapsis) * cos(elems.inclination);
    // − sin Ω sin ω + cos Ω cos ω cos i
    float i_2_2 = - sin(elems.long_of_asc_node) * sin(elems.arg_of_periapsis) + cos(elems.long_of_asc_node) * cos(elems.arg_of_periapsis) * cos(elems.inclination);
    // − cos Ω sin i
    float i_2_3 = - cos(elems.long_of_asc_node) * sin(elems.inclination);

    // sin ω sin i
    float i_3_1 = sin(elems.arg_of_periapsis) * sin(elems.inclination);
    // cos ω sin i
    float i_3_2 = cos(elems.arg_of_periapsis) * sin(elems.inclination);
    // cos i
    float i_3_3 = cos(elems.inclination);

    // Create 3x3 matrix
    Matrix mat = {
        i_1_1, i_1_2, i_1_3, 0,
        i_2_1, i_2_2, i_2_3, 0,
        i_3_1, i_3_2, i_3_3, 0,
        0,     0,     0,     1,
    };

    // Transform the point and return
    return Vector3Transform((Vector3){.x = perifocal_coords.x, .y = perifocal_coords.y, .z = 0.0}, mat);
}              
               
               
               
               
               
               
               
               
               
               
               
               
