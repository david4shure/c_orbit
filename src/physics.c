#include "physics.h"
#include "math.h"
#include "raymath.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

float dot(Vector3 a, Vector3 b) {
    // A.x * B.x + A.y * B.y + A.z * B.z
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

float mag(Vector3 a) {
    return sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

// Modifies passed in vector normalize it
void norm(Vector3* a) {
    float magn = mag(*a);
    
    a->x = a->x / magn;
    a->y = a->y / magn;
    a->z = a->z / magn;
}

void cross(Vector3 a, Vector3 b, Vector3* c) {
    // C.x = A.y * B.z - A.z * B.y
    // C.y = A.z * B.x - A.x * B.z
    // C.z = A.x * B.y - A.y * B.x
    
    c->x = a.y * b.z - a.z * b.y;
    c->y = a.z * b.x - a.x * b.z;
    c->z = a.x * b.y - a.y * b.x;
}

float mean_anom(float M_naught, float t, float t_naught, float T) {
    // First calculate mean motion
    // Given by n = (2 * PI) / T
    float n = (2 * PI) / T;
    // Calculate mean anomaly given time + mean anomaly at epoch + time elapsed since
    // Given by M = M_naught + n ( t - t_naught);
    float M = M_naught + n * (t - t_naught);
    
    return M;
}

float true_anomaly_to_eccentric_anomaly(float θ, float e) {
    // Calculate the factor sqrt((1 - e) / (1 + e))
    float factor = sqrt((1 - e) / (1 + e));
    
    // Calculate tan(E / 2)
    float tanE_half = factor * tan(θ / 2);
    
    // Calculate the eccentric anomaly E
    float E = 2 * atan(tanE_half);
    
    return E;
}

float ecc_anom_to_true_anom(float e, float E) {
    float numerator = 1 + e;
    float denomenator = 1 - e;
    
    float v = 2 * atan(sqrt(numerator/denomenator) * tan(E/2.0));

    return v;
}

float ecc_anom_to_mean_anom(float E, float e) {
    return E - e * sin(E);
}

/* float kepler_E_newt(float e, float M, int max_iters) { */
/*     float error = 1e-8; */

/*     float E = 0.0; */

/*     if (M < PI) { */
/*         E = M + e; */
/*     } else { */
/*         E = M - e; */
/*     } */

/*     printf("inital E = %.2f\n",E); */

/*     float ratio = 1.0; */
/*     int iters = 0; */

/*     while(fabs(ratio) > error) { */
/*         printf("ratio = %.2f, E = %.2f\n",ratio,E); */
/*         printf(".\n"); */
/*         ratio = (E - e*sin(E) - M)/(1 - e * cos(E)); */
/*         E = E - ratio; */
/*         iters++; */

/*         if (iters > max_iters) { */
/*             printf("Breaking on max iters...\n"); */
/*             break; */
/*         } */
/*     } */

/*     return E; */
/* } */

/*         let eta = 1e-15_f64; */
/*         let e_naught; */

/*         if (-1. * PI64 < mean_anomaly && mean_anomaly < 0.) || (mean_anomaly > PI64) { */
/*             e_naught = mean_anomaly - self.eccentricity; */
/*         } else { */
/*             e_naught = mean_anomaly + self.eccentricity; */
/*         } */

/*         let mut e_n = e_naught; */
/*         let mut delta = eta + 1.; */
/*         let mut count = 0; */
/*         let mut e_np1 = 0.; */

/*         while delta > eta { */
/*             e_np1 = e_n */
/*                 + (mean_anomaly - e_n + self.eccentricity * e_n.sin()) */
/*                     / (1. - self.eccentricity * e_n.cos()); */
/*             delta = (e_np1 - e_n).abs(); */
/*             e_n = e_np1; */
/*             count += 1; */

/*             if count > 20 { */
/*                 println!("Something bad happened, couldn't converge for eccentric anomaly."); */
/*                 return 0. as f64; */
/*             } */
/*         } */
        
/*         e_np1 */


float kepler_E_newt(float e, float M, int max_iters) {
    float eta = 1e-50;
    float e_naught;

    if ((-1 * PI < M && M < 0) || (M > PI)) {
        e_naught = M - e;
    } else {
        e_naught = M + e;
    }

    float e_n = e_naught;
    float delta = eta + 1;
    int count = 0;
    float e_np1 = 0;

    while (delta > eta) {
        e_np1 = e_n
                + (M - e_n + e * sin(e_n))
                    / (1. - e * cos(e_n));
        delta = fabsf(e_np1 - e_n);
        e_n = e_np1;
        count++;

        if (count > max_iters) {
            break;
        }
    }

    printf("Converged in %d iters\n",count);

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

float stump_s(float x, float min, float max) {
    
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

float distance(float e, float a, float E) {
    return a * (1. - e * cos(E));
}

OrbitalElements orb_elems_from_rv(OrbitalState rv, float μ) {
    float eps = 1.e-10;

    // R,V position velocity vectors
    Vector3 R = rv.r;
    Vector3 V = rv.v;

    // r,v magnitudes of R and V
    float r = mag(R); 
    float v = mag(V);

    // R dot V / r
    float vr = dot(R,V)/r;

    Vector3 H;
    cross(R,V,&H);

    float h = mag(H);

    float i = acos(H.z/h);

    Vector3 N;
    
    Vector3 z_ident = { 0, 0, 1 };

    cross(z_ident,H,&N);

    float n = mag(N);

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

    // Orbital eccentricity = mag(E);
    float e = mag(E);

    float w;

    if (n == 0) {
        w = 0;
    } else {
        if (e > eps) {
            w = acos(dot(N,E)/n/e);
            if (E.z < 0) {
                w = 2 * PI - w;
            }
        } else {
            w = 0;
        }
    }

    float Ta;

    if (e > eps) {
        Ta = acos(dot(E,R)/e/r);

        if (vr < 0) {
            Ta = 2 * PI - Ta;
        }
    } else {
        Vector3 cp;

        cross(N,R,&cp);

        if (cp.z >= 0) {
            Ta = acos(dot(N,R)/n/r);
        } else {
            Ta = 2 * PI - acos(dot(N,R)/n/r);
        }
    }

    float a = h * h / μ / (1 - e * e);
    float Ea = true_anomaly_to_eccentric_anomaly(Ta, e);
    float Ma = ecc_anom_to_mean_anom(Ea, e);

    OrbitalElements elems = { 
        .a = a,
        .e = e,
        .E = Ea,
        .M = Ma,
        .i = i,
        .Ω = Ra,
        .μ = μ,
        .θ = Ta,
        .ω = w 
    };

    return elems;
}
