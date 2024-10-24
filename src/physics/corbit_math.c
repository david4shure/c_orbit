#include <math.h>       // Required for: sinf(), cosf(), tan(), atan2f(), sqrtf(), floor(), fminf(), fmaxf(), ffabs()
#include "raymath.h"
#include "corbit_math.h"

//----------------------------------------------------------------------------------
// Module Functions Definition - Utils math
//----------------------------------------------------------------------------------

// Clamp double value
double DClamp(double value, double min, double max)
{
    double result = (value < min)? min : value;

    if (result > max) result = max;

    return result;
}

// Calculate linear interpolation between two doubles
double DLerp(double start, double end, double amount)
{
    double result = start + amount*(end - start);

    return result;
}

// Normalize input value within input range
double DNormalize(double value, double start, double end)
{
    double result = (value - start)/(end - start);

    return result;
}

// Remap input value within input range to output range
double DRemap(double value, double inputStart, double inputEnd, double outputStart, double outputEnd)
{
    double result = (value - inputStart)/(inputEnd - inputStart)*(outputEnd - outputStart) + outputStart;

    return result;
}

// Wrap input value from min to max
double DWrap(double value, double min, double max)
{
    double result = value - (max - min)*floorf((value - min)/(max - min));

    return result;
}

// Check whether two given doubles are almost equal
int DoubleEquals(double x, double y)
{
#if !defined(EPSILON)
    #define EPSILON 0.000001f
#endif

    int result = (fabs(x - y)) <= (EPSILON*fmaxf(1.0f, fmaxf(fabs(x), fabs(y))));

    return result;
}

//----------------------------------------------------------------------------------
// Module Functions Definition - DVector2 math
//----------------------------------------------------------------------------------

// Vector with components value 0.0f
DVector2 DVector2Zero(void)
{
    DVector2 result = { 0.0f, 0.0f };

    return result;
}

// Vector with components value 1.0f
DVector2 DVector2One(void)
{
    DVector2 result = { 1.0f, 1.0f };

    return result;
}

// Add two vectors (v1 + v2)
DVector2 DVector2Add(DVector2 v1, DVector2 v2)
{
    DVector2 result = { v1.x + v2.x, v1.y + v2.y };

    return result;
}

// Add vector and double value
DVector2 DVector2AddValue(DVector2 v, double add)
{
    DVector2 result = { v.x + add, v.y + add };

    return result;
}

// Subtract two vectors (v1 - v2)
DVector2 DVector2Subtract(DVector2 v1, DVector2 v2)
{
    DVector2 result = { v1.x - v2.x, v1.y - v2.y };

    return result;
}

// Subtract vector by double value
DVector2 DVector2SubtractValue(DVector2 v, double sub)
{
    DVector2 result = { v.x - sub, v.y - sub };

    return result;
}

// Calculate vector length
double DVector2Length(DVector2 v)
{
    double result = sqrtf((v.x*v.x) + (v.y*v.y));

    return result;
}

// Calculate vector square length
double DVector2LengthSqr(DVector2 v)
{
    double result = (v.x*v.x) + (v.y*v.y);

    return result;
}

// Calculate two vectors dot product
double DVector2DotProduct(DVector2 v1, DVector2 v2)
{
    double result = (v1.x*v2.x + v1.y*v2.y);

    return result;
}

// Calculate distance between two vectors
double DVector2Distance(DVector2 v1, DVector2 v2)
{
    double result = sqrtf((v1.x - v2.x)*(v1.x - v2.x) + (v1.y - v2.y)*(v1.y - v2.y));

    return result;
}

// Calculate square distance between two vectors
double DVector2DistanceSqr(DVector2 v1, DVector2 v2)
{
    double result = ((v1.x - v2.x)*(v1.x - v2.x) + (v1.y - v2.y)*(v1.y - v2.y));

    return result;
}

// Calculate angle between two vectors
// NOTE: Angle is calculated from origin point (0, 0)
double DVector2Angle(DVector2 v1, DVector2 v2)
{
    double result = 0.0f;

    double dot = v1.x*v2.x + v1.y*v2.y;
    double det = v1.x*v2.y - v1.y*v2.x;

    result = atan2f(det, dot);

    return result;
}

// Calculate angle defined by a two vectors line
// NOTE: Parameters need to be normalized
// Current implementation should be aligned with glm::angle
double DVector2LineAngle(DVector2 start, DVector2 end)
{
    double result = 0.0f;

    // TODO(10/9/2023): Currently angles move clockwise, determine if this is wanted behavior
    result = -atan2f(end.y - start.y, end.x - start.x);

    return result;
}

// Scale vector (multiply by value)
DVector2 DVector2Scale(DVector2 v, double scale)
{
    DVector2 result = { v.x*scale, v.y*scale };

    return result;
}

// Multiply vector by vector
DVector2 DVector2Multiply(DVector2 v1, DVector2 v2)
{
    DVector2 result = { v1.x*v2.x, v1.y*v2.y };

    return result;
}

// Negate vector
DVector2 DVector2Negate(DVector2 v)
{
    DVector2 result = { -v.x, -v.y };

    return result;
}

// Divide vector by vector
DVector2 DVector2Divide(DVector2 v1, DVector2 v2)
{
    DVector2 result = { v1.x/v2.x, v1.y/v2.y };

    return result;
}

// Normalize provided vector
DVector2 DVector2Normalize(DVector2 v)
{
    DVector2 result = { 0 };
    double length = sqrtf((v.x*v.x) + (v.y*v.y));

    if (length > 0)
    {
        double ilength = 1.0f/length;
        result.x = v.x*ilength;
        result.y = v.y*ilength;
    }

    return result;
}

// Transforms a DVector2 by a given DMatrix
DVector2 DVector2Transform(DVector2 v, DMatrix mat)
{
    DVector2 result = { 0 };

    double x = v.x;
    double y = v.y;
    double z = 0;

    result.x = mat.m0*x + mat.m4*y + mat.m8*z + mat.m12;
    result.y = mat.m1*x + mat.m5*y + mat.m9*z + mat.m13;

    return result;
}

// Calculate linear interpolation between two vectors
DVector2 DVector2Lerp(DVector2 v1, DVector2 v2, double amount)
{
    DVector2 result = { 0 };

    result.x = v1.x + amount*(v2.x - v1.x);
    result.y = v1.y + amount*(v2.y - v1.y);

    return result;
}

// Calculate reflected vector to normal
DVector2 DVector2Reflect(DVector2 v, DVector2 normal)
{
    DVector2 result = { 0 };

    double dotProduct = (v.x*normal.x + v.y*normal.y); // Dot product

    result.x = v.x - (2.0f*normal.x)*dotProduct;
    result.y = v.y - (2.0f*normal.y)*dotProduct;

    return result;
}

// Rotate vector by angle
DVector2 DVector2Rotate(DVector2 v, double angle)
{
    DVector2 result = { 0 };

    double cosres = cosf(angle);
    double sinres = sinf(angle);

    result.x = v.x*cosres - v.y*sinres;
    result.y = v.x*sinres + v.y*cosres;

    return result;
}

// Move Vector towards target
DVector2 DVector2MoveTowards(DVector2 v, DVector2 target, double maxDistance)
{
    DVector2 result = { 0 };

    double dx = target.x - v.x;
    double dy = target.y - v.y;
    double value = (dx*dx) + (dy*dy);

    if ((value == 0) || ((maxDistance >= 0) && (value <= maxDistance*maxDistance))) return target;

    double dist = sqrtf(value);

    result.x = v.x + dx/dist*maxDistance;
    result.y = v.y + dy/dist*maxDistance;

    return result;
}

// Invert the given vector
DVector2 DVector2Invert(DVector2 v)
{
    DVector2 result = { 1.0f/v.x, 1.0f/v.y };

    return result;
}

// Clamp the components of the vector between
// min and max values specified by the given vectors
DVector2 DVector2Clamp(DVector2 v, DVector2 min, DVector2 max)
{
    DVector2 result = { 0 };

    result.x = fminf(max.x, fmaxf(min.x, v.x));
    result.y = fminf(max.y, fmaxf(min.y, v.y));

    return result;
}

// Clamp the magnitude of the vector between two min and max values
DVector2 DVector2ClampValue(DVector2 v, double min, double max)
{
    DVector2 result = v;

    double length = (v.x*v.x) + (v.y*v.y);
    if (length > 0.0f)
    {
        length = sqrtf(length);

        if (length < min)
        {
            double scale = min/length;
            result.x = v.x*scale;
            result.y = v.y*scale;
        }
        else if (length > max)
        {
            double scale = max/length;
            result.x = v.x*scale;
            result.y = v.y*scale;
        }
    }

    return result;
}

// Check whether two given vectors are almost equal
int DVector2Equals(DVector2 p, DVector2 q)
{
#if !defined(EPSILON)
    #define EPSILON 0.000001f
#endif

    int result = ((fabs(p.x - q.x)) <= (EPSILON*fmaxf(1.0f, fmaxf(fabs(p.x), fabs(q.x))))) &&
                  ((fabs(p.y - q.y)) <= (EPSILON*fmaxf(1.0f, fmaxf(fabs(p.y), fabs(q.y)))));

    return result;
}

//----------------------------------------------------------------------------------
// Module Functions Definition - DVector3 math
//----------------------------------------------------------------------------------

// Vector with components value 0.0f
DVector3 DVector3Zero(void)
{
    DVector3 result = { 0.0f, 0.0f, 0.0f };

    return result;
}

// Vector with components value 1.0f
DVector3 DVector3One(void)
{
    DVector3 result = { 1.0f, 1.0f, 1.0f };

    return result;
}

// Add two vectors
DVector3 DVector3Add(DVector3 v1, DVector3 v2)
{
    DVector3 result = { v1.x + v2.x, v1.y + v2.y, v1.z + v2.z };

    return result;
}

// Add vector and double value
DVector3 DVector3AddValue(DVector3 v, double add)
{
    DVector3 result = { v.x + add, v.y + add, v.z + add };

    return result;
}

// Subtract two vectors
DVector3 DVector3Subtract(DVector3 v1, DVector3 v2)
{
    DVector3 result = { v1.x - v2.x, v1.y - v2.y, v1.z - v2.z };

    return result;
}

// Subtract vector by double value
DVector3 DVector3SubtractValue(DVector3 v, double sub)
{
    DVector3 result = { v.x - sub, v.y - sub, v.z - sub };

    return result;
}

// Multiply vector by scalar
DVector3 DVector3Scale(DVector3 v, double scalar)
{
    DVector3 result = { v.x*scalar, v.y*scalar, v.z*scalar };

    return result;
}

// Multiply vector by vector
DVector3 DVector3Multiply(DVector3 v1, DVector3 v2)
{
    DVector3 result = { v1.x*v2.x, v1.y*v2.y, v1.z*v2.z };

    return result;
}

// Calculate two vectors cross product
DVector3 DVector3CrossProduct(DVector3 v1, DVector3 v2)
{
    DVector3 result = { v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x };

    return result;
}

// Calculate one vector perpendicular vector
DVector3 DVector3Perpendicular(DVector3 v)
{
    DVector3 result = { 0 };

    double min = (double) fabs(v.x);
    DVector3 cardinalAxis = {1.0f, 0.0f, 0.0f};

    if (fabs(v.y) < min)
    {
        min = (double) fabs(v.y);
        DVector3 tmp = {0.0f, 1.0f, 0.0f};
        cardinalAxis = tmp;
    }

    if (fabs(v.z) < min)
    {
        DVector3 tmp = {0.0f, 0.0f, 1.0f};
        cardinalAxis = tmp;
    }

    // Cross product between vectors
    result.x = v.y*cardinalAxis.z - v.z*cardinalAxis.y;
    result.y = v.z*cardinalAxis.x - v.x*cardinalAxis.z;
    result.z = v.x*cardinalAxis.y - v.y*cardinalAxis.x;

    return result;
}

// Calculate vector length
double DVector3Length(const DVector3 v)
{
    double result = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);

    return result;
}

// Calculate vector square length
double DVector3LengthSqr(const DVector3 v)
{
    double result = v.x*v.x + v.y*v.y + v.z*v.z;

    return result;
}

// Calculate two vectors dot product
double DVector3DotProduct(DVector3 v1, DVector3 v2)
{
    double result = (v1.x*v2.x + v1.y*v2.y + v1.z*v2.z);

    return result;
}

// Calculate distance between two vectors
double DVector3Distance(DVector3 v1, DVector3 v2)
{
    double result = 0.0f;

    double dx = v2.x - v1.x;
    double dy = v2.y - v1.y;
    double dz = v2.z - v1.z;
    result = sqrtf(dx*dx + dy*dy + dz*dz);

    return result;
}

// Calculate square distance between two vectors
double DVector3DistanceSqr(DVector3 v1, DVector3 v2)
{
    double result = 0.0f;

    double dx = v2.x - v1.x;
    double dy = v2.y - v1.y;
    double dz = v2.z - v1.z;
    result = dx*dx + dy*dy + dz*dz;

    return result;
}

// Calculate angle between two vectors
double DVector3Angle(DVector3 v1, DVector3 v2)
{
    double result = 0.0f;

    DVector3 cross = { v1.y*v2.z - v1.z*v2.y, v1.z*v2.x - v1.x*v2.z, v1.x*v2.y - v1.y*v2.x };
    double len = sqrtf(cross.x*cross.x + cross.y*cross.y + cross.z*cross.z);
    double dot = (v1.x*v2.x + v1.y*v2.y + v1.z*v2.z);
    result = atan2f(len, dot);

    return result;
}

// Negate provided vector (invert direction)
DVector3 DVector3Negate(DVector3 v)
{
    DVector3 result = { -v.x, -v.y, -v.z };

    return result;
}

// Divide vector by vector
DVector3 DVector3Divide(DVector3 v1, DVector3 v2)
{
    DVector3 result = { v1.x/v2.x, v1.y/v2.y, v1.z/v2.z };

    return result;
}

// Normalize provided vector
DVector3 DVector3Normalize(DVector3 v)
{
    DVector3 result = v;

    double length = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
    if (length != 0.0f)
    {
        double ilength = 1.0f/length;

        result.x *= ilength;
        result.y *= ilength;
        result.z *= ilength;
    }

    return result;
}

//Calculate the projection of the vector v1 on to v2
DVector3 DVector3Project(DVector3 v1, DVector3 v2)
{
    DVector3 result = { 0 };
    
    double v1dv2 = (v1.x*v2.x + v1.y*v2.y + v1.z*v2.z);
    double v2dv2 = (v2.x*v2.x + v2.y*v2.y + v2.z*v2.z);

    double mag = v1dv2/v2dv2;

    result.x = v2.x*mag;
    result.y = v2.y*mag;
    result.z = v2.z*mag;

    return result;
}

//Calculate the rejection of the vector v1 on to v2
DVector3 DVector3Reject(DVector3 v1, DVector3 v2)
{
    DVector3 result = { 0 };
    
    double v1dv2 = (v1.x*v2.x + v1.y*v2.y + v1.z*v2.z);
    double v2dv2 = (v2.x*v2.x + v2.y*v2.y + v2.z*v2.z);

    double mag = v1dv2/v2dv2;

    result.x = v1.x - (v2.x*mag);
    result.y = v1.y - (v2.y*mag);
    result.z = v1.z - (v2.z*mag);

    return result;
}

// Orthonormalize provided vectors
// Makes vectors normalized and orthogonal to each other
// Gram-Schmidt function implementation
void DVector3OrthoNormalize(DVector3 *v1, DVector3 *v2)
{
    double length = 0.0f;
    double ilength = 0.0f;

    // DVector3Normalize(*v1);
    DVector3 v = *v1;
    length = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
    if (length == 0.0f) length = 1.0f;
    ilength = 1.0f/length;
    v1->x *= ilength;
    v1->y *= ilength;
    v1->z *= ilength;

    // DVector3CrossProduct(*v1, *v2)
    DVector3 vn1 = { v1->y*v2->z - v1->z*v2->y, v1->z*v2->x - v1->x*v2->z, v1->x*v2->y - v1->y*v2->x };

    // DVector3Normalize(vn1);
    v = vn1;
    length = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
    if (length == 0.0f) length = 1.0f;
    ilength = 1.0f/length;
    vn1.x *= ilength;
    vn1.y *= ilength;
    vn1.z *= ilength;

    // DVector3CrossProduct(vn1, *v1)
    DVector3 vn2 = { vn1.y*v1->z - vn1.z*v1->y, vn1.z*v1->x - vn1.x*v1->z, vn1.x*v1->y - vn1.y*v1->x };

    *v2 = vn2;
}

// Transforms a DVector3 by a given DMatrix
DVector3 DVector3Transform(DVector3 v, DMatrix mat)
{
    DVector3 result = { 0 };

    double x = v.x;
    double y = v.y;
    double z = v.z;

    result.x = mat.m0*x + mat.m4*y + mat.m8*z + mat.m12;
    result.y = mat.m1*x + mat.m5*y + mat.m9*z + mat.m13;
    result.z = mat.m2*x + mat.m6*y + mat.m10*z + mat.m14;

    return result;
}

// Transform a vector by quaternion rotation
DVector3 DVector3RotateByDQuaternion(DVector3 v, DQuaternion q)
{
    DVector3 result = { 0 };

    result.x = v.x*(q.x*q.x + q.w*q.w - q.y*q.y - q.z*q.z) + v.y*(2*q.x*q.y - 2*q.w*q.z) + v.z*(2*q.x*q.z + 2*q.w*q.y);
    result.y = v.x*(2*q.w*q.z + 2*q.x*q.y) + v.y*(q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z) + v.z*(-2*q.w*q.x + 2*q.y*q.z);
    result.z = v.x*(-2*q.w*q.y + 2*q.x*q.z) + v.y*(2*q.w*q.x + 2*q.y*q.z)+ v.z*(q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);

    return result;
}

// Rotates a vector around an axis
DVector3 DVector3RotateByAxisAngle(DVector3 v, DVector3 axis, double angle)
{
    // Using Euler-Rodrigues Formula
    // Ref.: https://en.wikipedia.org/w/index.php?title=Euler%E2%80%93Rodrigues_formula

    DVector3 result = v;

    // DVector3Normalize(axis);
    double length = sqrtf(axis.x*axis.x + axis.y*axis.y + axis.z*axis.z);
    if (length == 0.0f) length = 1.0f;
    double ilength = 1.0f / length;
    axis.x *= ilength;
    axis.y *= ilength;
    axis.z *= ilength;

    angle /= 2.0f;
    double a = sinf(angle);
    double b = axis.x*a;
    double c = axis.y*a;
    double d = axis.z*a;
    a = cosf(angle);
    DVector3 w = { b, c, d };

    // DVector3CrossProduct(w, v)
    DVector3 wv = { w.y*v.z - w.z*v.y, w.z*v.x - w.x*v.z, w.x*v.y - w.y*v.x };

    // DVector3CrossProduct(w, wv)
    DVector3 wwv = { w.y*wv.z - w.z*wv.y, w.z*wv.x - w.x*wv.z, w.x*wv.y - w.y*wv.x };

    // DVector3Scale(wv, 2*a)
    a *= 2;
    wv.x *= a;
    wv.y *= a;
    wv.z *= a;

    // DVector3Scale(wwv, 2)
    wwv.x *= 2;
    wwv.y *= 2;
    wwv.z *= 2;

    result.x += wv.x;
    result.y += wv.y;
    result.z += wv.z;

    result.x += wwv.x;
    result.y += wwv.y;
    result.z += wwv.z;

    return result;
}

// Calculate linear interpolation between two vectors
DVector3 DVector3Lerp(DVector3 v1, DVector3 v2, double amount)
{
    DVector3 result = { 0 };

    result.x = v1.x + amount*(v2.x - v1.x);
    result.y = v1.y + amount*(v2.y - v1.y);
    result.z = v1.z + amount*(v2.z - v1.z);

    return result;
}

// Calculate reflected vector to normal
DVector3 DVector3Reflect(DVector3 v, DVector3 normal)
{
    DVector3 result = { 0 };

    // I is the original vector
    // N is the normal of the incident plane
    // R = I - (2*N*(DotProduct[I, N]))

    double dotProduct = (v.x*normal.x + v.y*normal.y + v.z*normal.z);

    result.x = v.x - (2.0f*normal.x)*dotProduct;
    result.y = v.y - (2.0f*normal.y)*dotProduct;
    result.z = v.z - (2.0f*normal.z)*dotProduct;

    return result;
}

// Get min value for each pair of components
DVector3 DVector3Min(DVector3 v1, DVector3 v2)
{
    DVector3 result = { 0 };

    result.x = fminf(v1.x, v2.x);
    result.y = fminf(v1.y, v2.y);
    result.z = fminf(v1.z, v2.z);

    return result;
}

// Get max value for each pair of components
DVector3 DVector3Max(DVector3 v1, DVector3 v2)
{
    DVector3 result = { 0 };

    result.x = fmaxf(v1.x, v2.x);
    result.y = fmaxf(v1.y, v2.y);
    result.z = fmaxf(v1.z, v2.z);

    return result;
}

// Compute barycenter coordinates (u, v, w) for point p with respect to triangle (a, b, c)
// NOTE: Assumes P is on the plane of the triangle
DVector3 DVector3Barycenter(DVector3 p, DVector3 a, DVector3 b, DVector3 c)
{
    DVector3 result = { 0 };

    DVector3 v0 = { b.x - a.x, b.y - a.y, b.z - a.z };   // DVector3Subtract(b, a)
    DVector3 v1 = { c.x - a.x, c.y - a.y, c.z - a.z };   // DVector3Subtract(c, a)
    DVector3 v2 = { p.x - a.x, p.y - a.y, p.z - a.z };   // DVector3Subtract(p, a)
    double d00 = (v0.x*v0.x + v0.y*v0.y + v0.z*v0.z);    // DVector3DotProduct(v0, v0)
    double d01 = (v0.x*v1.x + v0.y*v1.y + v0.z*v1.z);    // DVector3DotProduct(v0, v1)
    double d11 = (v1.x*v1.x + v1.y*v1.y + v1.z*v1.z);    // DVector3DotProduct(v1, v1)
    double d20 = (v2.x*v0.x + v2.y*v0.y + v2.z*v0.z);    // DVector3DotProduct(v2, v0)
    double d21 = (v2.x*v1.x + v2.y*v1.y + v2.z*v1.z);    // DVector3DotProduct(v2, v1)

    double denom = d00*d11 - d01*d01;

    result.y = (d11*d20 - d01*d21)/denom;
    result.z = (d00*d21 - d01*d20)/denom;
    result.x = 1.0f - (result.z + result.y);

    return result;
}

// Projects a DVector3 from screen space into object space
// NOTE: We are avoiding calling other raymath functions despite available
DVector3 DVector3Unproject(DVector3 source, DMatrix projection, DMatrix view)
{
    DVector3 result = { 0 };

    // Calculate unprojected matrix (multiply view matrix by projection matrix) and invert it
    DMatrix matViewProj = {      // DMatrixMultiply(view, projection);
        view.m0*projection.m0 + view.m1*projection.m4 + view.m2*projection.m8 + view.m3*projection.m12,
        view.m0*projection.m1 + view.m1*projection.m5 + view.m2*projection.m9 + view.m3*projection.m13,
        view.m0*projection.m2 + view.m1*projection.m6 + view.m2*projection.m10 + view.m3*projection.m14,
        view.m0*projection.m3 + view.m1*projection.m7 + view.m2*projection.m11 + view.m3*projection.m15,
        view.m4*projection.m0 + view.m5*projection.m4 + view.m6*projection.m8 + view.m7*projection.m12,
        view.m4*projection.m1 + view.m5*projection.m5 + view.m6*projection.m9 + view.m7*projection.m13,
        view.m4*projection.m2 + view.m5*projection.m6 + view.m6*projection.m10 + view.m7*projection.m14,
        view.m4*projection.m3 + view.m5*projection.m7 + view.m6*projection.m11 + view.m7*projection.m15,
        view.m8*projection.m0 + view.m9*projection.m4 + view.m10*projection.m8 + view.m11*projection.m12,
        view.m8*projection.m1 + view.m9*projection.m5 + view.m10*projection.m9 + view.m11*projection.m13,
        view.m8*projection.m2 + view.m9*projection.m6 + view.m10*projection.m10 + view.m11*projection.m14,
        view.m8*projection.m3 + view.m9*projection.m7 + view.m10*projection.m11 + view.m11*projection.m15,
        view.m12*projection.m0 + view.m13*projection.m4 + view.m14*projection.m8 + view.m15*projection.m12,
        view.m12*projection.m1 + view.m13*projection.m5 + view.m14*projection.m9 + view.m15*projection.m13,
        view.m12*projection.m2 + view.m13*projection.m6 + view.m14*projection.m10 + view.m15*projection.m14,
        view.m12*projection.m3 + view.m13*projection.m7 + view.m14*projection.m11 + view.m15*projection.m15 };

    // Calculate inverted matrix -> DMatrixInvert(matViewProj);
    // Cache the matrix values (speed optimization)
    double a00 = matViewProj.m0, a01 = matViewProj.m1, a02 = matViewProj.m2, a03 = matViewProj.m3;
    double a10 = matViewProj.m4, a11 = matViewProj.m5, a12 = matViewProj.m6, a13 = matViewProj.m7;
    double a20 = matViewProj.m8, a21 = matViewProj.m9, a22 = matViewProj.m10, a23 = matViewProj.m11;
    double a30 = matViewProj.m12, a31 = matViewProj.m13, a32 = matViewProj.m14, a33 = matViewProj.m15;

    double b00 = a00*a11 - a01*a10;
    double b01 = a00*a12 - a02*a10;
    double b02 = a00*a13 - a03*a10;
    double b03 = a01*a12 - a02*a11;
    double b04 = a01*a13 - a03*a11;
    double b05 = a02*a13 - a03*a12;
    double b06 = a20*a31 - a21*a30;
    double b07 = a20*a32 - a22*a30;
    double b08 = a20*a33 - a23*a30;
    double b09 = a21*a32 - a22*a31;
    double b10 = a21*a33 - a23*a31;
    double b11 = a22*a33 - a23*a32;

    // Calculate the invert determinant (inlined to avoid double-caching)
    double invDet = 1.0f/(b00*b11 - b01*b10 + b02*b09 + b03*b08 - b04*b07 + b05*b06);

    DMatrix matViewProjInv = {
        (a11*b11 - a12*b10 + a13*b09)*invDet,
        (-a01*b11 + a02*b10 - a03*b09)*invDet,
        (a31*b05 - a32*b04 + a33*b03)*invDet,
        (-a21*b05 + a22*b04 - a23*b03)*invDet,
        (-a10*b11 + a12*b08 - a13*b07)*invDet,
        (a00*b11 - a02*b08 + a03*b07)*invDet,
        (-a30*b05 + a32*b02 - a33*b01)*invDet,
        (a20*b05 - a22*b02 + a23*b01)*invDet,
        (a10*b10 - a11*b08 + a13*b06)*invDet,
        (-a00*b10 + a01*b08 - a03*b06)*invDet,
        (a30*b04 - a31*b02 + a33*b00)*invDet,
        (-a20*b04 + a21*b02 - a23*b00)*invDet,
        (-a10*b09 + a11*b07 - a12*b06)*invDet,
        (a00*b09 - a01*b07 + a02*b06)*invDet,
        (-a30*b03 + a31*b01 - a32*b00)*invDet,
        (a20*b03 - a21*b01 + a22*b00)*invDet };

    // Create quaternion from source point
    DQuaternion quat = { source.x, source.y, source.z, 1.0f };

    // Multiply quat point by unprojecte matrix
    DQuaternion qtransformed = {     // DQuaternionTransform(quat, matViewProjInv)
        matViewProjInv.m0*quat.x + matViewProjInv.m4*quat.y + matViewProjInv.m8*quat.z + matViewProjInv.m12*quat.w,
        matViewProjInv.m1*quat.x + matViewProjInv.m5*quat.y + matViewProjInv.m9*quat.z + matViewProjInv.m13*quat.w,
        matViewProjInv.m2*quat.x + matViewProjInv.m6*quat.y + matViewProjInv.m10*quat.z + matViewProjInv.m14*quat.w,
        matViewProjInv.m3*quat.x + matViewProjInv.m7*quat.y + matViewProjInv.m11*quat.z + matViewProjInv.m15*quat.w };

    // Normalized world points in vectors
    result.x = qtransformed.x/qtransformed.w;
    result.y = qtransformed.y/qtransformed.w;
    result.z = qtransformed.z/qtransformed.w;

    return result;
}

// Get DVector3 as double array
double3 DVector3ToFloatV(DVector3 v)
{
    double3 buffer = { 0 };

    buffer.v[0] = v.x;
    buffer.v[1] = v.y;
    buffer.v[2] = v.z;

    return buffer;
}

// Invert the given vector
DVector3 DVector3Invert(DVector3 v)
{
    DVector3 result = { 1.0f/v.x, 1.0f/v.y, 1.0f/v.z };

    return result;
}

// Clamp the components of the vector between
// min and max values specified by the given vectors
DVector3 DVector3Clamp(DVector3 v, DVector3 min, DVector3 max)
{
    DVector3 result = { 0 };

    result.x = fminf(max.x, fmaxf(min.x, v.x));
    result.y = fminf(max.y, fmaxf(min.y, v.y));
    result.z = fminf(max.z, fmaxf(min.z, v.z));

    return result;
}

// Clamp the magnitude of the vector between two values
DVector3 DVector3ClampValue(DVector3 v, double min, double max)
{
    DVector3 result = v;

    double length = (v.x*v.x) + (v.y*v.y) + (v.z*v.z);
    if (length > 0.0f)
    {
        length = sqrtf(length);

        if (length < min)
        {
            double scale = min/length;
            result.x = v.x*scale;
            result.y = v.y*scale;
            result.z = v.z*scale;
        }
        else if (length > max)
        {
            double scale = max/length;
            result.x = v.x*scale;
            result.y = v.y*scale;
            result.z = v.z*scale;
        }
    }

    return result;
}

// Check whether two given vectors are almost equal
int DVector3Equals(DVector3 p, DVector3 q)
{
#if !defined(EPSILON)
    #define EPSILON 0.000001f
#endif

    int result = ((fabs(p.x - q.x)) <= (EPSILON*fmaxf(1.0f, fmaxf(fabs(p.x), fabs(q.x))))) &&
                 ((fabs(p.y - q.y)) <= (EPSILON*fmaxf(1.0f, fmaxf(fabs(p.y), fabs(q.y))))) &&
                 ((fabs(p.z - q.z)) <= (EPSILON*fmaxf(1.0f, fmaxf(fabs(p.z), fabs(q.z)))));

    return result;
}

// Compute the direction of a refracted ray
// v: normalized direction of the incoming ray
// n: normalized normal vector of the interface of two optical media
// r: ratio of the refractive index of the medium from where the ray comes
//    to the refractive index of the medium on the other side of the surface
DVector3 DVector3Refract(DVector3 v, DVector3 n, double r)
{
    DVector3 result = { 0 };

    double dot = v.x*n.x + v.y*n.y + v.z*n.z;
    double d = 1.0f - r*r*(1.0f - dot*dot);

    if (d >= 0.0f)
    {
        d = sqrtf(d);
        v.x = r*v.x - (r*dot + d)*n.x;
        v.y = r*v.y - (r*dot + d)*n.y;
        v.z = r*v.z - (r*dot + d)*n.z;

        result = v;
    }

    return result;
}

//----------------------------------------------------------------------------------
// Module Functions Definition - DMatrix math
//----------------------------------------------------------------------------------

// Compute matrix determinant
double DMatrixDeterminant(DMatrix mat)
{
    double result = 0.0f;

    // Cache the matrix values (speed optimization)
    double a00 = mat.m0, a01 = mat.m1, a02 = mat.m2, a03 = mat.m3;
    double a10 = mat.m4, a11 = mat.m5, a12 = mat.m6, a13 = mat.m7;
    double a20 = mat.m8, a21 = mat.m9, a22 = mat.m10, a23 = mat.m11;
    double a30 = mat.m12, a31 = mat.m13, a32 = mat.m14, a33 = mat.m15;

    result = a30*a21*a12*a03 - a20*a31*a12*a03 - a30*a11*a22*a03 + a10*a31*a22*a03 +
             a20*a11*a32*a03 - a10*a21*a32*a03 - a30*a21*a02*a13 + a20*a31*a02*a13 +
             a30*a01*a22*a13 - a00*a31*a22*a13 - a20*a01*a32*a13 + a00*a21*a32*a13 +
             a30*a11*a02*a23 - a10*a31*a02*a23 - a30*a01*a12*a23 + a00*a31*a12*a23 +
             a10*a01*a32*a23 - a00*a11*a32*a23 - a20*a11*a02*a33 + a10*a21*a02*a33 +
             a20*a01*a12*a33 - a00*a21*a12*a33 - a10*a01*a22*a33 + a00*a11*a22*a33;

    return result;
}

// Get the trace of the matrix (sum of the values along the diagonal)
double DMatrixTrace(DMatrix mat)
{
    double result = (mat.m0 + mat.m5 + mat.m10 + mat.m15);

    return result;
}

// Transposes provided matrix
DMatrix DMatrixTranspose(DMatrix mat)
{
    DMatrix result = { 0 };

    result.m0 = mat.m0;
    result.m1 = mat.m4;
    result.m2 = mat.m8;
    result.m3 = mat.m12;
    result.m4 = mat.m1;
    result.m5 = mat.m5;
    result.m6 = mat.m9;
    result.m7 = mat.m13;
    result.m8 = mat.m2;
    result.m9 = mat.m6;
    result.m10 = mat.m10;
    result.m11 = mat.m14;
    result.m12 = mat.m3;
    result.m13 = mat.m7;
    result.m14 = mat.m11;
    result.m15 = mat.m15;

    return result;
}

// Invert provided matrix
DMatrix DMatrixInvert(DMatrix mat)
{
    DMatrix result = { 0 };

    // Cache the matrix values (speed optimization)
    double a00 = mat.m0, a01 = mat.m1, a02 = mat.m2, a03 = mat.m3;
    double a10 = mat.m4, a11 = mat.m5, a12 = mat.m6, a13 = mat.m7;
    double a20 = mat.m8, a21 = mat.m9, a22 = mat.m10, a23 = mat.m11;
    double a30 = mat.m12, a31 = mat.m13, a32 = mat.m14, a33 = mat.m15;

    double b00 = a00*a11 - a01*a10;
    double b01 = a00*a12 - a02*a10;
    double b02 = a00*a13 - a03*a10;
    double b03 = a01*a12 - a02*a11;
    double b04 = a01*a13 - a03*a11;
    double b05 = a02*a13 - a03*a12;
    double b06 = a20*a31 - a21*a30;
    double b07 = a20*a32 - a22*a30;
    double b08 = a20*a33 - a23*a30;
    double b09 = a21*a32 - a22*a31;
    double b10 = a21*a33 - a23*a31;
    double b11 = a22*a33 - a23*a32;

    // Calculate the invert determinant (inlined to avoid double-caching)
    double invDet = 1.0f/(b00*b11 - b01*b10 + b02*b09 + b03*b08 - b04*b07 + b05*b06);

    result.m0 = (a11*b11 - a12*b10 + a13*b09)*invDet;
    result.m1 = (-a01*b11 + a02*b10 - a03*b09)*invDet;
    result.m2 = (a31*b05 - a32*b04 + a33*b03)*invDet;
    result.m3 = (-a21*b05 + a22*b04 - a23*b03)*invDet;
    result.m4 = (-a10*b11 + a12*b08 - a13*b07)*invDet;
    result.m5 = (a00*b11 - a02*b08 + a03*b07)*invDet;
    result.m6 = (-a30*b05 + a32*b02 - a33*b01)*invDet;
    result.m7 = (a20*b05 - a22*b02 + a23*b01)*invDet;
    result.m8 = (a10*b10 - a11*b08 + a13*b06)*invDet;
    result.m9 = (-a00*b10 + a01*b08 - a03*b06)*invDet;
    result.m10 = (a30*b04 - a31*b02 + a33*b00)*invDet;
    result.m11 = (-a20*b04 + a21*b02 - a23*b00)*invDet;
    result.m12 = (-a10*b09 + a11*b07 - a12*b06)*invDet;
    result.m13 = (a00*b09 - a01*b07 + a02*b06)*invDet;
    result.m14 = (-a30*b03 + a31*b01 - a32*b00)*invDet;
    result.m15 = (a20*b03 - a21*b01 + a22*b00)*invDet;

    return result;
}

// Get identity matrix
DMatrix DMatrixIdentity(void)
{
    DMatrix result = { 1.0f, 0.0f, 0.0f, 0.0f,
                      0.0f, 1.0f, 0.0f, 0.0f,
                      0.0f, 0.0f, 1.0f, 0.0f,
                      0.0f, 0.0f, 0.0f, 1.0f };

    return result;
}

// Add two matrices
DMatrix DMatrixAdd(DMatrix left, DMatrix right)
{
    DMatrix result = { 0 };

    result.m0 = left.m0 + right.m0;
    result.m1 = left.m1 + right.m1;
    result.m2 = left.m2 + right.m2;
    result.m3 = left.m3 + right.m3;
    result.m4 = left.m4 + right.m4;
    result.m5 = left.m5 + right.m5;
    result.m6 = left.m6 + right.m6;
    result.m7 = left.m7 + right.m7;
    result.m8 = left.m8 + right.m8;
    result.m9 = left.m9 + right.m9;
    result.m10 = left.m10 + right.m10;
    result.m11 = left.m11 + right.m11;
    result.m12 = left.m12 + right.m12;
    result.m13 = left.m13 + right.m13;
    result.m14 = left.m14 + right.m14;
    result.m15 = left.m15 + right.m15;

    return result;
}

// Subtract two matrices (left - right)
DMatrix DMatrixSubtract(DMatrix left, DMatrix right)
{
    DMatrix result = { 0 };

    result.m0 = left.m0 - right.m0;
    result.m1 = left.m1 - right.m1;
    result.m2 = left.m2 - right.m2;
    result.m3 = left.m3 - right.m3;
    result.m4 = left.m4 - right.m4;
    result.m5 = left.m5 - right.m5;
    result.m6 = left.m6 - right.m6;
    result.m7 = left.m7 - right.m7;
    result.m8 = left.m8 - right.m8;
    result.m9 = left.m9 - right.m9;
    result.m10 = left.m10 - right.m10;
    result.m11 = left.m11 - right.m11;
    result.m12 = left.m12 - right.m12;
    result.m13 = left.m13 - right.m13;
    result.m14 = left.m14 - right.m14;
    result.m15 = left.m15 - right.m15;

    return result;
}

// Get two matrix multiplication
// NOTE: When multiplying matrices... the order matters!
DMatrix DMatrixMultiply(DMatrix left, DMatrix right)
{
    DMatrix result = { 0 };

    result.m0 = left.m0*right.m0 + left.m1*right.m4 + left.m2*right.m8 + left.m3*right.m12;
    result.m1 = left.m0*right.m1 + left.m1*right.m5 + left.m2*right.m9 + left.m3*right.m13;
    result.m2 = left.m0*right.m2 + left.m1*right.m6 + left.m2*right.m10 + left.m3*right.m14;
    result.m3 = left.m0*right.m3 + left.m1*right.m7 + left.m2*right.m11 + left.m3*right.m15;
    result.m4 = left.m4*right.m0 + left.m5*right.m4 + left.m6*right.m8 + left.m7*right.m12;
    result.m5 = left.m4*right.m1 + left.m5*right.m5 + left.m6*right.m9 + left.m7*right.m13;
    result.m6 = left.m4*right.m2 + left.m5*right.m6 + left.m6*right.m10 + left.m7*right.m14;
    result.m7 = left.m4*right.m3 + left.m5*right.m7 + left.m6*right.m11 + left.m7*right.m15;
    result.m8 = left.m8*right.m0 + left.m9*right.m4 + left.m10*right.m8 + left.m11*right.m12;
    result.m9 = left.m8*right.m1 + left.m9*right.m5 + left.m10*right.m9 + left.m11*right.m13;
    result.m10 = left.m8*right.m2 + left.m9*right.m6 + left.m10*right.m10 + left.m11*right.m14;
    result.m11 = left.m8*right.m3 + left.m9*right.m7 + left.m10*right.m11 + left.m11*right.m15;
    result.m12 = left.m12*right.m0 + left.m13*right.m4 + left.m14*right.m8 + left.m15*right.m12;
    result.m13 = left.m12*right.m1 + left.m13*right.m5 + left.m14*right.m9 + left.m15*right.m13;
    result.m14 = left.m12*right.m2 + left.m13*right.m6 + left.m14*right.m10 + left.m15*right.m14;
    result.m15 = left.m12*right.m3 + left.m13*right.m7 + left.m14*right.m11 + left.m15*right.m15;

    return result;
}

// Get translation matrix
DMatrix DMatrixTranslate(double x, double y, double z)
{
    DMatrix result = { 1.0f, 0.0f, 0.0f, x,
                      0.0f, 1.0f, 0.0f, y,
                      0.0f, 0.0f, 1.0f, z,
                      0.0f, 0.0f, 0.0f, 1.0f };

    return result;
}

// Create rotation matrix from axis and angle
// NOTE: Angle should be provided in radians
DMatrix DMatrixRotate(DVector3 axis, double angle)
{
    DMatrix result = { 0 };

    double x = axis.x, y = axis.y, z = axis.z;

    double lengthSquared = x*x + y*y + z*z;

    if ((lengthSquared != 1.0f) && (lengthSquared != 0.0f))
    {
        double ilength = 1.0f/sqrtf(lengthSquared);
        x *= ilength;
        y *= ilength;
        z *= ilength;
    }

    double sinres = sinf(angle);
    double cosres = cosf(angle);
    double t = 1.0f - cosres;

    result.m0 = x*x*t + cosres;
    result.m1 = y*x*t + z*sinres;
    result.m2 = z*x*t - y*sinres;
    result.m3 = 0.0f;

    result.m4 = x*y*t - z*sinres;
    result.m5 = y*y*t + cosres;
    result.m6 = z*y*t + x*sinres;
    result.m7 = 0.0f;

    result.m8 = x*z*t + y*sinres;
    result.m9 = y*z*t - x*sinres;
    result.m10 = z*z*t + cosres;
    result.m11 = 0.0f;

    result.m12 = 0.0f;
    result.m13 = 0.0f;
    result.m14 = 0.0f;
    result.m15 = 1.0f;

    return result;
}

// Get x-rotation matrix
// NOTE: Angle must be provided in radians
DMatrix DMatrixRotateX(double angle)
{
    DMatrix result = { 1.0f, 0.0f, 0.0f, 0.0f,
                      0.0f, 1.0f, 0.0f, 0.0f,
                      0.0f, 0.0f, 1.0f, 0.0f,
                      0.0f, 0.0f, 0.0f, 1.0f }; // DMatrixIdentity()

    double cosres = cosf(angle);
    double sinres = sinf(angle);

    result.m5 = cosres;
    result.m6 = sinres;
    result.m9 = -sinres;
    result.m10 = cosres;

    return result;
}

// Get y-rotation matrix
// NOTE: Angle must be provided in radians
DMatrix DMatrixRotateY(double angle)
{
    DMatrix result = { 1.0f, 0.0f, 0.0f, 0.0f,
                      0.0f, 1.0f, 0.0f, 0.0f,
                      0.0f, 0.0f, 1.0f, 0.0f,
                      0.0f, 0.0f, 0.0f, 1.0f }; // DMatrixIdentity()

    double cosres = cosf(angle);
    double sinres = sinf(angle);

    result.m0 = cosres;
    result.m2 = -sinres;
    result.m8 = sinres;
    result.m10 = cosres;

    return result;
}

// Get z-rotation matrix
// NOTE: Angle must be provided in radians
DMatrix DMatrixRotateZ(double angle)
{
    DMatrix result = { 1.0f, 0.0f, 0.0f, 0.0f,
                      0.0f, 1.0f, 0.0f, 0.0f,
                      0.0f, 0.0f, 1.0f, 0.0f,
                      0.0f, 0.0f, 0.0f, 1.0f }; // DMatrixIdentity()

    double cosres = cosf(angle);
    double sinres = sinf(angle);

    result.m0 = cosres;
    result.m1 = sinres;
    result.m4 = -sinres;
    result.m5 = cosres;

    return result;
}


// Get xyz-rotation matrix
// NOTE: Angle must be provided in radians
DMatrix DMatrixRotateXYZ(DVector3 angle)
{
    DMatrix result = { 1.0f, 0.0f, 0.0f, 0.0f,
                      0.0f, 1.0f, 0.0f, 0.0f,
                      0.0f, 0.0f, 1.0f, 0.0f,
                      0.0f, 0.0f, 0.0f, 1.0f }; // DMatrixIdentity()

    double cosz = cosf(-angle.z);
    double sinz = sinf(-angle.z);
    double cosy = cosf(-angle.y);
    double siny = sinf(-angle.y);
    double cosx = cosf(-angle.x);
    double sinx = sinf(-angle.x);

    result.m0 = cosz*cosy;
    result.m1 = (cosz*siny*sinx) - (sinz*cosx);
    result.m2 = (cosz*siny*cosx) + (sinz*sinx);

    result.m4 = sinz*cosy;
    result.m5 = (sinz*siny*sinx) + (cosz*cosx);
    result.m6 = (sinz*siny*cosx) - (cosz*sinx);

    result.m8 = -siny;
    result.m9 = cosy*sinx;
    result.m10= cosy*cosx;

    return result;
}

// Get zyx-rotation matrix
// NOTE: Angle must be provided in radians
DMatrix DMatrixRotateZYX(DVector3 angle)
{
    DMatrix result = { 0 };

    double cz = cosf(angle.z);
    double sz = sinf(angle.z);
    double cy = cosf(angle.y);
    double sy = sinf(angle.y);
    double cx = cosf(angle.x);
    double sx = sinf(angle.x);

    result.m0 = cz*cy;
    result.m4 = cz*sy*sx - cx*sz;
    result.m8 = sz*sx + cz*cx*sy;
    result.m12 = 0;

    result.m1 = cy*sz;
    result.m5 = cz*cx + sz*sy*sx;
    result.m9 = cx*sz*sy - cz*sx;
    result.m13 = 0;

    result.m2 = -sy;
    result.m6 = cy*sx;
    result.m10 = cy*cx;
    result.m14 = 0;

    result.m3 = 0;
    result.m7 = 0;
    result.m11 = 0;
    result.m15 = 1;

    return result;
}

// Get scaling matrix
DMatrix DMatrixScale(double x, double y, double z)
{
    DMatrix result = { x, 0.0f, 0.0f, 0.0f,
                      0.0f, y, 0.0f, 0.0f,
                      0.0f, 0.0f, z, 0.0f,
                      0.0f, 0.0f, 0.0f, 1.0f };

    return result;
}

// Get perspective projection matrix
DMatrix DMatrixFrustum(double left, double right, double bottom, double top, double near, double far)
{
    DMatrix result = { 0 };

    double rl = (double)(right - left);
    double tb = (double)(top - bottom);
    double fn = (double)(far - near);

    result.m0 = ((double)near*2.0f)/rl;
    result.m1 = 0.0f;
    result.m2 = 0.0f;
    result.m3 = 0.0f;

    result.m4 = 0.0f;
    result.m5 = ((double)near*2.0f)/tb;
    result.m6 = 0.0f;
    result.m7 = 0.0f;

    result.m8 = ((double)right + (double)left)/rl;
    result.m9 = ((double)top + (double)bottom)/tb;
    result.m10 = -((double)far + (double)near)/fn;
    result.m11 = -1.0f;

    result.m12 = 0.0f;
    result.m13 = 0.0f;
    result.m14 = -((double)far*(double)near*2.0f)/fn;
    result.m15 = 0.0f;

    return result;
}

// Get perspective projection matrix
// NOTE: Fovy angle must be provided in radians
DMatrix DMatrixPerspective(double fovY, double aspect, double nearPlane, double farPlane)
{
    DMatrix result = { 0 };

    double top = nearPlane*tan(fovY*0.5);
    double bottom = -top;
    double right = top*aspect;
    double left = -right;

    // DMatrixFrustum(-right, right, -top, top, near, far);
    double rl = (double)(right - left);
    double tb = (double)(top - bottom);
    double fn = (double)(farPlane - nearPlane);

    result.m0 = ((double)nearPlane*2.0f)/rl;
    result.m5 = ((double)nearPlane*2.0f)/tb;
    result.m8 = ((double)right + (double)left)/rl;
    result.m9 = ((double)top + (double)bottom)/tb;
    result.m10 = -((double)farPlane + (double)nearPlane)/fn;
    result.m11 = -1.0f;
    result.m14 = -((double)farPlane*(double)nearPlane*2.0f)/fn;

    return result;
}

// Get orthographic projection matrix
DMatrix DMatrixOrtho(double left, double right, double bottom, double top, double nearPlane, double farPlane)
{
    DMatrix result = { 0 };

    double rl = (double)(right - left);
    double tb = (double)(top - bottom);
    double fn = (double)(farPlane - nearPlane);

    result.m0 = 2.0f/rl;
    result.m1 = 0.0f;
    result.m2 = 0.0f;
    result.m3 = 0.0f;
    result.m4 = 0.0f;
    result.m5 = 2.0f/tb;
    result.m6 = 0.0f;
    result.m7 = 0.0f;
    result.m8 = 0.0f;
    result.m9 = 0.0f;
    result.m10 = -2.0f/fn;
    result.m11 = 0.0f;
    result.m12 = -((double)left + (double)right)/rl;
    result.m13 = -((double)top + (double)bottom)/tb;
    result.m14 = -((double)farPlane + (double)nearPlane)/fn;
    result.m15 = 1.0f;

    return result;
}

// Get camera look-at matrix (view matrix)
DMatrix DMatrixLookAt(DVector3 eye, DVector3 target, DVector3 up)
{
    DMatrix result = { 0 };

    double length = 0.0f;
    double ilength = 0.0f;

    // DVector3Subtract(eye, target)
    DVector3 vz = { eye.x - target.x, eye.y - target.y, eye.z - target.z };

    // DVector3Normalize(vz)
    DVector3 v = vz;
    length = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
    if (length == 0.0f) length = 1.0f;
    ilength = 1.0f/length;
    vz.x *= ilength;
    vz.y *= ilength;
    vz.z *= ilength;

    // DVector3CrossProduct(up, vz)
    DVector3 vx = { up.y*vz.z - up.z*vz.y, up.z*vz.x - up.x*vz.z, up.x*vz.y - up.y*vz.x };

    // DVector3Normalize(x)
    v = vx;
    length = sqrtf(v.x*v.x + v.y*v.y + v.z*v.z);
    if (length == 0.0f) length = 1.0f;
    ilength = 1.0f/length;
    vx.x *= ilength;
    vx.y *= ilength;
    vx.z *= ilength;

    // DVector3CrossProduct(vz, vx)
    DVector3 vy = { vz.y*vx.z - vz.z*vx.y, vz.z*vx.x - vz.x*vx.z, vz.x*vx.y - vz.y*vx.x };

    result.m0 = vx.x;
    result.m1 = vy.x;
    result.m2 = vz.x;
    result.m3 = 0.0f;
    result.m4 = vx.y;
    result.m5 = vy.y;
    result.m6 = vz.y;
    result.m7 = 0.0f;
    result.m8 = vx.z;
    result.m9 = vy.z;
    result.m10 = vz.z;
    result.m11 = 0.0f;
    result.m12 = -(vx.x*eye.x + vx.y*eye.y + vx.z*eye.z);   // DVector3DotProduct(vx, eye)
    result.m13 = -(vy.x*eye.x + vy.y*eye.y + vy.z*eye.z);   // DVector3DotProduct(vy, eye)
    result.m14 = -(vz.x*eye.x + vz.y*eye.y + vz.z*eye.z);   // DVector3DotProduct(vz, eye)
    result.m15 = 1.0f;

    return result;
}

// Get double array of matrix data
double16 DMatrixToFloatV(DMatrix mat)
{
    double16 result = { 0 };

    result.v[0] = mat.m0;
    result.v[1] = mat.m1;
    result.v[2] = mat.m2;
    result.v[3] = mat.m3;
    result.v[4] = mat.m4;
    result.v[5] = mat.m5;
    result.v[6] = mat.m6;
    result.v[7] = mat.m7;
    result.v[8] = mat.m8;
    result.v[9] = mat.m9;
    result.v[10] = mat.m10;
    result.v[11] = mat.m11;
    result.v[12] = mat.m12;
    result.v[13] = mat.m13;
    result.v[14] = mat.m14;
    result.v[15] = mat.m15;

    return result;
}
