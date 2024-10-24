#ifndef DMATH_H_CORBIT
#define DMATH_H_CORBIT

#include <math.h>

// Types and Structures Definition
typedef struct DVector2 {
    double x;
    double y;
} DVector2;

typedef struct DVector3 {
    double x;
    double y;
    double z;
} DVector3;

typedef struct DVector4 {
    double x;
    double y;
    double z;
    double w;
} DVector4;

typedef DVector4 DQuaternion;

typedef struct DMatrix {
    double m0, m4, m8, m12;
    double m1, m5, m9, m13;
    double m2, m6, m10, m14;
    double m3, m7, m11, m15;
} DMatrix;

typedef struct double3 {
    double v[3];
} double3;

typedef struct double16 {
    double v[16];
} double16;

// Basic math functions
extern double DClamp(double value, double min, double max);
extern double DLerp(double start, double end, double amount);
extern double DNormalize(double value, double start, double end);
extern double DRemap(double value, double inputStart, double inputEnd, double outputStart, double outputEnd);
extern double DWrap(double value, double min, double max);
extern int DoubleEquals(double x, double y);

// Vector2 math functions
extern DVector2 DVector2Zero(void);
extern DVector2 DVector2One(void);
extern DVector2 DVector2Add(DVector2 v1, DVector2 v2);
extern DVector2 DVector2AddValue(DVector2 v, double add);
extern DVector2 DVector2Subtract(DVector2 v1, DVector2 v2);
extern DVector2 DVector2SubtractValue(DVector2 v, double sub);
extern double DVector2Length(DVector2 v);
extern double DVector2LengthSqr(DVector2 v);
extern double DVector2DotProduct(DVector2 v1, DVector2 v2);
extern double DVector2Distance(DVector2 v1, DVector2 v2);
extern double DVector2DistanceSqr(DVector2 v1, DVector2 v2);
extern double DVector2Angle(DVector2 v1, DVector2 v2);
extern double DVector2LineAngle(DVector2 start, DVector2 end);
extern DVector2 DVector2Scale(DVector2 v, double scale);
extern DVector2 DVector2Multiply(DVector2 v1, DVector2 v2);
extern DVector2 DVector2Negate(DVector2 v);
extern DVector2 DVector2Divide(DVector2 v1, DVector2 v2);
extern DVector2 DVector2Normalize(DVector2 v);
extern DVector2 DVector2Transform(DVector2 v, DMatrix mat);
extern DVector2 DVector2Lerp(DVector2 v1, DVector2 v2, double amount);
extern DVector2 DVector2Reflect(DVector2 v, DVector2 normal);
extern DVector2 DVector2Rotate(DVector2 v, double angle);
extern DVector2 DVector2MoveTowards(DVector2 v, DVector2 target, double maxDistance);
extern DVector2 DVector2Invert(DVector2 v);
extern DVector2 DVector2Clamp(DVector2 v, DVector2 min, DVector2 max);
extern DVector2 DVector2ClampValue(DVector2 v, double min, double max);
extern int DVector2Equals(DVector2 p, DVector2 q);

// Vector3 math functions
extern DVector3 DVector3Zero(void);
extern DVector3 DVector3One(void);
extern DVector3 DVector3Add(DVector3 v1, DVector3 v2);
extern DVector3 DVector3AddValue(DVector3 v, double add);
extern DVector3 DVector3Subtract(DVector3 v1, DVector3 v2);
extern DVector3 DVector3SubtractValue(DVector3 v, double sub);
extern DVector3 DVector3Scale(DVector3 v, double scalar);
extern DVector3 DVector3Multiply(DVector3 v1, DVector3 v2);
extern DVector3 DVector3CrossProduct(DVector3 v1, DVector3 v2);
extern DVector3 DVector3Perpendicular(DVector3 v);
extern double DVector3Length(const DVector3 v);
extern double DVector3LengthSqr(const DVector3 v);
extern double DVector3DotProduct(DVector3 v1, DVector3 v2);
extern double DVector3Distance(DVector3 v1, DVector3 v2);
extern double DVector3DistanceSqr(DVector3 v1, DVector3 v2);
extern double DVector3Angle(DVector3 v1, DVector3 v2);
extern DVector3 DVector3Negate(DVector3 v);
extern DVector3 DVector3Divide(DVector3 v1, DVector3 v2);
extern DVector3 DVector3Normalize(DVector3 v);
extern DVector3 DVector3Project(DVector3 v1, DVector3 v2);
extern DVector3 DVector3Reject(DVector3 v1, DVector3 v2);
extern void DVector3OrthoNormalize(DVector3 *v1, DVector3 *v2);
extern DVector3 DVector3Transform(DVector3 v, DMatrix mat);
extern DVector3 DVector3RotateByDQuaternion(DVector3 v, DQuaternion q);
extern DVector3 DVector3RotateByAxisAngle(DVector3 v, DVector3 axis, double angle);
extern DVector3 DVector3Lerp(DVector3 v1, DVector3 v2, double amount);
extern DVector3 DVector3Reflect(DVector3 v, DVector3 normal);
extern DVector3 DVector3Min(DVector3 v1, DVector3 v2);
extern DVector3 DVector3Max(DVector3 v1, DVector3 v2);
extern DVector3 DVector3Barycenter(DVector3 p, DVector3 a, DVector3 b, DVector3 c);
extern DVector3 DVector3Unproject(DVector3 source, DMatrix projection, DMatrix view);
extern double3 DVector3ToFloatV(DVector3 v);
extern DVector3 DVector3Invert(DVector3 v);
extern DVector3 DVector3Clamp(DVector3 v, DVector3 min, DVector3 max);
extern DVector3 DVector3ClampValue(DVector3 v, double min, double max);
extern int DVector3Equals(DVector3 p, DVector3 q);
extern DVector3 DVector3Refract(DVector3 v, DVector3 n, double r);

// Matrix math functions
extern double DMatrixDeterminant(DMatrix mat);
extern double DMatrixTrace(DMatrix mat);
extern DMatrix DMatrixTranspose(DMatrix mat);
extern DMatrix DMatrixInvert(DMatrix mat);
extern DMatrix DMatrixIdentity(void);
extern DMatrix DMatrixAdd(DMatrix left, DMatrix right);
extern DMatrix DMatrixSubtract(DMatrix left, DMatrix right);
extern DMatrix DMatrixMultiply(DMatrix left, DMatrix right);
extern DMatrix DMatrixTranslate(double x, double y, double z);
extern DMatrix DMatrixRotate(DVector3 axis, double angle);
extern DMatrix DMatrixRotateX(double angle);
extern DMatrix DMatrixRotateY(double angle);
extern DMatrix DMatrixRotateZ(double angle);
extern DMatrix DMatrixRotateXYZ(DVector3 angle);
extern DMatrix DMatrixRotateZYX(DVector3 angle);
extern DMatrix DMatrixScale(double x, double y, double z);
extern DMatrix DMatrixFrustum(double left, double right, double bottom, double top, double near, double far);
extern DMatrix DMatrixPerspective(double fovY, double aspect, double nearPlane, double farPlane);
extern DMatrix DMatrixOrtho(double left, double right, double bottom, double top, double nearPlane, double farPlane);
extern DMatrix DMatrixLookAt(DVector3 eye, DVector3 target, DVector3 up);
extern double16 DMatrixToFloatV(DMatrix mat);

#endif // DMATH_H_CORBIT
