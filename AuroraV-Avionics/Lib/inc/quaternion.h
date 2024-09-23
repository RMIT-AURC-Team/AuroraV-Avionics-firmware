#ifndef _QUATERNION_H
#define _QUATERNION_H

#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#ifndef M_PI
#define M_PI 3.14159265
#endif

typedef struct Quaternion {
  float w;
  float x;
  float y;
  float z;
  void (*init)(struct Quaternion *);
  void (*normalise)(struct Quaternion *);
  void (*fromEuler)(struct Quaternion *, float, float, float);
  void (*fRotateVector3D)(struct Quaternion *, float *, float *);
} Quaternion;

Quaternion Quaternion_mul(Quaternion *, Quaternion *);
void Quaternion_init(Quaternion *);
void Quaternion_normalise(Quaternion *);
void Quaternion_fromEuler(Quaternion *, float, float, float);
void Quaternion_fRotateVector3D(Quaternion *, float *, float *);

#endif
