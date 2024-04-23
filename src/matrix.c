/*****************************************************************************
 * Copyright (C) 2021-2051 anonymous
 *
 * Authors: anonymous <anonymous@anonymous>
 *****************************************************************************/

#include <string.h>
#include <math.h>
#include "matrix.h"


 /*Transposed matrix*/
void matrix_inverse_or_rotate_3x3(float *m, float *lhs)  // for rotate matrix, its inverse is just its rotate.
{
    int i, j;

    for (j = 0; j < 3; j++)
    {
        for (i = 0; i < 3; i++)
        {
            m[3 * j + i] = lhs[3 * i + j];
        }
    }
}

#define I(_i, _j) ((_j)+3*(_i))


void matrix_mult_3x3_by_3x3(float *m, float *lhs, float *rhs)
{
    // m = rhs * lhs

    m[0] = lhs[I(0, 0)] * rhs[I(0, 0)] + lhs[I(0, 1)] * rhs[I(1, 0)] + lhs[I(0, 2)] * rhs[I(2, 0)];
    m[1] = lhs[I(0, 0)] * rhs[I(0, 1)] + lhs[I(0, 1)] * rhs[I(1, 1)] + lhs[I(0, 2)] * rhs[I(2, 1)];
    m[2] = lhs[I(0, 0)] * rhs[I(0, 2)] + lhs[I(0, 1)] * rhs[I(1, 2)] + lhs[I(0, 2)] * rhs[I(2, 2)];

    m[3] = lhs[I(1, 0)] * rhs[I(0, 0)] + lhs[I(1, 1)] * rhs[I(1, 0)] + lhs[I(1, 2)] * rhs[I(2, 0)];
    m[4] = lhs[I(1, 0)] * rhs[I(0, 1)] + lhs[I(1, 1)] * rhs[I(1, 1)] + lhs[I(1, 2)] * rhs[I(2, 1)];
    m[5] = lhs[I(1, 0)] * rhs[I(0, 2)] + lhs[I(1, 1)] * rhs[I(1, 2)] + lhs[I(1, 2)] * rhs[I(2, 2)];

    m[6] = lhs[I(2, 0)] * rhs[I(0, 0)] + lhs[I(2, 1)] * rhs[I(1, 0)] + lhs[I(2, 2)] * rhs[I(2, 0)];
    m[7] = lhs[I(2, 0)] * rhs[I(0, 1)] + lhs[I(2, 1)] * rhs[I(1, 1)] + lhs[I(2, 2)] * rhs[I(2, 1)];
    m[8] = lhs[I(2, 0)] * rhs[I(0, 2)] + lhs[I(2, 1)] * rhs[I(1, 2)] + lhs[I(2, 2)] * rhs[I(2, 2)];
}


void vec3x1_sub(float* dst, float* mat3x1, float* vec3x1)
{
    float t[3];
    t[0] = mat3x1[0] - vec3x1[0];
    t[1] = mat3x1[1] - vec3x1[1];
    t[2] = mat3x1[2] - vec3x1[2];
    memcpy(dst, t, sizeof(t));
}

void matrix_mult_3x3_by_3x1(float* dst, float* mat3x3, float* vec3x1)
{
    float t[3];

    t[0] = mat3x3[0] * vec3x1[0] + mat3x3[1] * vec3x1[1] + mat3x3[2] * vec3x1[2];
    t[1] = mat3x3[3] * vec3x1[0] + mat3x3[4] * vec3x1[1] + mat3x3[5] * vec3x1[2];
    t[2] = mat3x3[6] * vec3x1[0] + mat3x3[7] * vec3x1[1] + mat3x3[8] * vec3x1[2];
    memcpy(dst, t, sizeof(t));
}

/* X* R + T */
void euclidean_transform_point(float* newP, float* R, float* P, float *t)
{
    float Rp[3];
    matrix_mult_3x3_by_3x1(Rp, R, P);
    newP[0] = Rp[0] + t[0];
    newP[1] = Rp[1] + t[1];
    newP[2] = Rp[2] + t[2];
}