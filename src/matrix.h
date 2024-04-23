/*****************************************************************************
 * Copyright (C) 2021-2051 anonymous
 *
 * Authors: anonymous <anonymous@anonymous>
 *****************************************************************************/

#ifndef __VSS_MATRIX_H__
#define __VSS_MATRIX_H__

#include<stdio.h>
#include"vss.h"

void matrix_inverse_or_rotate_3x3(float *m, float *lhs);  // for rotate matrix, its inverse is just its rotate.

void matrix_mult_3x3_by_3x3(float *m, float *lhs, float *rhs);

void vec3x1_sub(float* dst, float* mat3x1, float* vec3x1);

void matrix_mult_3x3_by_3x1(float* dst, float* mat3x3, float* vec3x1);

void euclidean_transform_point(float* newP, float* R, float* P, float *t);

#endif