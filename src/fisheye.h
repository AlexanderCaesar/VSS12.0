/*****************************************************************************
 * Copyright (C) 2021-2051 anonymous
 *
 * Authors: anonymous <anonymous@anonymous>
 *****************************************************************************/

#ifndef __VSS_FISHEYE_H__
#define __VSS_FISHEYE_H__

#include<stdio.h>
#include"vss.h"

void  set_theta_table(vss_param* params, vss_view* ref_view, vss_view* warp_view);
float get_theta(float* arr_rtheta, int length_rtheta, float fisheye_radius, float r);

int   warp_ref_depth_fisheye(vss_param* params, vss_view* ref_view, vss_view* warp_view);
void  generate_vir_view_fisheye(vss_param* params, vss_view* ref_view, vss_view* warp_view);

#endif