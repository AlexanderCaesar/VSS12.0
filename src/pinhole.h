/*****************************************************************************
 * Copyright (C) 2021-2051 anonymous
 *
 * Authors: anonymous <anonymous@anonymous>
 *****************************************************************************/

#ifndef __VSS_PINHOLE_H__
#define __VSS_PINHOLE_H__

#include<stdio.h>
#include"vss.h"

int   warp_ref_depth_pinhole(vss_param* params, vss_view* ref_view, vss_view* warp_view);
void  generate_vir_view_pinhole(vss_param* params, vss_view* ref_view, vss_view* warp_view);

#endif