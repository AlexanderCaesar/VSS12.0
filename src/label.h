/*****************************************************************************
 * Copyright (C) 2021-2051 anonymous
 *
 * Authors: anonymous <anonymous@anonymous>
 *****************************************************************************/

#ifndef __VSS_LABEL_H__
#define __VSS_LABEL_H__

#include<stdio.h>
#include"vss.h"

void label_background_boudary_pixels(vss_param* params, vss_view* ref_view);
void label_foreground_boudary_pixels(vss_param* params, vss_view* vir_view);

#endif