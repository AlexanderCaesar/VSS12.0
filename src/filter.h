/*****************************************************************************
 * Copyright (C) 2021-2051 anonymous
 *
 * Authors: anonymous <anonymous@anonymous>
 *****************************************************************************/

#ifndef __VSS_FILTER_H__
#define __VSS_FILTER_H__

void  init_lanczos_filter();
unsigned char filtre_lanczos(unsigned char * src, float j, float i, int input_width, int input_height, int i_src, int RGB_offset);
void  median_filter(unsigned char * src, unsigned char * buf, int width, int height, int kradius);

int median_vir_depth(vss_param* params, vss_view* warp_view);
int bilateral_vir_depth(vss_param* params, vss_view* warp_view);

#endif