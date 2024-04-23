/*****************************************************************************
 * Copyright (C) 2021-2051 anonymous
 *
 * Authors: anonymous <anonymous@anonymous>
 *****************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "label.h"


#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define MIN(a,b) (((a) < (b)) ? (a) : (b))

void label_background_boudary_pixels(vss_param* params, vss_view* ref_view)
{
    int x, y;

    float max_disp = params->fB / params->min_depth, min_disp = params->fB / params->max_depth;
    float drange = max_disp - min_disp + 1;

    memset(ref_view->labels, HOLE, params->source_width * params->source_height * sizeof(pixel_label));
    for (y = 0; y < params->source_height; y++)
        for (x = 0; x < params->source_width; x++)
        {
            float cval = params->fB / (ref_view->z_depth[y * params->source_width + x] + 1e-5f);

            int rx = MIN(x + 1, params->source_width - 1);
            int dy = MIN(y + 1, params->source_height - 1);

            float dval = params->fB / (ref_view->z_depth[dy * params->source_width + x] + 1e-5f);
            float rval = params->fB / (ref_view->z_depth[y * params->source_width + rx] + 1e-5f);

            float grad_x = cval - rval;
            float grad_y = cval - dval;

            if (fabs(grad_x) >= (params->depth_edge_th*drange))
            {
                int sid = grad_x > 0.f ? 1 : -params->depth_edge_radius; //grad_x > 0.f On the right is the background
                int eid = grad_x > 0.f ? params->depth_edge_radius : 0;
                for (int i = sid; i <= eid; i++)          //Label the background image on the left or right side respectively. Right side 1,4, left side -4, 0
                {
                    int imx = MAX(0, MIN(params->source_width - 1, i + x));
                    ref_view->labels[y * params->source_width + imx] = B_BOUNDAY;
                }
            } // -- process horizontal gradients
            if (fabs(grad_y) >= (params->depth_edge_th*drange))
            {
                int sid = grad_y > 0.f ? 1 : -params->depth_edge_radius;
                int eid = grad_y > 0.f ? params->depth_edge_radius : 0;
                for (int i = sid; i <= eid; i++)
                {
                    int imy = MAX(0, MIN(params->source_height - 1, i + y));
                    ref_view->labels[imy * params->source_width + x] = B_BOUNDAY;
                }
            } // -- process vertical gradients
            if (ref_view->labels[y * params->source_width + x] != B_BOUNDAY)
            {
                ref_view->labels[y * params->source_width + x] = MAIN;
            }
        }
}

void expand_fore_label_pixels(vss_param* params, vss_view* vir_view)
{
    int x, y;
    for (y = 0; y < params->source_height; y++)
        for (x = 0; x < params->source_width; x++)
        {

            int nearFB = 0;
            for (int i = -params->foreground_expand; i <= params->foreground_expand; i++)
            {
                for (int j = -params->foreground_expand; j <= params->foreground_expand; j++)
                {
                    int imy = MAX(0, MIN(y + i, params->source_height - 1));
                    int imx = MAX(0, MIN(x + j, params->source_width - 1));
                    if (vir_view->labels[imy * params->source_width + imx] == F_BOUNDARY)
                    {
                        nearFB = 1;
                        break;
                    }
                }
            } // -- disparity based joint bilateral filtering
            if (nearFB)
            {
                vir_view->filter_label_buf[y * params->source_width + x] = F_BOUNDARY;
            }
            else
            {
                vir_view->filter_label_buf[y * params->source_width + x] = vir_view->labels[y * params->source_width + x];
            }
        }

    memcpy(vir_view->labels, vir_view->filter_label_buf, params->source_width * params->source_height * sizeof(pixel_label));
}

void label_foreground_boudary_pixels(vss_param* params, vss_view* vir_view)
{
    float max_disp = params->fB / params->min_depth, min_disp = params->fB / params->max_depth;
    float drange = max_disp - min_disp + 1;

    int x, y;

    memset(vir_view->labels, HOLE, params->source_width * params->source_height * sizeof(pixel_label));
    for (y = 0; y < params->source_height; y++)
        for (x = 0; x < params->source_width; x++)
        {
            float cval = params->fB / (vir_view->z_depth[y * params->source_width + x] + 1e-5f);

            int rx = MIN(x + 1, params->source_width - 1);
            int dy = MIN(y + 1, params->source_height - 1);

            float dval = params->fB / (vir_view->z_depth[dy * params->source_width + x] + 1e-5f);
            float rval = params->fB / (vir_view->z_depth[y * params->source_width + rx] + 1e-5f);

            float grad_x = cval - rval;
            float grad_y = cval - dval;

            if (fabs(grad_x) >= (params->depth_edge_th*drange))
            {
                int sid = grad_x > 0.f ? -params->foreground_edge_radius : 1;
                int eid = grad_x > 0.f ? 0 : params->foreground_edge_radius;
                for (int i = sid; i <= eid; i++)
                {
                    int imx = MAX(0, MIN(params->source_width - 1, i + x));
                    vir_view->labels[y * params->source_width + imx] = F_BOUNDARY;
                }
            } // -- process horizontal gradients
            if (fabs(grad_y) >= (params->depth_edge_th*drange))
            {
                int sid = grad_y > 0.f ? -params->foreground_edge_radius : 1;
                int eid = grad_y > 0.f ? 0 : params->foreground_edge_radius;
                for (int i = sid; i <= eid; i++)
                {
                    int imy = MAX(0, MIN(params->source_height - 1, i + y));
                    vir_view->labels[imy * params->source_width + x] = F_BOUNDARY;
                }
            } // -- process vertical gradients

            if (vir_view->labels[y * params->source_width + x] != F_BOUNDARY)
            {
                vir_view->labels[y * params->source_width + x] = MAIN;
            }
        }

    expand_fore_label_pixels(params, vir_view); // -- expand foreground boundary
}