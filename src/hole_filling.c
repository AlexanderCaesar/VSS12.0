/*****************************************************************************
 * Copyright (C) 2021-2051 anonymous
 *
 * Authors: anonymous <anonymous@anonymous>
 *****************************************************************************/

#include <math.h>
#include <string.h>
#include "vss.h"


#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define MIN(a,b) (((a) < (b)) ? (a) : (b))

void bilateral_hole_filling(vss_param* params, vss_view* vir_view)
{
    float max_disp = params->fB / params->min_depth, min_disp = params->fB / params->max_depth;
    float drange = max_disp - min_disp + 1;

    int x, y;
    for (y = 0; y < params->source_height; y++)
        for (x = 0; x < params->source_width; x++)
        {

            if (vir_view->labels[y * params->source_width + x] != HOLE)
            {
                vir_view->merge_label_buf[y * params->source_width + x] = vir_view->labels[y * params->source_width + x];
                continue;
            }

            vir_view->z_depth[y * params->source_width + x] = 0.0f;
            vir_view->merge_label_buf[y * params->source_width + x] = HOLE;

            float min_disp = 1e10f;
            int min_dist = 0;
            // -- move along 4 directions to find the smallest disparity
            for (int movx = x + 1; movx < params->source_width; movx++)
            {
                if (vir_view->labels[y * params->source_width + movx] != HOLE)
                {
                    float disp = params->fB / vir_view->z_depth[y * params->source_width + movx];
                    if (disp < min_disp)
                    {
                        min_disp = disp; min_dist = abs(movx - x);
                    }
                    break;
                }
            } // -- move horizontally

            for (int movx = x - 1; movx >= 0; movx--)
            {
                if (vir_view->labels[y * params->source_width + movx] != HOLE)
                {
                    float disp = params->fB / vir_view->z_depth[y * params->source_width + movx];
                    if (disp < min_disp)
                    {
                        min_disp = disp; min_dist = abs(movx - x);
                    }
                    break;
                }
            } // -- move horizontally

            for (int movy = y + 1; movy < params->source_height; movy++)
            {
                if (vir_view->labels[movy * params->source_width + x] != HOLE)
                {
                    float disp = params->fB / vir_view->z_depth[movy * params->source_width + x];
                    if (disp < min_disp)
                    {
                        min_disp = disp; min_dist = abs(movy - y);
                    }
                    break;
                }
            } // -- move vertically

            for (int movy = y - 1; movy >= 0; movy--)
            {
                if (vir_view->labels[movy * params->source_width + x] != HOLE)
                {
                    float disp = params->fB / vir_view->z_depth[movy * params->source_width + x];
                    if (disp < min_disp)
                    {
                        min_disp = disp; min_dist = abs(movy - y);
                    }
                    break;
                }
            } // -- move vertically

            if (min_disp > 1e9f) { continue; }

            float sumC[] = { 0.f, 0.f, 0.f };
            float sumW = 0.f;
            const float sigs = (float)(params->hole_radius * 1.2f);
            for (int i = -params->hole_radius; i <= params->hole_radius; i++) {
                for (int j = -params->hole_radius; j <= params->hole_radius; j++)
                {
                    int imy = MAX(0, MIN(y + i, params->source_height - 1));
                    int imx = MAX(0, MIN(x + j, params->source_width - 1));
                    if (vir_view->labels[imy * params->source_width + imx] == HOLE)
                    {
                        continue;
                    }
                    float dist = sqrtf((float)(i * i + j * j));
                    float ws = expf(-dist / sigs);

                    float disp = params->fB / vir_view->z_depth[imy * params->source_width + imx];
                    float wd = expf(-((float)fabs(disp - min_disp)) / (params->depth_edge_th*drange));

                    for (int c = 0; c < 3; c++)
                    {
                        float d = vir_view->texture[imy * params->source_width * 3 + imx * 3 + c];
                        sumC[c] += (ws * wd * d);
                    }
                    sumW += (ws * wd);
                }
            } // -- disparity based joint bilateral filtering

            //if (sumW < 1e-5f) { return; }
            if (sumW < 1e-5f) {
                continue;
            }
            for (int c = 0; c < 3; c++)
            {
                vir_view->texture[y * params->source_width * 3 + x * 3 + c] = (unsigned char)(sumC[c] / sumW);
            }
            vir_view->z_depth[y * params->source_width + x] = params->fB / min_disp;
            vir_view->merge_label_buf[y * params->source_width + x] = MAIN;
        }

    memcpy(vir_view->labels, vir_view->merge_label_buf, params->source_width * params->source_height * sizeof(pixel_label));
}

int hole_filling(vss_param* params, vss_view* vir_view)
{

    for (int iter = 0; iter < params->hole_iters; iter++)
        bilateral_hole_filling(params, vir_view);

    return VSS_SUCCESS;
}