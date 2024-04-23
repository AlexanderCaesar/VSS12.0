/*****************************************************************************
 * Copyright (C) 2021-2051 anonymous
 *
 * Authors: anonymous <anonymous@anonymous>
 *****************************************************************************/

#include <math.h>
#include "vss.h"


#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define MIN(a,b) (((a) < (b)) ? (a) : (b))

void merge_warp_views(vss_param* params, vss_view* warp_views, vss_view* vir_view)
{
    int x, y;
    float dist[2] = { 0.f,0.f };
    float dist_weight[2] = {
        expf(-dist[0] / 5.f),
        expf(-dist[1] / 5.f)
    };

    for (y = 0; y < params->source_height; y++)
        for (x = 0; x < params->source_width; x++)
        {
            float bgr[3] = { 0.f, 0.f, 0.f };
            float sumW = 0.f, sumR = 0.f, min_depth = 1e10f;

            for (int i = 0; i < REF_CAM_N; i++)
            {
                if (params->use_label && (warp_views[i].labels[y * params->source_width + x] != MAIN))
                {
                    continue;
                }

                float depth = warp_views[i].z_depth[y * params->source_width + x];
                float conf = warp_views[i].mconf[y * params->source_width + x] * dist_weight[i];
                sumW += conf;
                sumR += depth;
                min_depth = depth < min_depth ? depth : min_depth;

                for (int chan = 0; chan < 3; chan++)
                {
                    bgr[chan] += (conf *warp_views[i].texture[y * params->source_width * 3 + x * 3 + chan]);
                }

            }

            if (sumW < 1e-5f) { continue; }

            for (int chan = 0; chan < 3; chan++)
            {
                vir_view->texture[y * params->source_width * 3 + x * 3 + chan] = (unsigned char)(MAX(0.f, MIN(255.f, bgr[chan] / sumW)));
            }

            vir_view->labels[y * params->source_width + x] = MAIN;
            vir_view->z_depth[y * params->source_width + x] = min_depth;

        }
}
