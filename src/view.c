/*****************************************************************************
 * Copyright (C) 2021-2051 anonymous
 *
 * Authors: anonymous <anonymous@anonymous>
 *****************************************************************************/
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "vss.h"
#include "filter.h"
#include "fisheye.h"
#include "pinhole.h"
#include "depth.h"
#include "label.h"

int vss_open(vss_view* views, vss_view* warp_view, vss_param* param)
{
    init_lanczos_filter();
    for (int i = 0; i <= param->cam_num; i++) //The last file list is a virtual view 
    {
        views[i].view_num = i;
        views[i].texture = (unsigned char *)malloc(param->source_width * param->source_height * 3 * sizeof(unsigned char));
        if (!views[i].texture)
        {
            printf("malloc texture failed!\n");
            return -1;
        }

        if (param->use_label)
        {
            views[i].labels = (pixel_label *)malloc(param->source_width * param->source_height * sizeof(pixel_label));
            if (!views[i].labels)
            {
                printf("malloc labels failed!\n");
                return -1;
            }
        }

        if (param->model == FISHEYE)
        {
            int theta_len = (int)(param->krt_rcam[i].fisheye_radius * 2.f);
            views[i].theta_table = (float*)malloc(theta_len * sizeof(float));
            if (!views[i].theta_table)
            {
                printf("malloc theta_table failed!\n");
                return -1;
            }
        }

        if (param->depth_scale_type)
        {
            views[i].texture_y = (unsigned char *)malloc(param->source_width * param->source_height * sizeof(unsigned char));
            if (!views[i].texture_y)
            {
                printf("malloc texture failed!\n");
                return -1;
            }
        }
        if (param->depth_scale_type || param->depth_mf_radius)
        {
            views[i].depth_y = (unsigned char *)malloc(param->source_width * param->source_height * sizeof(unsigned char));
            if (!views[i].depth_y)
            {
                printf("malloc texture failed!\n");
                return -1;
            }
        }
        views[i].depth = (unsigned char *)malloc(param->source_width * param->source_height * 3 * sizeof(unsigned char));
        if (!views[i].depth)
        {
            printf("malloc depth failed!\n");
            return -1;
        }
        views[i].z_depth = (float *)malloc(param->source_width * param->source_height * 3 * sizeof(float));
        if (!views[i].z_depth)
        {
            printf("malloc z_depth failed!\n");
            return -1;
        }
        if (param->background_flag)
        {
            views[i].texture_bg = (unsigned char *)malloc(param->source_width * param->source_height * 3 * sizeof(unsigned char));
            if (!views[i].texture_bg)
            {
                printf("malloc texture_bg failed!\n");
                return -1;
            }

            views[i].depth_bg = (unsigned char *)malloc(param->source_width * param->source_height * 3 * sizeof(unsigned char));
            if (!views[i].depth_bg)
            {
                printf("malloc depth_bg failed!\n");
                return -1;
            }
        }
    }

    views[param->cam_num].merge_label_buf = (pixel_label*)malloc(param->source_width * param->source_height * sizeof(pixel_label));
    if (!views[param->cam_num].merge_label_buf)
    {
        printf("malloc merge_label_buf failed!\n");
        return -1;
    }

    views[param->cam_num].labels = (pixel_label *)malloc(param->source_width * param->source_height * sizeof(pixel_label));
    if (!views[param->cam_num].labels)
    {
        printf("malloc labels failed!\n");
        return -1;
    }

    if (param->foreground_edge_radius)
    {
        views[param->cam_num].filter_buf = (float*)malloc(param->source_width * param->source_height * sizeof(float));
        if (!views[param->cam_num].filter_buf)
        {
            printf("malloc vir filter_buf failed!\n");
            return -1;
        }
        views[param->cam_num].filter_label_buf = (pixel_label*)malloc(param->source_width * param->source_height * sizeof(pixel_label));
        if (!views[param->cam_num].filter_label_buf)
        {
            printf("malloc vir filter_label_buf failed!\n");
            return -1;
        }

        views[param->cam_num].texture_fore_smooth = (unsigned char*)malloc(param->source_width * param->source_height * 3* sizeof(unsigned char));
        if (!views[param->cam_num].texture_fore_smooth)
        {
            printf("malloc vir texture_fore_smooth failed!\n");
            return -1;
        }       
    }

    for (int i = 0; i < REF_CAM_N; i++)
    {
        warp_view[i].view_num = i;
        warp_view[i].texture = (unsigned char *)malloc(param->source_width * param->source_height * 3 * sizeof(unsigned char));
        if (!warp_view[i].texture)
        {
            printf("malloc warp_view texture failed!\n");
            return -1;
        }

        memset(warp_view[i].texture, 0, param->source_width * param->source_height * 3 * sizeof(unsigned char));

        if (param->use_label)
        {
            warp_view[i].labels = (pixel_label *)malloc(param->source_width * param->source_height * sizeof(pixel_label));
            if (!warp_view[i].labels)
            {
                printf("malloc warp_view labels failed!\n");
                return -1;
            }
            memset(warp_view[i].labels, HOLE, param->source_width * param->source_height * sizeof(float));
        }
        warp_view[i].z_depth = (float *)malloc(param->source_width * param->source_height * sizeof(float));
        if (!warp_view[i].z_depth)
        {
            printf("malloc warp_view z_depth failed!\n");
            return -1;
        }
        memset(warp_view[i].z_depth, 0, param->source_width * param->source_height * sizeof(float));
        warp_view[i].mconf = (float *)malloc(param->source_width * param->source_height * sizeof(float));
        if (!warp_view[i].mconf)
        {
            printf("malloc warp_view mconf failed!\n");
            return -1;
        }
        memset(warp_view[i].mconf, 0, param->source_width * param->source_height * sizeof(float));

        warp_view[i].pre_weight = 1.0;

        if (param->model == FISHEYE)
        {
            int theta_len = (int)(param->krt_vcam.fisheye_radius * 2.f);
            warp_view[i].theta_table = (float*)malloc(theta_len * sizeof(float));
            if (!warp_view[i].theta_table)
            {
                printf("malloc theta_table failed!\n");
                return -1;
            }
        }

        if (param->vir_depth_filter_type == 2)
        {
            warp_view[i].ws_array_buf=(float*)malloc((param->vir_depth_filter_radius *param->vir_depth_filter_radius * 2 + 1) * sizeof(float));
            if (!warp_view[i].ws_array_buf)
            {
                printf("malloc ws_array_buf failed!\n");
                return -1;
            }

            float max_disp = param->fB / param->min_depth, min_disp = param->fB / param->max_depth;
            float drange = max_disp - min_disp + 1;

            int wd_array_len = (int)(drange * 2.f);
            warp_view[i].wd_array_buf = (float*)malloc(wd_array_len * sizeof(float));
            if (!warp_view[i].wd_array_buf)
            {
                printf("malloc wd_array_buf failed!\n");
                return -1;
            }
        }

        if (param->vir_depth_filter_type)
        {
            warp_view[i].filter_buf = (float*)malloc(param->source_width * param->source_height * sizeof(float));
            if (!warp_view[i].filter_buf)
            {
                printf("malloc filter_buf failed!\n");
                return -1;
            }
            warp_view[i].filter_label_buf = (pixel_label*)malloc(param->source_width * param->source_height * sizeof(pixel_label));
            if (!warp_view[i].filter_label_buf)
            {
                printf("malloc filter_label_buf failed!\n");
                return -1;
            }
        }

    }
    return 0;
}
int vss_close(vss_view* views, vss_view* warp_view, vss_param* param)
{
    for (int i = 0; i <= param->cam_num; i++)
    {
        if (views[i].texture)
        {
            free(views[i].texture);
            views[i].texture = NULL;
        }

        if (param->use_label)
        {
            if (views[i].labels)
            {
                free(views[i].labels);
                views[i].labels = NULL;
            }
        }

        if (param->model == FISHEYE)
        {
            if (views[i].theta_table)
            {
                free(views[i].theta_table);
                views[i].theta_table = NULL;
            }
        }

        if (param->depth_scale_type)
        {
            if (views[i].texture_y)
            {
                free(views[i].texture_y);
                views[i].texture_y = NULL;
            }
        }

        if (param->depth_scale_type || param->depth_mf_radius)
        {
            if (views[i].depth_y)
            {
                free(views[i].depth_y);
                views[i].depth_y = NULL;
            }
        }

        if (views[i].depth)
        {
            free(views[i].depth);
            views[i].depth = NULL;
        }
        if (views[i].z_depth)
        {
            free(views[i].z_depth);
            views[i].z_depth = NULL;
        }
        if (param->background_flag)
        {
            if(views[i].texture_bg)
            {
                free(views[i].texture_bg);
                views[i].texture_bg = NULL;
            }

            if(views[i].depth_bg)
            {
                free(views[i].depth_bg);
                views[i].depth_bg = NULL;
            }
        }
    }

    if (views[param->cam_num].merge_label_buf)
    {
        free(views[param->cam_num].merge_label_buf);
        views[param->cam_num].merge_label_buf = NULL;
    }

    if (param->foreground_edge_radius)
    {
        if (views[param->cam_num].filter_buf)
        {
            free(views[param->cam_num].filter_buf);
            views[param->cam_num].filter_buf = NULL;
        }

        if (views[param->cam_num].filter_label_buf)
        {
            free(views[param->cam_num].filter_label_buf);
            views[param->cam_num].filter_label_buf = NULL;
        }

        if (views[param->cam_num].texture_fore_smooth)
        {
            free(views[param->cam_num].texture_fore_smooth);
            views[param->cam_num].texture_fore_smooth = NULL;
        }
    }

    if (views[param->cam_num].labels)
    {
        free(views[param->cam_num].labels);
        views[param->cam_num].labels = NULL;
    }

    for (int i = 0; i < REF_CAM_N; i++)
    {
        if (warp_view[i].texture)
        {
            free(warp_view[i].texture);
            warp_view[i].texture = NULL;
        }

        if (param->use_label)
        {
            if (warp_view[i].labels)
            {
                free(warp_view[i].labels);
                warp_view[i].labels = NULL;
            }
        }

        if (warp_view[i].z_depth)
        {
            free(warp_view[i].z_depth);
            warp_view[i].z_depth = NULL;
        }

        if (warp_view[i].mconf)
        {
            free(warp_view[i].mconf);
            warp_view[i].mconf = NULL;
        }

        if (param->model == FISHEYE)
        {
            if (warp_view[i].theta_table)
            {
                free(warp_view[i].theta_table);
                warp_view[i].theta_table = NULL;
            }
        }

        if (param->vir_depth_filter_type)
        {
            if (warp_view[i].filter_buf)
            {
                free(warp_view[i].filter_buf);
                warp_view[i].filter_buf = NULL;
            }
            if (warp_view[i].filter_label_buf)
            {
                free(warp_view[i].filter_label_buf);
                warp_view[i].filter_label_buf = NULL;
            }
        }

        if (param->vir_depth_filter_type == 2)
        {
            if (warp_view[i].ws_array_buf)
            {
                free(warp_view[i].ws_array_buf);
                warp_view[i].ws_array_buf = NULL;
            }

            if (warp_view[i].wd_array_buf)
            {
                free(warp_view[i].wd_array_buf);
                warp_view[i].wd_array_buf = NULL;
            }
        }
    }
    
    return 0;
}


int vss_clear_warp_view(vss_view* warp_view, vss_param* param)
{
    memset(warp_view->texture, 0, param->source_width * param->source_height * 3 * sizeof(unsigned char));
    if (param->use_label)
    {
        memset(warp_view->labels, HOLE, param->source_width * param->source_height * sizeof(float));
    }

    memset(warp_view->z_depth, 0, param->source_width * param->source_height * sizeof(float));
    memset(warp_view->mconf, 0, param->source_width * param->source_height * sizeof(float));

    warp_view->pre_weight = 1.0;

    return 0;
}

int vss_syn_view(vss_param* params, vss_view* ref_view, vss_view* warp_view)
{
    vss_clear_warp_view(warp_view, params);
    process_ref_depth(ref_view, params);

    /*label_boundary*/
    if (params->use_label)
        label_background_boudary_pixels(params, ref_view);

    if (params->model == FISHEYE)
    {
        set_theta_table(params, ref_view, warp_view);
    }

    int width = params->source_width;
    int height = params->source_height;

    cam_param* ref_cam = &(params->krt_rcam[ref_view->view_num]);

    clock_t st;

    st = clock();

    if (params->model == FISHEYE)
    {
        warp_ref_depth_fisheye(params, ref_view, warp_view);
    }
    else
    {
        warp_ref_depth_pinhole(params, ref_view, warp_view);
    }

    printf("warp_ref_depth time = %f\n", (double)(clock() - st) / CLOCKS_PER_SEC);

    st = clock();

    if (params->use_label)
    {
        if (params->vir_depth_filter_type == 1)
        {
            median_vir_depth(params, warp_view);
        }
        else if (params->vir_depth_filter_type == 2)
        {
            bilateral_vir_depth(params, warp_view);
        }
    }

    printf("filter vir depth time = %f\n", (double)(clock() - st) / CLOCKS_PER_SEC);

    st = clock();

    if (params->inverse_warp)
    {
        if (params->model == FISHEYE)
            generate_vir_view_fisheye(params, ref_view, warp_view);
        else
            generate_vir_view_pinhole(params, ref_view, warp_view);
    }
    
    printf("generate_vir_view time = %f\n", (double)(clock() - st) / CLOCKS_PER_SEC);


    return VSS_SUCCESS;
}

int vss_clear_vir_view(vss_view* vir_view, vss_param* param)
{
    memset(vir_view->texture, 0, param->source_width * param->source_height * 3 * sizeof(unsigned char));

    memset(vir_view->labels, HOLE, param->source_width * param->source_height * sizeof(float));

    memset(vir_view->z_depth, 0, param->source_width * param->source_height * sizeof(float));
    return 0;
}