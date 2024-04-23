/*****************************************************************************
 * Copyright (C) 2021-2051 anonymous
 *
 * Authors: anonymous <anonymous@anonymous>
 *****************************************************************************/

#ifndef __VSS_H__
#define __VSS_H__

#include<stdio.h>

#define MAX_SFM_CAMNUM (50) // default 200   -- MAXimum support sfm image number, 2K
#define CV_PI       3.14159265358979323846
#define REF_CAM_N   2    //ref cameras or frames used to interpolate

#define VSS_SUCCESS    1
#define VSS_ERR        0 

typedef enum {

    HOLE = 0,
    MAIN,
    F_BOUNDARY,
    B_BOUNDAY
} pixel_label;


typedef enum {
    FISHEYE = 0,
    PINHOLE = 1,
} Model;


typedef struct cam_param
{
    float          krt_R[9];             //parameters Rotation matrix
    float          krt_WorldPosition[3];
    float          krt_kc[3];            //FISHEYE K 
    float          krt_cc[2];
    int            width;
    int            height;
    float          lens_fov;
    float          fisheye_radius; 

    float          krt_K[9];             //parameters Rotation matrix
}cam_param;

typedef struct
{
    int            cam_num;
    int            source_width;
    int            source_height;
    float          min_depth;  
    float          max_depth;   
    int            depth_scale_type;             // 0 full 1 1/2 downsample 2 1/4 downsample in width
    int            background_flag;
    int            background_th;                //background threshold
    float          depth_scale_th;               //depth_scale threshold
    int            depth_mf_radius;              //Median filtering radius
    int            vir_depth_filter_type;
    int            vir_depth_filter_radius;
    int            vir_interpolation_type;
    int            lanczos_alpha;
    float          merge_th;
    int            hole_radius;
    int            hole_th;
    int            hole_iters;
    int            foreground_edge_radius;
    int            foreground_expand;
    int            smooth_foreground_radius;
    float          smooth_foreground_sig;

    int            left_view;
    int            right_view;

    int            model;
    float          fB;
    int            use_label;
    float          depth_edge_th;
    int            depth_edge_radius;
    int            inverse_warp;
    int            expand_vir_depth;


   

    cam_param      krt_vcam;                     // virtual  camera

    cam_param      krt_rcam[MAX_SFM_CAMNUM];     // reference cameras

} vss_param;


typedef struct vss_view
{
    int            view_num;
    unsigned char* texture;
    unsigned char* depth;
    float*         z_depth;   //real depth

    unsigned char* texture_bg;//background frame
    unsigned char* depth_bg;  //background frame

    unsigned char* texture_y;  //The luMINance component of a texture view  for down sample type
    unsigned char* depth_y;    //The luMINance component of a depth map  for down sample type

    pixel_label*   labels;
    float*         theta_table;

    float*         mconf;
    float          pre_weight;

    float*         filter_buf;
    pixel_label*   filter_label_buf;
    unsigned char* texture_fore_smooth;

    float*         ws_array_buf;
    float*         wd_array_buf;

    pixel_label*   merge_label_buf;


}vss_view;


int  vss_open(vss_view* view, vss_view* warp_view, vss_param* param);
int  vss_close(vss_view* views, vss_view* warp_view, vss_param* param);
int  vss_syn_view(vss_param* params, vss_view* ref_view, vss_view* warp_view);
int  vss_clear_vir_view(vss_view* vir_view, vss_param* param);
void merge_warp_views(vss_param* params, vss_view* warp_views, vss_view* vir_view);
int  hole_filling(vss_param* params, vss_view* vir_view);
int  filter_foreground(vss_param* params, vss_view* vir_view);



#endif