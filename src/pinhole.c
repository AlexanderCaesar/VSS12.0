/*****************************************************************************
 * Copyright (C) 2021-2051 anonymous
 *
 * Authors: anonymous <anonymous@anonymous>
 *****************************************************************************/

#include <string.h>
#include <math.h>
#include "pinhole.h"
#include "filter.h"
#include "matrix.h"

#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define MIN(a,b) (((a) < (b)) ? (a) : (b))

static void Get3DPoint(cam_param*  camera, int x, int y, const float depth, float *X)
{
    X[0] = depth * (x - camera->krt_K[2]) / camera->krt_K[0];
    X[1] = depth * (y - camera->krt_K[5]) / camera->krt_K[4];
    X[2] = depth;
}

static void Get2DPoint(cam_param*  camera, float *x, float *y, float *X)
{
    *x = (camera->krt_K[0] * X[0] + camera->krt_K[2] * X[2]) / X[2];
    *y = (camera->krt_K[4] * X[1] + camera->krt_K[5] * X[2]) / X[2];
}

int warp_ref_depth_pinhole(vss_param* params, vss_view* ref_view, vss_view* warp_view)
{
    cam_param* ref_cam = &(params->krt_rcam[ref_view->view_num]);

    float p3D[3] = { 0, 0, 0 };
    float krt_R[9];

    /*Camera coordinate conversion*/
    /*From the reference camera to the virtual camera*/
    float inv_ref_R[9];
    matrix_inverse_or_rotate_3x3(inv_ref_R, ref_cam->krt_R);
    matrix_mult_3x3_by_3x3(krt_R, params->krt_vcam.krt_R, inv_ref_R);               // krt_R = vcam->krt_R * inv_ref_R

    /*To calculate Translation matrix T = Rv*(ccam->krt_WorldPosition - vcam->krt_WorldPosition)*/
    float krt_t[3];
    vec3x1_sub(krt_t, ref_cam->krt_WorldPosition, params->krt_vcam.krt_WorldPosition); // Tref - Tv
    matrix_mult_3x3_by_3x1(krt_t, params->krt_vcam.krt_R, krt_t);                      //  T = Rv*(Tref - Tv)

    for (int i = 0; i < params->source_height; i++)
    {
        for (int j = 0; j < params->source_width; j++)
        {
            if (params->use_label && (ref_view->labels[i * params->source_width + j] != MAIN))
                continue; // project novel_pixel_main

            float z_depth = ref_view->z_depth[i * params->source_width + j];

            if (z_depth <= 0.f) { continue; }

            Get3DPoint(ref_cam, j, i, z_depth, p3D);

            float vp3D[3];

            euclidean_transform_point(vp3D, krt_R, p3D, krt_t); //X*R + T;  

            float xpf = 0.0f;
            float ypf = 0.0f;

            Get2DPoint(&params->krt_vcam, &xpf, &ypf, vp3D);

            int xp = (int)roundf(xpf);
            int yp = (int)roundf(ypf);

            if ( xp >= 1 && xp <= (params->source_width - 1) - 1 && yp >= 1 && yp <= (params->source_height - 1) - 1)   //because 3x3 
            {
                float vir_depth;

                vir_depth = vp3D[2]; //step 4:					

                if (params->expand_vir_depth)
                {
                    for (int m = yp - 1; m <= yp + 1; m++)
                    {
                        for (int n = xp - 1; n <= xp + 1; n++)
                        {
                            {
                                float ndepth = warp_view->z_depth[m * params->source_width + n];
                                if (ndepth == 0.f)
                                {
                                    warp_view->z_depth[m * params->source_width + n] = vir_depth;
                                    if (params->use_label)
                                        warp_view->labels[m * params->source_width + n] = MAIN;
                                    if (!params->inverse_warp)
                                    {
                                        warp_view->texture[m * params->source_width * 3 + n * 3 + 0] = ref_view->texture[i * params->source_width * 3 + j * 3 + 0];
                                        warp_view->texture[m * params->source_width * 3 + n * 3 + 1] = ref_view->texture[i * params->source_width * 3 + j * 3 + 1];
                                        warp_view->texture[m * params->source_width * 3 + n * 3 + 2] = ref_view->texture[i * params->source_width * 3 + j * 3 + 2];
                                        warp_view->mconf[m * params->source_width + n] = 1.0;
                                    }
                                }
                                else if (vir_depth < ndepth)
                                {
                                    warp_view->z_depth[m * params->source_width + n] = vir_depth;
                                    if (params->use_label)
                                        warp_view->labels[m * params->source_width + n] = MAIN;
                                    if (!params->inverse_warp)
                                    {
                                        warp_view->texture[m * params->source_width * 3 + n * 3 + 0] = ref_view->texture[i * params->source_width * 3 + j * 3 + 0];
                                        warp_view->texture[m * params->source_width * 3 + n * 3 + 1] = ref_view->texture[i * params->source_width * 3 + j * 3 + 1];
                                        warp_view->texture[m * params->source_width * 3 + n * 3 + 2] = ref_view->texture[i * params->source_width * 3 + j * 3 + 2];
                                        warp_view->mconf[m * params->source_width + n] = 1.0;
                                    }
                                }
                            }
                        }
                    } // -- project 1 pixel to 9 pixels to avoid cracks
                }
                else
                {
                    warp_view->z_depth[yp * params->source_width + xp] = vir_depth;
                    if (params->use_label)
                        warp_view->labels[yp * params->source_width + xp] = MAIN;

                    if (!params->inverse_warp)
                    {
                        warp_view->texture[yp * params->source_width * 3 + xp * 3 + 0] = ref_view->texture[i * params->source_width * 3 + j * 3 + 0];
                        warp_view->texture[yp * params->source_width * 3 + xp * 3 + 1] = ref_view->texture[i * params->source_width * 3 + j * 3 + 1];
                        warp_view->texture[yp * params->source_width * 3 + xp * 3 + 2] = ref_view->texture[i * params->source_width * 3 + j * 3 + 2];
                        warp_view->mconf[yp * params->source_width + xp] = 1.0;
                    }

                }

            }

        }
    }

    return VSS_SUCCESS;
}




void generate_vir_view_pinhole(vss_param* params, vss_view* ref_view, vss_view* warp_view)
{
    cam_param* ref_cam = &(params->krt_rcam[ref_view->view_num]);
    cam_param* vir_cam = &(params->krt_vcam);
    int height = params->source_height;
    int width = params->source_width;

    /*Camera coordinate conversion*/
    /*From the reference camera to the virtual camera*/
    /* virtural to reference */
    float inv_vir_R[9];
    matrix_inverse_or_rotate_3x3(inv_vir_R, vir_cam->krt_R);

    float krt_R_vir_to_ref[9];
    matrix_mult_3x3_by_3x3(krt_R_vir_to_ref, ref_cam->krt_R, inv_vir_R);

    float krt_t_vir_sub_ref[3];
    vec3x1_sub(krt_t_vir_sub_ref, vir_cam->krt_WorldPosition, ref_cam->krt_WorldPosition);
    float krt_t_vir_to_ref[3];
    matrix_mult_3x3_by_3x1(krt_t_vir_to_ref, ref_cam->krt_R, krt_t_vir_sub_ref);


    float inv_krt_R[9];
    matrix_inverse_or_rotate_3x3(inv_krt_R, krt_R_vir_to_ref); // (AB)'=B'*A'

    float I[9] = { 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0 };//Mat::eye(3, 3, CV_32FC1); // -- identity matrix
    float Zero[3] = { 0.0,0.0,0.0 };

    int x, y;
    for (y = 0; y < height; y++)
        for (x = 0; x < width; x++)
        {

            float depth = warp_view->z_depth[y * width + x];

            if (depth <= 0.f)
            {
                continue;    // -- invalid range data    
            }

            float p_vir3D[3];
            Get3DPoint(vir_cam, x, y, depth, p_vir3D);

            float ref_p3D[3];

            euclidean_transform_point(ref_p3D, krt_R_vir_to_ref, p_vir3D, krt_t_vir_to_ref); //X*R + T; 

            float xpf, ypf;

            Get2DPoint(ref_cam, &xpf, &ypf, ref_p3D);

            if ((xpf < width) && (xpf >= 0.0f) || (ypf < params->source_height) && (ypf >= 0.0f))
            {
                int xp = MAX(0, MIN((int)(round(xpf + 0.5f)), 1 * width - 1));
                int yp = MAX(0, MIN((int)(round(ypf + 0.5f)), 1 * height - 1)); //ref pixel pos

                if (params->vir_interpolation_type)
                {
                    unsigned char  pix[3];

                    pix[0] = filtre_lanczos(ref_view->texture, ypf + 0.5f, xpf + 0.5f, width, height, width, 0);    //use subpixel interpolation
                    pix[1] = filtre_lanczos(ref_view->texture, ypf + 0.5f, xpf + 0.5f, width, height, width, 1);
                    pix[2] = filtre_lanczos(ref_view->texture, ypf + 0.5f, xpf + 0.5f, width, height, width, 2);

                    warp_view->texture[y * width * 3 + x * 3 + 0] = (unsigned char)(pix[0]);
                    warp_view->texture[y * width * 3 + x * 3 + 1] = (unsigned char)(pix[1]);
                    warp_view->texture[y * width * 3 + x * 3 + 2] = (unsigned char)(pix[2]);
                }
                else
                {
                    warp_view->texture[y * width * 3 + x * 3 + 0] = ref_view->texture[yp * width * 3 + xp * 3 + 0];
                    warp_view->texture[y * width * 3 + x * 3 + 1] = ref_view->texture[yp * width * 3 + xp * 3 + 1];
                    warp_view->texture[y * width * 3 + x * 3 + 2] = ref_view->texture[yp * width * 3 + xp * 3 + 2];
                }

                float ref_depth = ref_view->z_depth[yp  * width + xp];
                if (ref_depth <= 0.f)
                {
                    continue;
                }  // -- invalid holes

                float p_ref3D[3]; // 3D reference camera

                Get3DPoint(ref_cam, xp, yp, ref_depth, p_ref3D);

                //warp ref to vir
                // -- project back  
                /*  (Rc*Rv^-1)^-1 *(P - Rc(Tv-Tc))
                     = Rv*Rc^-1(P - Rc(Tv-Tc))
                     = Rv*Rc^-1P - Rv(Tv-Tc)
                */
                float pmt[] = { p_ref3D[0] - krt_t_vir_to_ref[0], p_ref3D[1] - krt_t_vir_to_ref[1], p_ref3D[2] - krt_t_vir_to_ref[2] };//P - Rc(Tv-Tc)
                float Vback[3]; // -- 3D points under vir camera's coordinate system
                matrix_mult_3x3_by_3x1(Vback, inv_krt_R, pmt);

                float bxpf, bypf;
                //warp ref to vir
                float vir_p3D[3]; // 3D reference camera
                euclidean_transform_point(vir_p3D, I, Vback, Zero); //X*R + T; 

                Get2DPoint(vir_cam, &bxpf, &bypf, vir_p3D);

                if ((bxpf < width) && (bxpf >= 0.0f) || (bypf < params->source_height) && (bypf >= 0.0f))
                {
                    float diffx = x - bxpf, diffy = y - bypf;
                    float dist = sqrtf(diffx * diffx + diffy * diffy);
                    warp_view->mconf[y * width + x] = expf(-dist / 5.f) * warp_view->pre_weight;
                }
            }


        }
}