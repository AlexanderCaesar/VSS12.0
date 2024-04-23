/*****************************************************************************
 * Copyright (C) 2021-2051 anonymous
 *
 * Authors: anonymous <anonymous@anonymous>
 *****************************************************************************/

#include <string.h>
#include <math.h>
#include "fisheye.h"
#include "filter.h"
#include "matrix.h"

#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define MIN(a,b) (((a) < (b)) ? (a) : (b))


 //Brute-Force Search the root for five-degree polynomial equation
static float  roots_one_real(float* coeff)
{
    int i;

    float root = 0;
    float min_diff = 100.0;
    for (i = 0; i < 100000; i = i + 20)
    {
        float theta = ((float)i) * 0.00001f; //
        float theta3 = theta * theta * theta;
        float theta5 = theta * theta * theta3;
        float thetaD = coeff[5] * theta5 + coeff[3] * theta3 + coeff[1] * theta + coeff[0];
        if (fabs(thetaD) < min_diff)
        {
            min_diff = (float)fabs(thetaD);
            root = theta;
        }
    }
    return root;
}


//Polynomial calculation theta table
static int set_r2t_array_by_kc(float* arr, float* kc, int rtheta_length, float fisheye_radius)
{
    float coeff[6] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };//-radius+k_1 r+k_2 r^3+k_3 r^5

    coeff[1] = kc[0];
    coeff[3] = kc[1];
    coeff[5] = kc[2];

    for (int i = 0; i < rtheta_length; i++)    // rtheta_length = r2t_len = 2*fisheye_radius 
    {
        float r = (float)(i) / (float)(rtheta_length)* fisheye_radius;
        coeff[0] = -r;

        arr[i] = roots_one_real(coeff); // MIN_theta_val;                
    }
    return VSS_SUCCESS;
}

void set_theta_table(vss_param* params, vss_view* ref_view, vss_view* warp_view)
{
    cam_param* ref_cam = &(params->krt_rcam[ref_view->view_num]);
    int ref_len = (int)(ref_cam->fisheye_radius * 2.f);

    memset(ref_view->theta_table, 0, ref_len * sizeof(float));

    set_r2t_array_by_kc(ref_view->theta_table, ref_cam->krt_kc, ref_len, ref_cam->fisheye_radius);

    int vir_len = (int)(params->krt_vcam.fisheye_radius * 2.f);
    memset(warp_view->theta_table, 0, vir_len * sizeof(float));

    set_r2t_array_by_kc(warp_view->theta_table, params->krt_vcam.krt_kc, vir_len, params->krt_vcam.fisheye_radius);
}

float get_theta(float* arr_rtheta, int length_rtheta, float fisheye_radius, float r)
{
    int indx = (int)(r / fisheye_radius * length_rtheta); // length_rtheta = 2*fisheye_radius
    indx = indx < length_rtheta ? indx : length_rtheta - 1;//clip
    return arr_rtheta[indx];
}



int warp_ref_depth_fisheye(vss_param* params, vss_view* ref_view, vss_view* warp_view)
{

    cam_param* ref_cam = &(params->krt_rcam[ref_view->view_num]);
    int ref_len = (int)(ref_cam->fisheye_radius * 2.f);

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
    matrix_mult_3x3_by_3x1(krt_t, params->krt_vcam.krt_R, krt_t);                   //  T = Rv*(Tref - Tv)

    for (int i = 0; i < params->source_height; i++)
    {
        for (int j = 0; j < params->source_width; j++)
        {
            if (params->use_label && (ref_view->labels[i * params->source_width + j] != MAIN))
                continue; // project novel_pixel_main

            float xd_norm = (j - ref_cam->krt_cc[0]);
            float yd_norm = (i - ref_cam->krt_cc[1]);
            float radius = sqrtf(xd_norm * xd_norm + yd_norm * yd_norm);
            float theta = get_theta(ref_view->theta_table, ref_len, ref_cam->fisheye_radius, radius);     //get theta by look-up table

            float phi = (float)(atan2(yd_norm, xd_norm));
            float z_depth = ref_view->z_depth[i * params->source_width + j];

            if (z_depth <= 0.f) { continue; }

            float vp3D[3];
            float r = 0.0f;
            int xp = 0;
            int yp = 0;
            p3D[2] = (float)(z_depth * cos(theta));
            float r_undistort = sqrtf(z_depth * z_depth - p3D[2] * p3D[2]);
            p3D[0] = (float)(r_undistort * cos(phi));
            p3D[1] = (float)(r_undistort * sin(phi));
            
            euclidean_transform_point(vp3D, krt_R, p3D, krt_t); //X*R + T;     

            float x_norm = vp3D[0] / vp3D[2];   //step 3:  check: FOV, image res,fisheye_radius   
            float y_norm = vp3D[1] / vp3D[2];   //map x,y,z on the plane z = 1
            phi = (float)(atan2(y_norm, x_norm));
            r_undistort = sqrtf(x_norm * x_norm + y_norm * y_norm); //(phi, theta) under virtual camera
            theta = (float)(atan(r_undistort));

            if (vp3D[2] < 0.f) { theta = (float)(CV_PI - theta); } // -- beyond 180 degree
            if (theta > params->krt_vcam.lens_fov)
            {
                continue;
            } // -- out of the lens fov
            float tp2 = theta * theta;
            float tp3 = theta * tp2;
            float tp5 = tp3 * tp2;
            r = params->krt_vcam.krt_kc[0] * theta + params->krt_vcam.krt_kc[1] * tp3 + params->krt_vcam.krt_kc[2] * tp5;
            xp = (int)roundf((float)(r * cos(phi) + params->krt_vcam.krt_cc[0])); // virtual view pox x
            yp = (int)roundf((float)(r * sin(phi) + params->krt_vcam.krt_cc[1])); // virtual view pox y

            if (r < params->krt_vcam.fisheye_radius && xp >= 1 && xp <= (params->source_width - 1) - 1 && yp >= 1 && yp <= (params->source_height - 1) - 1)   //because 3x3 
            {
                float vir_depth;

                vir_depth = sqrtf(vp3D[0] * vp3D[0] + vp3D[1] * vp3D[1] + vp3D[2] * vp3D[2]); //step 4:					

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
        } // -- within valid image plane
    }

    return VSS_SUCCESS;
}


//Camera coordinate system depth = 1
void get_camera_coordinate(vss_param* params, cam_param* cam, float* rtheta_arr, float x, float y, float *p3D_norm)
{
    float xd_norm = x - cam->krt_cc[0];
    float yd_norm = y - cam->krt_cc[1];
    float radius = sqrtf(xd_norm * xd_norm + yd_norm * yd_norm);

    int rtheta_len = (int)(cam->fisheye_radius * 2.f);

    float theta = get_theta(rtheta_arr, rtheta_len, cam->fisheye_radius, radius);  //resort to  look-up table
    float phi = (float)(atan2(yd_norm, xd_norm));

    float sphrad = 1.f; // -- unit vector

    p3D_norm[2] = (float)(sphrad * cos(theta));
    float r_undistort = sqrtf(sphrad - p3D_norm[2] * p3D_norm[2]);
    p3D_norm[0] = (float)(r_undistort * cos(phi));
    p3D_norm[1] = (float)(r_undistort * sin(phi));

}

int project_3D_to_2D_fisheye(vss_param* params, cam_param* cam, float* p3D, float* R, float *t, float *xp, float *yp)
{

    float warp_p3D[3];
    euclidean_transform_point(warp_p3D, R, p3D, t);

    float r = 0.0f;

    float x_norm = warp_p3D[0] / warp_p3D[2];
    float y_norm = warp_p3D[1] / warp_p3D[2];

    float ryx = (float)(fabs(y_norm / (x_norm + 1e-10f))); // -- avoid /0 error
    float r_undistort = sqrtf(x_norm * x_norm + y_norm * y_norm);
    float theta = (float)(atan(r_undistort));

    if (warp_p3D[2] < 0.f) { theta = (float)(CV_PI - theta); } // -- beyond 180 degree
    if (theta > cam->lens_fov) { return VSS_ERR; } // -- out of the lens fov

    float tp2 = theta * theta;
    float tp3 = theta * tp2;
    float tp5 = tp3 * tp2;
    r = cam->krt_kc[0] * theta + cam->krt_kc[1] * tp3 + cam->krt_kc[2] * tp5;
    float fabsx = sqrtf(r * r / (1.f + ryx * ryx));
    float fabsy = ryx * fabsx;
    *xp = (x_norm > 0.f ? fabsx : -fabsx) + cam->krt_cc[0];
    *yp = (y_norm > 0.f ? fabsy : -fabsy) + cam->krt_cc[1];

    if ((r > cam->fisheye_radius) || (*xp > params->source_width) || (*xp < 0) || (*yp > params->source_height) || (*yp < 0))
    {
        return VSS_ERR;// VSS_ERR;
    } // -- out of the image plane
    else
    {
        return VSS_SUCCESS;
    }
}

void generate_vir_view_fisheye(vss_param* params, vss_view* ref_view, vss_view* warp_view)
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
            //vitrual view(x,y) -> (X/Z,Y/Z,1)
            get_camera_coordinate(params, vir_cam, warp_view->theta_table, (float)x, (float)y, p_vir3D);
            p_vir3D[0] *= depth; p_vir3D[1] *= depth; p_vir3D[2] *= depth;

            float xpf, ypf;
            if (project_3D_to_2D_fisheye(params, ref_cam, p_vir3D, krt_R_vir_to_ref, krt_t_vir_to_ref, &xpf, &ypf)) //virtura(X,Y,Z) ---> ref(xpf,ypf)
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
                //ref view(xpf,ypf) -> (X/Z,Y/Z,1)
                get_camera_coordinate(params, ref_cam, ref_view->theta_table, xpf, ypf, p_ref3D);
                p_ref3D[0] *= ref_depth; p_ref3D[1] *= ref_depth; p_ref3D[2] *= ref_depth;


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
                if (project_3D_to_2D_fisheye(params, vir_cam, Vback, I, Zero, &bxpf, &bypf))
                {
                    float diffx = x - bxpf, diffy = y - bypf;
                    float dist = sqrtf(diffx * diffx + diffy * diffy);
                    warp_view->mconf[y * width + x] = expf(-dist / 5.f) * warp_view->pre_weight;
                }
            }

        }
}