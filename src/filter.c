/*****************************************************************************
 * Copyright (C) 2021-2051 anonymous
 *
 * Authors: anonymous <anonymous@anonymous>
 *****************************************************************************/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include "vss.h"
#include "filter.h"
#include "label.h"

#define LANCZOS_TAB_SIZE        3
#define LANCZOS_FAST_SCALE      100
#define LANCZOS_FAST_MAX_SIZE   4096
static double tbl_lanczos_coef[LANCZOS_FAST_MAX_SIZE * LANCZOS_FAST_SCALE];
#define lanczos_coef(x) tbl_lanczos_coef[(int)(fabs(x) * LANCZOS_FAST_SCALE + 0.5)]

#define MAX(a,b) (((a) > (b)) ? (a) : (b))
#define MIN(a,b) (((a) < (b)) ? (a) : (b))

// sinc function
static double sinc(double x)
{
    x *= CV_PI;
    if (x < 0.01 && x > -0.01) {
        double x2 = x * x;
        return 1.0f + x2 * (-1.0 / 6.0 + x2 / 120.0);
    }
    else {
        return sin(x) / x;
    }
}

void init_lanczos_filter()
{
    int i;
    for (i = 0; i < LANCZOS_FAST_MAX_SIZE*LANCZOS_FAST_SCALE; i++)
    {
        double x = (double)i / LANCZOS_FAST_SCALE;
        tbl_lanczos_coef[i] = sinc(x) * sinc(x / LANCZOS_TAB_SIZE);
    }
}

// if f < a, return a
// if f >= a, then
//      if f > z, return z
//      else, return f
static inline float clip(float f, float a, float z) {
    return (f < a) ? a : (f > z) ? z : f;
}

unsigned char filtre_lanczos(unsigned char * src, float j, float i, int input_width, int input_height, int i_src, int RGB_offset)
{
    double coef, sum = 0, res = 0;
    int m, n, idx_x, idx_y;
    float ret_val = 0;

    for (n = -LANCZOS_TAB_SIZE; n < LANCZOS_TAB_SIZE; n++)
    {
        for (m = -LANCZOS_TAB_SIZE; m < LANCZOS_TAB_SIZE; m++)
        {
            idx_x = (int)i + m + 1;
            idx_y = (int)j + n + 1;

            coef = lanczos_coef(i - idx_x) * lanczos_coef(j - idx_y);

            // when the neib. pixel is outside the boundary, using the boundary pixels
            idx_x = (idx_x < 0) ? 0 : idx_x;
            idx_y = (idx_y < 0) ? 0 : idx_y;
            idx_x = (idx_x >= input_width) ? (input_width - 1) : idx_x;
            idx_y = (idx_y >= input_height) ? (input_height - 1) : idx_y;

            //res += src[idx_x + idx_y * i_src] * coef;
            res += src[(idx_x + idx_y * i_src) * 3 + RGB_offset] * coef;
            sum += coef;
        }
    }

    if (sum != 0) {
        ret_val = (float)(res / sum + 0.5);
        ret_val = clip(ret_val, 0.0f, 255.0f);
    }

    return (unsigned char)ret_val;
}

unsigned char find_medium(unsigned char* depth_window, int N)
{
    int length = N - 1;
    for (int i = 0; i < length; i++)
    {
        //find MAX in each cycle
        for (int j = 0; j < length - i; j++)
        {
            if (depth_window[j] > depth_window[j + 1])
            {
                //interchange positions
                unsigned char temp = depth_window[j];
                depth_window[j] = depth_window[j + 1];
                depth_window[j + 1] = temp;
            }
        }
    }
    return depth_window[N / 2];

}

#define MAX_WINDOW_SIZE 441//(10+10+1)*(10+10+1)
void median_filter(unsigned char * src, unsigned char * buf, int width, int height, int kradius)
{
    unsigned char depth_window[MAX_WINDOW_SIZE*MAX_WINDOW_SIZE];
    int window_size = (kradius + kradius + 1)*(kradius + kradius + 1);
    for (int h = 0; h < height; h++)
        for (int w = 0; w < width; w++)
        {
            unsigned char medium_value = 0;
            for (int i = -kradius; i <= kradius; i++)
            {
                for (int j = -kradius; j <= kradius; j++)
                {
                    int imy = MAX(0, MIN(h + i, height - 1));  
                    int imx = MAX(0, MIN(w + j, width - 1));  //clip
                    depth_window[(i + kradius) * (2 * kradius + 1) + (j + kradius)] = src[imy * width + imx];
                }
            }
            medium_value = find_medium(depth_window, window_size);
            buf[h * width + w] = medium_value;
        }
    memcpy(src, buf, width*height * sizeof(unsigned char));
}


void median_filter_range_image(float* depth, float* out_depth, pixel_label* ilabels, pixel_label* olabels, int width, int height, int kradius, float fB, float MINdp, float dprange)
{
    int x, y;
    for (y = 0; y < height; y++)
        for (x = 0; x < width; x++)
        {
            if (ilabels[y * width + x] == HOLE)
            {
                olabels[y * width + x] = HOLE;
                out_depth[y * width + x] = 0.f;
                continue;
            }

            const int range_level = 512 + 1; // -- evenly divide the range into multiple bins, +1 for invalid holes
            //int hist[range_level];
            int hist[512 + 1];
            memset(hist, 0, range_level * sizeof(int)); // -- histogram
            float bin_len = dprange / (float)(range_level - 1);
            //int vnbr = 0;
            for (int i = -kradius; i <= kradius; i++)
            {
                for (int j = -kradius; j <= kradius; j++)
                {
                    int imy = MAX(0, MIN(y + i, height - 1));  //make sure the index (x+j, y+i) is located in the image.
                    int imx = MAX(0, MIN(x + j, width - 1)); // -- clamp
                    if (ilabels[imy * width + imx] == HOLE)
                    {
                        hist[0] ++;
                        continue;
                    } // -- invalid holes

                    float ival = fB / depth[imy * width + imx]; // -- input range value
                    int binID = MAX(0, MIN(range_level - 2, (int)floor((ival - MINdp) / bin_len)));
                    hist[binID + 1] ++;
                    //vnbr++;
                }
            }
            const int tNbr = (2 * kradius + 1) * (2 * kradius + 1);
            int mID = 0; // -- median bin ID
            int aval = 0; // -- accumulated value
            for (int id = 0; id < range_level + 1; id++)
            {
                aval += hist[id];
                if (aval > tNbr / 2)
                {
                    mID = id;
                    break;
                }
            } // -- select the median value by accumulating the histogram
            if (mID == 0)
            {
                out_depth[y * width + x] = 0.f;
                olabels[y * width + x] = HOLE;
            }
            else
            {
                float oval = fB / ((mID - 1.f + 0.5f) * bin_len + MINdp);
                out_depth[y * width + x] = oval;
                olabels[y * width + x] = MAIN;
            }
        }
}

int median_vir_depth(vss_param* params, vss_view* warp_view)
{
    float max_disp = params->fB / params->min_depth, min_disp = params->fB / params->max_depth;
    float drange = max_disp - min_disp + 1;

    memset(warp_view->filter_buf, 0, params->source_width * params->source_height * sizeof(float));
    memset(warp_view->filter_label_buf, HOLE, params->source_width * params->source_height * sizeof(pixel_label));

    median_filter_range_image(warp_view->z_depth, warp_view->filter_buf, warp_view->labels, warp_view->filter_label_buf, params->source_width, params->source_height, params->vir_depth_filter_radius, params->fB, min_disp, drange);
    memcpy(warp_view->z_depth, warp_view->filter_buf, params->source_width * params->source_height * sizeof(float));
    memcpy(warp_view->labels, warp_view->filter_label_buf, params->source_width * params->source_height * sizeof(pixel_label));

    return VSS_SUCCESS;
}

// add bilateral filter for range image. Modified in Aug, 2020
void bilateral_filter_vir_depth_image(float* depth, float* out_depth, pixel_label* ilabels, pixel_label* olabels, int width, int height, int kradius, float fB, float* ws_arr, float* wd_arr, int wd_arr_len, float drange)
{
    int x, y;

    for (y = 0; y < height; y++)
        for (x = 0; x < width; x++)
        {
            if (ilabels[y * width + x] == HOLE)
            {
                olabels[y * width + x] = HOLE;
                out_depth[y * width + x] = 0.f;
                continue;
            }

            float range = fB / depth[y * width + x];
            float sumC = 0.f;
            float sumW = 0.f;
            const float sigs = (float)(kradius * 1.2f);
            const float sigd = (float)(kradius * 3.6f);
            for (int i = -kradius; i <= kradius; i++) {
                for (int j = -kradius; j <= kradius; j++)
                {

                    int imy = MAX(0, MIN(y + i, height - 1));
                    int imx = MAX(0, MIN(x + j, width - 1));

                    if (ilabels[imy * width + imx] == HOLE)
                    {
                        continue;
                    }
                    int dist = (i * i + j * j);
                    float ws = ws_arr[dist];

                    float disp = fB / depth[imy * width + imx];
                    int indx = (int)(fabs(disp - range) / drange * wd_arr_len);
                    indx = indx < wd_arr_len ? indx : wd_arr_len - 1;
                    float wd = wd_arr[indx];
                    float d = depth[imy * width + imx];

                    sumC += (ws * wd * d);
                    sumW += (ws * wd);

                }
            }


            if (sumW < 1e-5f) { continue; }

            out_depth[y * width + x] = sumC / sumW;
            olabels[y * width + x] = MAIN;
        }
}

// bilteral filter added, modified in Aug, 2020
int set_ws_array(float* arr, float drange, int radius)
{
    const float sigs = (float)(radius * 1.2f);

    for (int i = 0; i <= radius * radius * 2; i++)
    {
        float dist = sqrtf((float)(i));
        float ws = expf(-dist / sigs);
        arr[i] = ws;
    }
    return VSS_SUCCESS;
}
// bilteral filter added, modified in Aug, 2020
int set_wd_array(float* arr, int wd_array_len, const int radius, float drange)
{
    const float sigd = (float)(radius * 3.6f);
    for (int i = 0; i < wd_array_len; i++)
    {
        float r = (float)(i) / (float)(wd_array_len)* drange;
        float wd = expf(-r / sigd);
        arr[i] = wd;
    }
    return VSS_SUCCESS;
}

int bilateral_vir_depth(vss_param* params, vss_view* warp_view)
{
    float max_disp = params->fB / params->min_depth, min_disp = params->fB / params->max_depth;
    float drange = max_disp - min_disp + 1;

    memset(warp_view->ws_array_buf, 0, (params->vir_depth_filter_radius * params->vir_depth_filter_radius * 2 + 1) * sizeof(float));
    set_ws_array(warp_view->ws_array_buf, drange, params->vir_depth_filter_radius);

    int wd_array_len = (int)(drange * 2.f);
    memset(warp_view->wd_array_buf, 0, wd_array_len * sizeof(float));
    set_wd_array(warp_view->wd_array_buf, wd_array_len, params->vir_depth_filter_radius, drange);


    memset(warp_view->filter_buf, 0, params->source_width * params->source_height * sizeof(float));
    memset(warp_view->filter_label_buf, HOLE, params->source_width * params->source_height * sizeof(pixel_label));

    bilateral_filter_vir_depth_image(warp_view->z_depth, warp_view->filter_buf, warp_view->labels, warp_view->filter_label_buf, params->source_width, params->source_height, params->vir_depth_filter_radius, params->fB, warp_view->ws_array_buf, warp_view->wd_array_buf, wd_array_len, drange);

    memcpy(warp_view->z_depth, warp_view->filter_buf, params->source_width * params->source_height * sizeof(float));
    memcpy(warp_view->labels, warp_view->filter_label_buf, params->source_width * params->source_height * sizeof(pixel_label));

    return VSS_SUCCESS;
}

void smooth_foreground(vss_param* params, vss_view* vir_view)
{
    int x, y;
    for (y = 0; y < params->source_height; y++)
        for (x = 0; x < params->source_width; x++)
        {
            if (vir_view->labels[y * params->source_width + x] != F_BOUNDARY)
            {
                for (int c = 0; c < 3; c++)
                {
                    vir_view->texture_fore_smooth[y * params->source_width * 3 + x * 3 + c] = vir_view->texture[y * params->source_width * 3 + x * 3 + c];
                }

                continue;
            }

            float sumC[] = { 0.f, 0.f, 0.f };
            float sumW = 0.f;
            for (int i = -params->smooth_foreground_radius; i <= params->smooth_foreground_radius; i++)
            {
                for (int j = -params->smooth_foreground_radius; j <= params->smooth_foreground_radius; j++)
                {
                    int imy = MAX(0, MIN(y + i, params->source_height - 1));
                    int imx = MAX(0, MIN(x + j, params->source_width - 1));
                    float dist = sqrtf((float)(i * i + j * j));
                    float w = expf(-dist / params->smooth_foreground_sig);
                    for (int c = 0; c < 3; c++)
                    {
                        float d = vir_view->texture[imy * params->source_width * 3 + imx * 3 + c];
                        sumC[c] += (w * d);
                    }
                    sumW += w;
                }
            }
            sumW += 1e-5f; // -- avoid /0 error
            for (int c = 0; c < 3; c++)
            {
                vir_view->texture_fore_smooth[y * params->source_width * 3 + x * 3 + c] = (unsigned char)(sumC[c] / sumW);
            }
        }

    memcpy(vir_view->texture, vir_view->texture_fore_smooth, params->source_width * params->source_height * 3 * sizeof(unsigned char));  //copy final result after smooth
}


int filter_foreground(vss_param* params, vss_view* vir_view)
{
    if (!params->foreground_edge_radius)
        return 0;

    label_foreground_boudary_pixels(params, vir_view);
    smooth_foreground(params, vir_view); // -- smooth the foreground edges

    return VSS_SUCCESS;
}
