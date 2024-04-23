/*****************************************************************************
 * Copyright (C) 2021-2051 anonymous
 *
 * Authors: anonymous <anonymous@anonymous>
 *****************************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include "vss.h"
#include "file.h"
#include "parser.h"


int main(int argc, char** argv)
{
    vss_param params;
    vss_view  views[MAX_SFM_CAMNUM];
    vss_view  warp_views[REF_CAM_N];
    vss_files files[MAX_SFM_CAMNUM];

    if (Configure(argc, argv, &params, files) < 0)  //get patameters from configure file
    {
        printf("configure failed\n");
        return -1;
    }

    vss_files* vir_file = &files[params.cam_num]; //The last file list is a virtual view file

    if (vss_files_open(files,&params)<0)
    {
        printf("vss open file failed!\n");
        return -1;
    }

    vss_view* vir_view = &views[params.cam_num]; //The last file list is a virtual view

    if (vss_open(views, warp_views, &params) < 0)
    {
        printf("vss open failed!\n");
        return -1;
    }

    int cam_ref[REF_CAM_N] = { 9, 11 };
    cam_ref[0] = params.left_view;
    cam_ref[1] = params.right_view;
    vss_read_files(files, views, &params, cam_ref);

    if (params.background_flag)
    {
        vss_read_bg_files(files, views, &params, cam_ref);
    }

    clock_t dibr_start, start;
    dibr_start = clock();
    

	for (int ref = 0; ref < REF_CAM_N; ref++)
	{
		start = clock();

        vss_view* warp_view = &warp_views[ref];

        vss_view* ref_view = &views[cam_ref[ref]];

		start = clock();

        vss_syn_view(&params, ref_view, warp_view);

		printf("warp time = %f\n", (double)(clock() - start) / CLOCKS_PER_SEC);
	}

    start = clock();
    vss_clear_vir_view(vir_view, &params);
    merge_warp_views(&params, warp_views, vir_view);
    printf("merge time = %f\n", (double)(clock() - start) / CLOCKS_PER_SEC);
    start = clock();
    hole_filling(&params, vir_view);
    printf("hole filling time = %f\n", (double)(clock() - start) / CLOCKS_PER_SEC);
    start = clock();
    filter_foreground(&params, vir_view);
    printf("filter_foreground time = %f\n", (double)(clock() - start) / CLOCKS_PER_SEC);

    printf("total time = %f\n", (double)(clock() - dibr_start) / CLOCKS_PER_SEC);

	fwrite(vir_view->depth, 1, params.source_width * params.source_height, vir_file->depth);
	fwrite(vir_view->texture, 1, params.source_width * params.source_height * 3, vir_file->texture);

    vss_close(views, warp_views, &params);
    vss_files_close(files, &params);

    return 0;
}

