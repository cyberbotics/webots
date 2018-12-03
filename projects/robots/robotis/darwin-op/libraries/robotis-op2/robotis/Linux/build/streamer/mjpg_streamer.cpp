/*
 * mjpg_streamer.cpp
 *
 *  Created on: 2011. 1. 4.
 *      Author: zerom
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <arpa/inet.h>
#include <linux/videodev2.h>

#include "mjpg_streamer.h"
#include "jpeg_utils.h"

#define WWW_FOLDER          "./www/"

globals mjpg_streamer::global;

mjpg_streamer::mjpg_streamer(int width, int height)
{
    // TODO Auto-generated constructor stub
    input_yuv = new Image(width, height, Image::YUV_PIXEL_SIZE);
    input_rgb = new Image(width, height, Image::RGB_PIXEL_SIZE);
    global.buf = (unsigned char*)malloc(width*height*Image::YUV_PIXEL_SIZE);
    global.size = 0;

    if(pthread_mutex_init(&global.db, NULL) != 0)
        exit(EXIT_FAILURE);
    if(pthread_cond_init(&global.db_update, NULL) != 0)
        exit(EXIT_FAILURE);
    if(pthread_mutex_init(&controls_mutex, NULL) != 0)
        exit(EXIT_FAILURE);

    server.pglobal = &global;
    server.conf.port = htons(8080);
    server.conf.credentials = NULL;
    server.conf.www_folder = WWW_FOLDER;
    server.conf.nocommands = 0;

    pthread_create(&(server.threadID), NULL, server_thread, &(server));
    pthread_detach(server.threadID);
}

mjpg_streamer::mjpg_streamer(int width, int height, char* wwwdir)
{
    // TODO Auto-generated constructor stub
    input_yuv = new Image(width, height, Image::YUV_PIXEL_SIZE);
    input_rgb = new Image(width, height, Image::RGB_PIXEL_SIZE);
    global.buf = (unsigned char*)malloc(width*height*Image::YUV_PIXEL_SIZE);
    global.size = 0;

    if(pthread_mutex_init(&global.db, NULL) != 0)
        exit(EXIT_FAILURE);
    if(pthread_cond_init(&global.db_update, NULL) != 0)
        exit(EXIT_FAILURE);
    if(pthread_mutex_init(&controls_mutex, NULL) != 0)
        exit(EXIT_FAILURE);

    server.pglobal = &global;
    server.conf.port = htons(8080);
    server.conf.credentials = NULL;
    server.conf.www_folder = wwwdir;
    server.conf.nocommands = 0;

    pthread_create(&(server.threadID), NULL, server_thread, &(server));
    pthread_detach(server.threadID);
}

mjpg_streamer::~mjpg_streamer()
{
    // TODO Auto-generated destructor stub
}

int mjpg_streamer::input_cmd(in_cmd_type cmd, int value)
{
    int res = 0;

    fprintf(stderr, "input_cmd:%d, %d", cmd, value);

    pthread_mutex_lock(&controls_mutex);

    switch(cmd) {
    default:
        res = -1;
    }

    pthread_mutex_unlock(&controls_mutex);

    return res;
}

int mjpg_streamer::send_image(Image* img)
{
	if(httpd::ClientRequest == true)
	{
		if(img->m_PixelSize == Image::YUV_PIXEL_SIZE)
			memcpy(input_yuv->m_ImageData, img->m_ImageData, img->m_ImageSize);
		else if(img->m_PixelSize == Image::RGB_PIXEL_SIZE)
			memcpy(input_rgb->m_ImageData, img->m_ImageData, img->m_ImageSize);

		pthread_mutex_lock(&global.db);

		if(img->m_PixelSize == Image::YUV_PIXEL_SIZE)
			global.size = jpeg_utils::compress_yuyv_to_jpeg(input_yuv, global.buf, input_yuv->m_ImageSize, 80);
		else if(img->m_PixelSize == Image::RGB_PIXEL_SIZE)
			global.size = jpeg_utils::compress_rgb_to_jpeg(input_rgb, global.buf, input_rgb->m_ImageSize, 80);

		pthread_cond_broadcast(&global.db_update);
		pthread_mutex_unlock(&global.db);
		httpd::ClientRequest = false;
	}
	else
	{
        pthread_mutex_lock(&global.db);
        pthread_cond_broadcast(&global.db_update);
        pthread_mutex_unlock(&global.db);
	}

    return 0;
}

void* mjpg_streamer::server_thread(void* arg)
{
    httpd::server_thread(arg);
    return NULL;
}

