/*
 *   ImgProcess.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <string.h>

#include "ImgProcess.h"

using namespace Robot;

void ImgProcess::YUVtoRGB(FrameBuffer *buf)
{
    unsigned char *yuyv, *rgb;
    int z = 0;

    yuyv = buf->m_YUVFrame->m_ImageData;
    rgb = buf->m_RGBFrame->m_ImageData;

    for(int height = 0; height < buf->m_YUVFrame->m_Height; height++)
    {
        for(int width = 0; width < buf->m_YUVFrame->m_Width; width++)
        {
            int r, g, b;
            int y, u, v;

            if(!z)
                y = yuyv[0] << 8;
            else
                y = yuyv[2] << 8;
            u = yuyv[1] - 128;
            v = yuyv[3] - 128;

            r = (y + (359 * v)) >> 8;
            g = (y - (88 * u) - (183 * v)) >> 8;
            b = (y + (454 * u)) >> 8;

            *(rgb++) = (r > 255) ? 255 : ((r < 0) ? 0 : r);
            *(rgb++) = (g > 255) ? 255 : ((g < 0) ? 0 : g);
            *(rgb++) = (b > 255) ? 255 : ((b < 0) ? 0 : b);

            if (z++)
            {
                z = 0;
                yuyv += 4;
            }
        }
    }
}

void ImgProcess::RGBtoHSV(FrameBuffer *buf)
{
    int ir, ig, ib, imin, imax;
    int th, ts, tv, diffvmin;

    for(int i = 0; i < buf->m_RGBFrame->m_Width*buf->m_RGBFrame->m_Height; i++)
    {
        ir = buf->m_RGBFrame->m_ImageData[3*i+0];
        ig = buf->m_RGBFrame->m_ImageData[3*i+1];
        ib = buf->m_RGBFrame->m_ImageData[3*i+2];

        if( ir > ig )
        {
            imax = ir;
            imin = ig;
        }
        else
        {
            imax = ig;
            imin = ir;
        }

        if( imax > ib ) {
            if( imin > ib ) imin = ib;
        } else imax = ib;

        tv = imax;
        diffvmin = imax - imin;

        if( (tv!=0) && (diffvmin!=0) )
        {
            ts = (255* diffvmin) / imax;
            if( tv == ir ) th = (ig-ib)*60/diffvmin;
            else if( tv == ig ) th = 120 + (ib-ir)*60/diffvmin;
            else th = 240 + (ir-ig)*60/diffvmin;
            if( th < 0 ) th += 360;
            th &= 0x0000FFFF;
        }
        else
        {
            tv = 0;
            ts = 0;
            th = 0xFFFF;
        }

        ts = ts * 100 / 255;
        tv = tv * 100 / 255;

        //buf->m_HSVFrame->m_ImageData[i]= (unsigned int)th | ((unsigned int)ts<<16) | ((unsigned int)tv<<24);
        buf->m_HSVFrame->m_ImageData[i*buf->m_HSVFrame->m_PixelSize+0] = (unsigned char)(th >> 8);
        buf->m_HSVFrame->m_ImageData[i*buf->m_HSVFrame->m_PixelSize+1] = (unsigned char)(th & 0xFF);
        buf->m_HSVFrame->m_ImageData[i*buf->m_HSVFrame->m_PixelSize+2] = (unsigned char)(ts & 0xFF);
        buf->m_HSVFrame->m_ImageData[i*buf->m_HSVFrame->m_PixelSize+3] = (unsigned char)(tv & 0xFF);
    }
}

void ImgProcess::Erosion(Image* img)
{
    int x, y;

    unsigned char* temp_img = new unsigned char[img->m_Width*img->m_Height];
    memset(temp_img, 0, img->m_Width*img->m_Height);

    for( y=1; y<(img->m_Height-1); y++ )
    {
        for( x=1; x<(img->m_Width-1); x++ )
        {
            temp_img[y*img->m_Width+x]= img->m_ImageData[(y-1)*img->m_Width+(x-1)]
                                      & img->m_ImageData[(y-1)*img->m_Width+(x  )]
                                      & img->m_ImageData[(y-1)*img->m_Width+(x+1)]
                                      & img->m_ImageData[(y  )*img->m_Width+(x-1)]
                                      & img->m_ImageData[(y  )*img->m_Width+(x  )]
                                      & img->m_ImageData[(y  )*img->m_Width+(x+1)]
                                      & img->m_ImageData[(y+1)*img->m_Width+(x-1)]
                                      & img->m_ImageData[(y+1)*img->m_Width+(x  )]
                                      & img->m_ImageData[(y+1)*img->m_Width+(x+1)];
        }
    }

    memcpy(img->m_ImageData, temp_img, img->m_Width*img->m_Height);

    delete[] temp_img;
}

void ImgProcess::Erosion(Image* src, Image* dest)
{
    int x, y;

    for( y=1; y<(src->m_Height-1); y++ )
    {
        for( x=1; x<(src->m_Width-1); x++ )
        {
            dest->m_ImageData[y*src->m_Width+x]= src->m_ImageData[(y-1)*src->m_Width+(x-1)]
                                               & src->m_ImageData[(y-1)*src->m_Width+(x  )]
                                               & src->m_ImageData[(y-1)*src->m_Width+(x+1)]
                                               & src->m_ImageData[(y  )*src->m_Width+(x-1)]
                                               & src->m_ImageData[(y  )*src->m_Width+(x  )]
                                               & src->m_ImageData[(y  )*src->m_Width+(x+1)]
                                               & src->m_ImageData[(y+1)*src->m_Width+(x-1)]
                                               & src->m_ImageData[(y+1)*src->m_Width+(x  )]
                                               & src->m_ImageData[(y+1)*src->m_Width+(x+1)];
        }
    }
}

void ImgProcess::Dilation(Image* img)
{
    int x, y;

    unsigned char* temp_img = new unsigned char[img->m_Width*img->m_Height];
    memset(temp_img, 0, img->m_Width*img->m_Height);

    for( y=1; y<(img->m_Height-1); y++ )
    {
        for( x=1; x<(img->m_Width-1); x++ )
        {
            temp_img[y*img->m_Width+x]= img->m_ImageData[(y-1)*img->m_Width+(x-1)]
                                      | img->m_ImageData[(y-1)*img->m_Width+(x  )]
                                      | img->m_ImageData[(y-1)*img->m_Width+(x+1)]
                                      | img->m_ImageData[(y  )*img->m_Width+(x-1)]
                                      | img->m_ImageData[(y  )*img->m_Width+(x  )]
                                      | img->m_ImageData[(y  )*img->m_Width+(x+1)]
                                      | img->m_ImageData[(y+1)*img->m_Width+(x-1)]
                                      | img->m_ImageData[(y+1)*img->m_Width+(x  )]
                                      | img->m_ImageData[(y+1)*img->m_Width+(x+1)];
        }
    }

    memcpy(img->m_ImageData, temp_img, img->m_Width*img->m_Height);

    delete[] temp_img;
}


/*TODO: not understandable... src, dest should have the same number of bytes / pixel?
what does this method? It does not appear to me that it is a dilation?*/
void ImgProcess::Dilation(Image* src, Image* dest)
{
    int x, y;

    for( y=1; y<(src->m_Height-1); y++ )
    {
        for( x=1; x<(src->m_Width-1); x++ )
        {
            dest->m_ImageData[y*src->m_Width+x]= src->m_ImageData[(y-1)*src->m_Width+(x-1)]
                                               | src->m_ImageData[(y-1)*src->m_Width+(x  )]
                                               | src->m_ImageData[(y-1)*src->m_Width+(x+1)]
                                               | src->m_ImageData[(y  )*src->m_Width+(x-1)]
                                               | src->m_ImageData[(y  )*src->m_Width+(x  )]
                                               | src->m_ImageData[(y  )*src->m_Width+(x+1)]
                                               | src->m_ImageData[(y+1)*src->m_Width+(x-1)]
                                               | src->m_ImageData[(y+1)*src->m_Width+(x  )]
                                               | src->m_ImageData[(y+1)*src->m_Width+(x+1)];

        }
    }
}

void ImgProcess::HFlipYUV(Image* img)
{
    int sizeline = img->m_Width * 2; /* 2 bytes per pixel*/
    unsigned char *pframe;
    pframe=img->m_ImageData;
    unsigned char line[sizeline-1];/*line buffer*/
    for (int h = 0; h < img->m_Height; h++)
    {   /*line iterator*/
        for(int w = sizeline-1; w > 0; w = w - 4)
        {   /* pixel iterator */
            line[w-1]=*pframe++;
            line[w-2]=*pframe++;
            line[w-3]=*pframe++;
            line[w]=*pframe++;
        }
        memcpy(img->m_ImageData+(h*sizeline), line, sizeline); /*copy reversed line to frame buffer*/
    }
}

void ImgProcess::VFlipYUV(Image* img)
{
    int sizeline = img->m_Width * 2; /* 2 bytes per pixel */
    unsigned char line1[sizeline-1];/*line1 buffer*/
    unsigned char line2[sizeline-1];/*line2 buffer*/
    for(int h = 0; h < img->m_Height/2; h++)
    {   /*line iterator*/
        memcpy(line1,img->m_ImageData+h*sizeline,sizeline);
        memcpy(line2,img->m_ImageData+(img->m_Height-1-h)*sizeline,sizeline);

        memcpy(img->m_ImageData+h*sizeline, line2, sizeline);
        memcpy(img->m_ImageData+(img->m_Height-1-h)*sizeline, line1, sizeline);
    }
}

// ***   WEBOTS PART  *** //

void ImgProcess::BGRAtoHSV(FrameBuffer *buf)
{
    int ir, ig, ib, imin, imax;
    int th, ts, tv, diffvmin;

    for(int i = 0; i < buf->m_BGRAFrame->m_Width*buf->m_BGRAFrame->m_Height; i++)
    {
        ib = buf->m_BGRAFrame->m_ImageData[4*i+0];
        ig = buf->m_BGRAFrame->m_ImageData[4*i+1];
        ir = buf->m_BGRAFrame->m_ImageData[4*i+2];

        if( ir > ig )
        {
            imax = ir;
            imin = ig;
        }
        else
        {
            imax = ig;
            imin = ir;
        }

        if( imax > ib ) {
            if( imin > ib ) imin = ib;
        } else imax = ib;

        tv = imax;
        diffvmin = imax - imin;

        if( (tv!=0) && (diffvmin!=0) )
        {
            ts = (255* diffvmin) / imax;
            if( tv == ir ) th = (ig-ib)*60/diffvmin;
            else if( tv == ig ) th = 120 + (ib-ir)*60/diffvmin;
            else th = 240 + (ir-ig)*60/diffvmin;
            if( th < 0 ) th += 360;
            th &= 0x0000FFFF;
        }
        else
        {
            tv = 0;
            ts = 0;
            th = 0xFFFF;
        }

        ts = ts * 100 / 255;
        tv = tv * 100 / 255;

        //buf->m_HSVFrame->m_ImageData[i]= (unsigned int)th | ((unsigned int)ts<<16) | ((unsigned int)tv<<24);
        buf->m_HSVFrame->m_ImageData[i*buf->m_HSVFrame->m_PixelSize+0] = (unsigned char)(th >> 8);
        buf->m_HSVFrame->m_ImageData[i*buf->m_HSVFrame->m_PixelSize+1] = (unsigned char)(th & 0xFF);
        buf->m_HSVFrame->m_ImageData[i*buf->m_HSVFrame->m_PixelSize+2] = (unsigned char)(ts & 0xFF);
        buf->m_HSVFrame->m_ImageData[i*buf->m_HSVFrame->m_PixelSize+3] = (unsigned char)(tv & 0xFF);
    }
}

