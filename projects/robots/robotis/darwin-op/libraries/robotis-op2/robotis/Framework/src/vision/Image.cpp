/*
 *   Image.cpp
 *
 *   Author: ROBOTIS
 *
 */

#include <stdlib.h>
#include <string.h>
#include "Image.h"

using namespace Robot;

Image::Image(int width, int height, int pixelsize) :
        m_Width(width),
        m_Height(height),
        m_PixelSize(pixelsize),
        m_NumberOfPixels(m_Width*m_Height),
        m_WidthStep(m_Width*m_PixelSize),
        m_ImageSize(m_Height*m_WidthStep)
{
	/*creates an array containing the data of the image*/
    m_ImageData = new unsigned char[m_ImageSize];
}

Image::~Image()
{
	delete[] m_ImageData;
    m_ImageData = 0;
}

Image& Image::operator = (Image &img)
{
	memcpy(m_ImageData, img.m_ImageData, m_ImageSize);
	return *this;
}


FrameBuffer::FrameBuffer(int width, int height)
{
    m_YUVFrame = new Image(width, height, Image::YUV_PIXEL_SIZE);
    m_RGBFrame = new Image(width, height, Image::RGB_PIXEL_SIZE);
    m_HSVFrame = new Image(width, height, Image::HSV_PIXEL_SIZE);
    m_BGRAFrame = new Image(width, height, Image::BGRA_PIXEL_SIZE); // for Webots only
}

FrameBuffer::~FrameBuffer()
{
    delete m_YUVFrame;
    delete m_RGBFrame;
    delete m_HSVFrame;
    delete m_BGRAFrame; // for Webots only
}
