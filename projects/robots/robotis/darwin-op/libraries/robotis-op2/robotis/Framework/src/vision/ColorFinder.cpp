/*
 * ColorFinder.cpp
 *
 *  Created on: 2011. 1. 10.
 *      Author: zerom
 */

#include <stdlib.h>

#include "ColorFinder.h"
#include "ImgProcess.h"

using namespace Robot;

ColorFinder::ColorFinder() :
        m_center_point(Point2D()),
        m_hue(356),
        m_hue_tolerance(15),
        m_min_saturation(50),
        m_max_saturation(100),
        m_min_value(10),
        m_max_value(100),
        m_min_percent(0.07),
        m_max_percent(30.0),
        color_section(""),
        m_result(0)
{ }

ColorFinder::ColorFinder(int hue, int hue_tol, int min_sat, int min_val, double min_per, double max_per) :
        m_hue(hue),
        m_hue_tolerance(hue_tol),
        m_min_saturation(min_sat),
        m_max_saturation(100),
        m_min_value(min_val),
        m_max_value(100),
        m_min_percent(min_per),
        m_max_percent(max_per),
        color_section(""),
        m_result(0)
{ }

ColorFinder::ColorFinder(int hue, int hue_tol, int min_sat, int max_sat, int min_val, int max_val, double min_per, double max_per) :
        m_hue(hue),
        m_hue_tolerance(hue_tol),
        m_min_saturation(min_sat),
        m_max_saturation(max_sat),
        m_min_value(min_val),
        m_max_value(max_val),
        m_min_percent(min_per),
        m_max_percent(max_per),
        color_section(""),
        m_result(0)
{ }

ColorFinder::~ColorFinder()
{
    // TODO Auto-generated destructor stub
}


/*
input: an image img
output: no output
effects: modify m_result. m_result is an image of the same size that img. _result->m_ImageData[i] is then 1 if the pixel n. i is correct and  else.
*/
void ColorFinder::Filtering(Image *img)
{
    unsigned int h, s, v;
    int h_max, h_min;

    if(m_result == NULL)
        m_result = new Image(img->m_Width, img->m_Height, 1);

    h_max = m_hue + m_hue_tolerance;
    h_min = m_hue - m_hue_tolerance;
    if(h_max > 360)
        h_max -= 360;
    if(h_min < 0)
        h_min += 360;

    for(int i = 0; i < img->m_NumberOfPixels; i++)
    {
        h = (img->m_ImageData[i*img->m_PixelSize + 0] << 8) | img->m_ImageData[i*img->m_PixelSize + 1];
        s =  img->m_ImageData[i*img->m_PixelSize + 2];
        v =  img->m_ImageData[i*img->m_PixelSize + 3];

        if( h > 360 )
            h = h % 360;

        if( ((int)s >= m_min_saturation) && ((int)s <= m_max_saturation) &&
            ((int)v >= m_min_value) && ((int)v <= m_max_value) )
        {
            if(h_min <= h_max)
            {
                if((h_min < (int)h) && ((int)h < h_max))
                    m_result->m_ImageData[i]= 1;
                else
                    m_result->m_ImageData[i]= 0;
            }
            else
            {
                if((h_min < (int)h) || ((int)h < h_max))
                    m_result->m_ImageData[i]= 1;
                else
                    m_result->m_ImageData[i]= 0;
            }
        }
        else
        {
            m_result->m_ImageData[i]= 0;
        }
    }
}

void ColorFinder::LoadINISettings(minIni* ini)
{
    LoadINISettings(ini, COLOR_SECTION);
}

void ColorFinder::LoadINISettings(minIni* ini, const std::string &section)
{
    int value = -2;
    if((value = ini->geti(section, "hue", INVALID_VALUE)) != INVALID_VALUE)             m_hue = value;
    if((value = ini->geti(section, "hue_tolerance", INVALID_VALUE)) != INVALID_VALUE)   m_hue_tolerance = value;
    if((value = ini->geti(section, "min_saturation", INVALID_VALUE)) != INVALID_VALUE)  m_min_saturation = value;
    if((value = ini->geti(section, "max_saturation", INVALID_VALUE)) != INVALID_VALUE)  m_max_saturation = value;
    if((value = ini->geti(section, "min_value", INVALID_VALUE)) != INVALID_VALUE)       m_min_value = value;
    if((value = ini->geti(section, "max_value", INVALID_VALUE)) != INVALID_VALUE)       m_max_value = value;

    double dvalue = -2.0;
    if((dvalue = ini->getd(section, "min_percent", INVALID_VALUE)) != INVALID_VALUE)    m_min_percent = dvalue;
    if((dvalue = ini->getd(section, "max_percent", INVALID_VALUE)) != INVALID_VALUE)    m_max_percent = dvalue;

    color_section = section;
}

void ColorFinder::SaveINISettings(minIni* ini)
{
    SaveINISettings(ini, COLOR_SECTION);
}

void ColorFinder::SaveINISettings(minIni* ini, const std::string &section)
{
    ini->put(section,   "hue",              m_hue);
    ini->put(section,   "hue_tolerance",    m_hue_tolerance);
    ini->put(section,   "min_saturation",   m_min_saturation);
    ini->put(section,   "max_saturation",   m_max_saturation);
    ini->put(section,   "min_value",        m_min_value);
    ini->put(section,   "max_value",        m_max_value);
    ini->put(section,   "min_percent",      m_min_percent);
    ini->put(section,   "max_percent",      m_max_percent);

    color_section = section;
}

/*
input: an image hsv_img
output: the average point where the color is found, or (-1, -1) if the color is not found 
effects: modify m_result via Filtering
*/
Point2D& ColorFinder::GetPosition(Image* hsv_img)
{
    int sum_x = 0, sum_y = 0, count = 0;

    Filtering(hsv_img);

    ImgProcess::Erosion(m_result);
    ImgProcess::Dilation(m_result);

    for(int y = 0; y < m_result->m_Height; y++)
    {
        for(int x = 0; x < m_result->m_Width; x++)
        {
            if(m_result->m_ImageData[m_result->m_Width * y + x] > 0)
            {
                sum_x += x;
                sum_y += y;
                count++;
            }
        }
    }

    if(count <= (hsv_img->m_NumberOfPixels * m_min_percent / 100) || count > (hsv_img->m_NumberOfPixels * m_max_percent / 100))
    {
        m_center_point.X = -1.0;
        m_center_point.Y = -1.0;
    }
    else
    {
        m_center_point.X = (int)((double)sum_x / (double)count);
        m_center_point.Y = (int)((double)sum_y / (double)count);
    }

    return m_center_point;
}
