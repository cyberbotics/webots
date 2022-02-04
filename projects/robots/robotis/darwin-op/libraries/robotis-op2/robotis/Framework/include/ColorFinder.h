/*
 * ColorFinder.h
 * this class provides features to find a color in an image
 
 *  Created on: 2011. 1. 10.
 *      Author: zerom
 */

#ifndef COLORFINDER_H_
#define COLORFINDER_H_

#include <string>

#include "Point.h"
#include "Image.h"
#include "minIni.h"

#define COLOR_SECTION   "Find Color"
#define INVALID_VALUE   -1024.0

namespace Robot
{



    class ColorFinder
    {
    private:
        Point2D m_center_point;

        void Filtering(Image* img);

    public:
        int m_hue;             /* 0 ~ 360 */
        int m_hue_tolerance;   /* 0 ~ 180 */
        int m_min_saturation;  /* 0 ~ 100 */
        int m_max_saturation;  /* 0 ~ 100 */
        int m_min_value;       /* 0 ~ 100 */
        int m_max_value;       /* 0 ~ 100 */
        double m_min_percent;  /* 0.0 ~ 100.0 */
        double m_max_percent;  /* 0.0 ~ 100.0 */

        std::string color_section; /*TODO: ?*/

        Image*  m_result;

		
		
		/*TODO: not clear what it creates*/
        ColorFinder();

		/*
		Create an object to detect a color in an image
		The color is given according to the HSV model (hue, saturation and value)
		*/
        ColorFinder(int hue, int hue_tol, int min_sat, int min_val, double min_per, double max_per);
        ColorFinder(int hue, int hue_tol, int min_sat, int max_sat, int min_val, int max_val, double min_per, double max_per);

        virtual ~ColorFinder();

		
		
/*load from INI file*/		
		
        void LoadINISettings(minIni* ini);
        void LoadINISettings(minIni* ini, const std::string &section);
        void SaveINISettings(minIni* ini);
        void SaveINISettings(minIni* ini, const std::string &section);

        
		/*
		input: an image hsv_img
		output: the average point where the color is found, or (-1, -1) if the color is not found 
		effects: modify m_result via Filtering
		*/
		Point2D& GetPosition(Image* hsv_img);
    };
}

#endif /* COLORFINDER_H_ */
