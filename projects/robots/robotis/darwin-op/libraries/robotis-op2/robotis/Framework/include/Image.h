/*
 *   Image.h
 *   François : what is the difference between Image and FrameBuffer ?
 *   Author: ROBOTIS
 *
 */

#ifndef _IMAGE_H_
#define _IMAGE_H_


namespace Robot
{
	class Image
	{
	private:

	protected:

	public:
	    static const int YUV_PIXEL_SIZE = 4;
	    static const int RGB_PIXEL_SIZE = 3;
	    static const int HSV_PIXEL_SIZE = 4;
	    static const int BGRA_PIXEL_SIZE = 4;  // For Webots only

        unsigned char *m_ImageData; /* pointer to aligned image data */
        int m_Width;                /* image width in pixels */
        int m_Height;               /* image height in pixels */
        int m_PixelSize;            /* pixel size in bytes */
        int m_NumberOfPixels;       /* number of pixels */
        int m_WidthStep;            /* size of aligned image row in bytes */
        int m_ImageSize;            /* image data size in bytes (=image->m_Height*image->m_WidthStep) */

		/*create an image with width, height, pixelsize (number of bytes per pixel)
		TODO: what is the content of the image?*/
       	Image(int width, int height, int pixelsize);
		
		
		virtual ~Image();

		/*the affectation on images makes a copy*/
		Image& operator = (Image &img); 
	};

	
	
	
	/*TODO: what is the utility of FrameBuffer*/
	
	class FrameBuffer
	{
		private:

		protected:

		public:
	    		Image *m_YUVFrame;
	    		Image *m_RGBFrame;
	    		Image *m_HSVFrame;
	    		Image *m_BGRAFrame;  // for Webots only

	    		FrameBuffer(int width, int height);
	    		virtual ~FrameBuffer();
	};
}

#endif
