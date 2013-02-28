/*
 * falsecolor.h
 *
 *  Created on: Feb 28, 2013
 *      Author: slynen
 */

#ifndef FALSECOLOR_H_
#define FALSECOLOR_H_

#include <opencv2/opencv.hpp>
#include <assert.h>

struct color{
unsigned char rgbBlue;
unsigned char rgbGreen;
unsigned char rgbRed;
color(){
rgbBlue = rgbGreen = rgbRed = 0;
}
};

struct palette{
enum palettetypes{
Linear_red_palettes,
GammaLog_red_palettes,
Inversion_red_palette,
Linear_palettes,
GammaLog_palettes,
Inversion_palette,
False_color_palette1,
False_color_palette2,
False_color_palette3
};
color colors[256];
};

#define DEG2RAD 0.01745329
palette GetPalette(palette::palettetypes pal)
{
palette ret;

    int i, r, g, b;
    float f;

    switch (pal)
    {
     case palette::Linear_red_palettes:
            /*
* Linear red palettes.
*/
            for (i = 0; i < 256; i++)
            {
                ret.colors[i].rgbBlue = 0;
                ret.colors[i].rgbGreen = 0;
                ret.colors[i].rgbRed = i;
            }
            break;
        case palette::GammaLog_red_palettes:
            /*
* GammaLog red palettes.
*/
            for (i = 0; i < 256; i++)
            {
                f = log10(pow((i/255.0), 1.0)*9.0 + 1.0) * 255.0;
                ret.colors[i].rgbBlue = 0;
                ret.colors[i].rgbGreen = 0;
                ret.colors[i].rgbRed = f;
            }
            break;
        case palette::Inversion_red_palette:
            /*
* Inversion red palette.
*/
            for (i = 0; i < 256; i++)
            {
                ret.colors[i].rgbBlue = 0;
                ret.colors[i].rgbGreen = 0;
                ret.colors[i].rgbRed = 255 - i;
            }
            break;
     case palette::Linear_palettes:
            /*
* Linear palettes.
*/
            for (i = 0; i < 256; i++)
            {
                ret.colors[i].rgbBlue =
                ret.colors[i].rgbGreen =
                ret.colors[i].rgbRed = i;
            }
            break;
        case palette::GammaLog_palettes:
            /*
* GammaLog palettes.
*/
            for (i = 0; i < 256; i++)
            {
                f = log10(pow((i/255.0), 1.0)*9.0 + 1.0) * 255.0;
                ret.colors[i].rgbBlue =
                ret.colors[i].rgbGreen =
                ret.colors[i].rgbRed = f;
            }
            break;
        case palette::Inversion_palette:
            /*
* Inversion palette.
*/
            for (i = 0; i < 256; i++)
            {
                ret.colors[i].rgbBlue =
                ret.colors[i].rgbGreen =
                ret.colors[i].rgbRed = 255 - i;
            }
            break;
        case palette::False_color_palette1:
            /*
* False color palette #1.
*/
            for (i = 0; i < 256; i++)
            {
                r = (sin((i/255.0 * 360.0 + 0.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
                g = (sin((i/255.0 * 360.0 + 120.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
                b = (sin((i/255.0 * 360.0 + 240.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
                ret.colors[i].rgbBlue = b;
                ret.colors[i].rgbGreen = g;
                ret.colors[i].rgbRed = r;
            }
            break;
        case palette::False_color_palette2:
            /*
* False color palette #2.
*/
            for (i = 0; i < 256; i++)
            {
                r = (sin((i/255.0 * 360.0 + 120.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
                g = (sin((i/255.0 * 360.0 + 240.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
                b = (sin((i/255.0 * 360.0 + 0.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
                ret.colors[i].rgbBlue = b;
                ret.colors[i].rgbGreen = g;
                ret.colors[i].rgbRed = r;
            }
            break;
        case palette::False_color_palette3:
            /*
* False color palette #3.
*/
            for (i = 0; i < 256; i++)
            {
                r = (sin((i/255.0 * 360.0 + 240.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
                g = (sin((i/255.0 * 360.0 + 0.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
                b = (sin((i/255.0 * 360.0 + 120.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
                ret.colors[i].rgbBlue = b;
                ret.colors[i].rgbGreen = g;
                ret.colors[i].rgbRed = r;
            }
            break;
    }
    return ret;
}
#undef DEG2RAD

#endif /* FALSECOLOR_H_ */
