/*

Copyright (c) 2013, Simon Lynen, ASL, ETH Zurich, Switzerland
You can contact the author at <slynen at ethz dot ch>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
 * Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
False_color_palette3,
False_color_palette4
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

        case palette::False_color_palette4:
            /*
* False color palette #4. a little like matlab jet
*/
            for (i = 0; i < 256; i++)
            {
                r = (sin((i/255.0 * 360.0 + 240.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
                g = (sin((i/255.0 * 360.0 + 120.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
                b = (sin((i/255.0 * 360.0 + 0.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
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
