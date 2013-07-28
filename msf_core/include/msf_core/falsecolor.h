/*
 * Copyright (C) 2012-2013 Simon Lynen, ASL, ETH Zurich, Switzerland
 * You can contact the author at <slynen at ethz dot ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef FALSECOLOR_H_
#define FALSECOLOR_H_

struct color {
  unsigned char rgbBlue;
  unsigned char rgbGreen;
  unsigned char rgbRed;
  color() {
    rgbBlue = rgbGreen = rgbRed = 0;
  }
};

struct palette {
  enum palettetypes {
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

palette GetPalette(palette::palettetypes pal);

#endif  // FALSECOLOR_H_
