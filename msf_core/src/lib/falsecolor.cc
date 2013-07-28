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
#include <msf_core/falsecolor.h>
#include <cmath>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

#define DEG2RAD 0.01745329
palette GetPalette(palette::palettetypes pal) {
  palette ret;
  int i, r, g, b;
  float f;
  switch (pal) {
    case palette::Linear_red_palettes:
      /*
       * Linear red palettes.
       */
      for (i = 0; i < 256; i++) {
        ret.colors[i].rgbBlue = 0;
        ret.colors[i].rgbGreen = 0;
        ret.colors[i].rgbRed = i;
      }
      break;
    case palette::GammaLog_red_palettes:
      /*
       * GammaLog red palettes.
       */
      for (i = 0; i < 256; i++) {
        f = log10(pow((i / 255.0), 1.0) * 9.0 + 1.0) * 255.0;
        ret.colors[i].rgbBlue = 0;
        ret.colors[i].rgbGreen = 0;
        ret.colors[i].rgbRed = f;
      }
      break;
    case palette::Inversion_red_palette:
      /*
       * Inversion red palette.
       */
      for (i = 0; i < 256; i++) {
        ret.colors[i].rgbBlue = 0;
        ret.colors[i].rgbGreen = 0;
        ret.colors[i].rgbRed = 255 - i;
      }
      break;
    case palette::Linear_palettes:
      /*
       * Linear palettes.
       */
      for (i = 0; i < 256; i++) {
        ret.colors[i].rgbBlue = ret.colors[i].rgbGreen = ret.colors[i].rgbRed =
            i;
      }
      break;
    case palette::GammaLog_palettes:
      /*
       * GammaLog palettes.
       */
      for (i = 0; i < 256; i++) {
        f = log10(pow((i / 255.0), 1.0) * 9.0 + 1.0) * 255.0;
        ret.colors[i].rgbBlue = ret.colors[i].rgbGreen = ret.colors[i].rgbRed =
            f;
      }
      break;
    case palette::Inversion_palette:
      /*
       * Inversion palette.
       */
      for (i = 0; i < 256; i++) {
        ret.colors[i].rgbBlue = ret.colors[i].rgbGreen = ret.colors[i].rgbRed =
            255 - i;
      }
      break;
    case palette::False_color_palette1:
      /*
       * False color palette #1.
       */
      for (i = 0; i < 256; i++) {
        r = (sin(
            (i / 255.0 * 360.0 - 120.0 > 0 ? i / 255.0 * 360.0 - 120.0 : 0)
                * DEG2RAD) * 0.5 + 0.5) * 255.0;
        g = (sin((i / 255.0 * 360.0 + 60.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        b = (sin((i / 255.0 * 360.0 + 140.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        ret.colors[i].rgbBlue = b;
        ret.colors[i].rgbGreen = g;
        ret.colors[i].rgbRed = r;
      }
      break;
    case palette::False_color_palette2:
      /*
       * False color palette #2.
       */
      for (i = 0; i < 256; i++) {
        r = (sin((i / 255.0 * 360.0 + 120.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        g = (sin((i / 255.0 * 360.0 + 240.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        b = (sin((i / 255.0 * 360.0 + 0.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        ret.colors[i].rgbBlue = b;
        ret.colors[i].rgbGreen = g;
        ret.colors[i].rgbRed = r;
      }
      break;
    case palette::False_color_palette3:
      /*
       * False color palette #3.
       */
      for (i = 0; i < 256; i++) {
        r = (sin((i / 255.0 * 360.0 + 240.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        g = (sin((i / 255.0 * 360.0 + 0.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        b = (sin((i / 255.0 * 360.0 + 120.0) * DEG2RAD) * 0.5 + 0.5) * 255.0;
        ret.colors[i].rgbBlue = b;
        ret.colors[i].rgbGreen = g;
        ret.colors[i].rgbRed = r;
      }
      break;

    case palette::False_color_palette4:
      /*
       * False color palette #4. Matlab JET
       */

      enum {
        nsep = 64,
        nvals = 192,
        n = 256
      };

      Eigen::Matrix<double, nvals, 1> vals;
      vals.setZero();

      int idx = 0;
      for (int i = 0; i < nsep; ++i) {
        vals(idx++, 0) = (i / (double) nsep);
      }
      vals.block<nsep, 1>(idx, 0).setConstant(1);
      idx += nsep;
      for (int i = nsep - 1; i >= 0; --i) {
        vals(idx++, 0) = i / (double) nsep;
      }

      Eigen::Matrix<int, nvals, 1> g;
      Eigen::Matrix<int, nvals, 1> r;
      Eigen::Matrix<int, nvals, 1> b;
      for (std::size_t i = 0; i < nvals; ++i) {
        g(i, 0) = ceil(nsep / 2) - 1 + i;
      }
      r = g + Eigen::Matrix<int, nvals, 1>::Constant(nsep);
      b = g - Eigen::Matrix<int, nvals, 1>::Constant(nsep);

      int idxr = 0;
      int idxg = 0;

      for (int i = 0; i < nvals; ++i) {
        if (r(i, 0) >= 0 && r(i, 0) < n)
          ret.colors[r(i, 0)].rgbRed = vals(idxr++, 0) * 255.;

        if (g(i, 0) >= 0 && g(i, 0) < n)
          ret.colors[g(i, 0)].rgbGreen = vals(idxg++, 0) * 255.;
      }

      int idxb = 0;
      int cntblue = 0;
      for (int i = 0; i < nvals; ++i) {
        if (b(i, 0) >= 0 && b(i, 0) < n)
          cntblue++;
      }

      for (int i = 0; i < nvals; ++i) {
        if (b(i, 0) >= 0 && b(i, 0) < n)
          ret.colors[b(i, 0)].rgbBlue = vals(nvals - 1 - cntblue + idxb++, 0)
              * 255.;
      }
      break;
  }
  return ret;
}
#undef DEG2RAD
