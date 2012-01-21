/*
 * Copyright (C) 2010 Georg Martius <georg.martius@web.de>
 * Copyright (C) 2010 Daniel G. Taylor <dan@programmer-art.org>
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * transform input video
 */

#include "libavutil/common.h"
#include "transform.h"
#include "libavutil/exper01.h"

#if 0
#  define SAFE_SUBSCRIPT(ys,xs,sstride)                                                                                                                    \
     (((subscript = (int)(ys) * (sstride) + (int)(xs)) >= size)?                                                                                         \
     av_log(NULL,AV_LOG_ERROR,"%s %s %d: Subscript out of range: %d  (width=%d height=%d)\n", __FILE__, __func__, __LINE__, subscript, width, height), 0 \
      : subscript)
#else
#  define SAFE_SUBSCRIPT(ys,xs,sstride) ((int)(ys) * (sstride) + (int)(xs))
#endif

#define INTERPOLATE_METHOD(name) \
    static uint8_t name(float x, float y, const uint8_t *src, \
                        int width, int height, int stride, uint8_t def)

#define PIXEL(img, x, y, w, h, stride, def) \
    ((x) < 0 || (y) < 0) ? (def) : \
    (((x) >= (w) || (y) >= (h)) ? (def) : \
    img[(x) + (y) * (stride)])

/**
 * Nearest neighbor interpolation
 */
INTERPOLATE_METHOD(interpolate_nearest)
{
    return PIXEL(src, (int)(x + 0.5), (int)(y + 0.5), width, height, stride, def);
}

/**
 * Bilinear interpolation
 */
INTERPOLATE_METHOD(interpolate_bilinear)
{
    int x_c, x_f, y_c, y_f;
    int v1, v2, v3, v4;

    if (x < -1 || x > width || y < -1 || y > height) {
        return def;
    } else {
        x_f = (int)x;
        x_c = x_f + 1;

        y_f = (int)y;
        y_c = y_f + 1;

        v1 = PIXEL(src, x_c, y_c, width, height, stride, def);
        v2 = PIXEL(src, x_c, y_f, width, height, stride, def);
        v3 = PIXEL(src, x_f, y_c, width, height, stride, def);
        v4 = PIXEL(src, x_f, y_f, width, height, stride, def);

        return (v1*(x - x_f)*(y - y_f) + v2*((x - x_f)*(y_c - y)) +
                v3*(x_c - x)*(y - y_f) + v4*((x_c - x)*(y_c - y)));
    }
}

/**
 * Biquadratic interpolation
 */
INTERPOLATE_METHOD(interpolate_biquadratic)
{
    int     x_c, x_f, y_c, y_f;
    uint8_t v1,  v2,  v3,  v4;
    float   f1,  f2,  f3,  f4;

    if (x < - 1 || x > width || y < -1 || y > height)
        return def;
    else {
        x_f = (int)x;
        x_c = x_f + 1;
        y_f = (int)y;
        y_c = y_f + 1;

        v1 = PIXEL(src, x_c, y_c, width, height, stride, def);
        v2 = PIXEL(src, x_c, y_f, width, height, stride, def);
        v3 = PIXEL(src, x_f, y_c, width, height, stride, def);
        v4 = PIXEL(src, x_f, y_f, width, height, stride, def);

        f1 = 1 - sqrt((x_c - x) * (y_c - y));
        f2 = 1 - sqrt((x_c - x) * (y - y_f));
        f3 = 1 - sqrt((x - x_f) * (y_c - y));
        f4 = 1 - sqrt((x - x_f) * (y - y_f));
        return (v1 * f1 + v2 * f2 + v3 * f3 + v4 * f4) / (f1 + f2 + f3 + f4);
    }
}

void avfilter_get_matrix(float x_shift, float y_shift, float angle, float zoom, float *matrix) {
    matrix[0] = zoom * cos(angle);
    matrix[1] = -sin(angle);
    matrix[2] = x_shift;
    matrix[3] = -matrix[1];
    matrix[4] = matrix[0];
    matrix[5] = y_shift;
    matrix[6] = 0;
    matrix[7] = 0;
    matrix[8] = 1;
}

void avfilter_add_matrix(const float *m1, const float *m2, float *result)
{
    int i;
    for (i = 0; i < 9; i++)
        result[i] = m1[i] + m2[i];
}

void avfilter_sub_matrix(const float *m1, const float *m2, float *result)
{
    int i;
    for (i = 0; i < 9; i++)
        result[i] = m1[i] - m2[i];
}

void avfilter_mul_matrix(const float *m1, float scalar, float *result)
{
    int i;
    for (i = 0; i < 9; i++)
        result[i] = m1[i] * scalar;
}

void avfilter_transform(const uint8_t *src, uint8_t *dst,
                        int src_stride, int dst_stride,
                        int width, int height, const float *matrix, uint8_t blank_default,
                        enum InterpolateMethod interpolate,
#ifdef EXPER01
                        enum FillMethod fill, DeshakeContextExtra *deshake_extra)
#else
                        enum FillMethod fill)
#endif
{
    int x, y, subscript;
    unsigned int size = width * height;
    float x_s, y_s;
    uint8_t def = blank_default;
    uint8_t (*func)(float, float, const uint8_t *, int, int, int, uint8_t) = NULL;
#ifdef EXPER01
    static int fuss=5;
    if (fuss) {
         fuss--;
//         av_log(NULL,AV_LOG_ERROR,"%s %s %d: (info) src_stride %d  dst_stride %d  width: %d  height: %d  matrix [%f %f %f %f %f %f %f %f %f]  src = %p  dst = %p\n",
//                __FILE__,__func__,__LINE__,src_stride, dst_stride, width, height, matrix[0], matrix[1], matrix[2], matrix[3], matrix[4], matrix[5], matrix[6], matrix[7], matrix[8],
//                src, dst);
    }
#endif
    switch(interpolate) {
        case INTERPOLATE_NEAREST:
            func = interpolate_nearest;
            break;
        case INTERPOLATE_BILINEAR:
            func = interpolate_bilinear;
            break;
        case INTERPOLATE_BIQUADRATIC:
            func = interpolate_biquadratic;
            break;
    }

    for (y = 0; y < height; y++) {
        for(x = 0; x < width; x++) {
            x_s = x * matrix[0] + y * matrix[1] + matrix[2];
            y_s = x * matrix[3] + y * matrix[4] + matrix[5];

#if 0
            av_log(NULL,AV_LOG_ERROR,"x_s %f  y_s %f\n", x_s, y_s);
#endif
            switch(fill) {
                case FILL_ORIGINAL:
                    def = src[y * src_stride + x];
                    break;
                case FILL_CLAMP:
                    y_s = av_clipf(y_s, 0, height - 1);
                    x_s = av_clipf(x_s, 0, width - 1);
                    def = src[SAFE_SUBSCRIPT(y_s, x_s, src_stride)];
                    break;
                case FILL_MIRROR:
                    y_s = (y_s < 0) ? -y_s : (y_s >= height) ? (height + height - y_s) : y_s;
                    x_s = (x_s < 0) ? -x_s : (x_s >= width) ? (width + width - x_s) : x_s;
                    def = src[SAFE_SUBSCRIPT(y_s, x_s, src_stride)];
                    break;
            }

            if (deshake_extra->optmask & 0x1) {
                 dst[y * dst_stride + x] = src[y * src_stride + x];  // Null transform; just copy.
            } else {
                 dst[y * dst_stride + x] = func(x_s, y_s, src, width, height, src_stride, def);
            }
        }
    }
}

