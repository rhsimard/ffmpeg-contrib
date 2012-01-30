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

#ifndef AVFILTER_TRANSFORM_H
#define AVFILTER_TRANSFORM_H

#include "libavutil/exper01.h"

/**
 * @file
 * transform input video
 */

#if defined(EXPER01) && defined(MATRIX_RECTANGULAR)
#  define MATRIX_ROWS (3)
#  define MATRIX_COLS (3)
#else
#  define MATRIX_SIZE (9)
#endif

#if defined(EXPER01) && defined (MATRIX_RECTANGULAR)
// Just for experimentation; the single-dimension array is fine.
typedef float matrix_t[MATRIX_ROWS][MATRIX_COLS];
#else
/** All matrices are defined as a single 9-item block of contiguous memory. For
 * example, the identity matrix would be:
 *
 *     float *matrix = {1, 0, 0,
 *                      0, 1, 0,
 *                      0, 0, 1};
 */
#endif


enum InterpolateMethod {
    INTERPOLATE_NEAREST,        //< Nearest-neighbor (fast)
    INTERPOLATE_BILINEAR,       //< Bilinear
    INTERPOLATE_BIQUADRATIC,    //< Biquadratic (best)
    INTERPOLATE_COUNT,          //< Number of interpolation methods
};

// Shortcuts for the fastest and best interpolation methods
#define INTERPOLATE_DEFAULT INTERPOLATE_BILINEAR
#define INTERPOLATE_FAST    INTERPOLATE_NEAREST
#define INTERPOLATE_BEST    INTERPOLATE_BIQUADRATIC

enum FillMethod {
    FILL_BLANK,         //< Fill zeroes at blank locations
    FILL_ORIGINAL,      //< Original image at blank locations
    FILL_CLAMP,         //< Extruded edge value at blank locations
    FILL_MIRROR,        //< Mirrored edge at blank locations
    FILL_COUNT,         //< Number of edge fill methods
};

/**
 * Get an affine transformation matrix from a given translation and rotation.
 *
 * The matrix will look like:
 *
 * [ zoom * cos(angle),           -sin(angle),     x_shift,
 *          sin(angle),            cos(angle),     y_shift,
 *                   0,                     0,           1 ]
 *
 * @param x_shift horizontal translation
 * @param y_shift vertical translation
 * @param angle   rotation in radians
 * @param matrix  9-item affine transformation matrix
 */
void avfilter_get_matrix(float x_shift, float y_shift, float angle, float *matrix);

/**
 * Add two matrices together. result = m1 + m2.
 *
 * @param m1     9-item transformation matrix
 * @param m2     9-item transformation matrix
 * @param result 9-item transformation matrix
 */
void avfilter_add_matrix(const float *m1, const float *m2, float *result);

/**
 * Subtract one matrix from another. result = m1 - m2.
 *
 * @param m1     9-item transformation matrix
 * @param m2     9-item transformation matrix
 * @param result 9-item transformation matrix
 */
void avfilter_sub_matrix(const float *m1, const float *m2, float *result);

/**
 * Multiply a matrix by a scalar value. result = m1 * scalar.
 *
 * @param m1     9-item transformation matrix
 * @param scalar a number
 * @param result 9-item transformation matrix
 */
void avfilter_mul_matrix(const float *m1, float scalar, float *result);

/**
 * Do an affine transformation with the given interpolation method. This
 * multiplies each vector [x,y,1] by the matrix and then interpolates to
 * get the final value.
 *
 * @param src            source image
 * @param dst            destination image
 * @param src_stride     source image line size in bytes
 * @param dst_stride     destination image line size in bytes
 * @param width          image width in pixels
 * @param height         image height in pixels
 * @param matrix         9-value affine transformation matrix
 * @param def            value for pixels outside the transformed image when FILL_BLANK chosen
 * @param interpolate    pixel interpolation method
 * @param fill           edge fill method
 */
void avfilter_transform(const uint8_t *src, uint8_t *dst,
                        int src_stride, int dst_stride,
                        int width, int height, const float *matrix, uint8_t def,
                        enum InterpolateMethod interpolate,
#ifdef EXPER01
                        enum FillMethod fill, DeshakeContextExtra *deshake_extra);
#else
enum FillMethod fill);
#endif

#endif /* AVFILTER_TRANSFORM_H */
