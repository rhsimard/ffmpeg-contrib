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
 * fast deshake / depan video filter
 *
 * This filter helps remove camera shake
 * from hand-holding a camera, bumping a tripod, moving on a vehicle, etc.
 * SAD (sum of absolute differences) methods are used to identify movement.
 *
 * Algorithm:
 *   - For each frame with one previous reference frame
 *       - For each block in the frame
 *           - If contrast > threshold then find likely motion vector
 *       - For all found motion vectors
 *           - Find most common, store as global motion vector
 *       - Find most likely rotation angle
 *       - Transform image along global motion
 *
 * @todo:
 *   - Fill frame edges based on previous/next reference frames
 *   - Fill frame edges by stretching image near the edges?
 *       - Can this be done quickly and look decent?
 *   - Near term: Investigate and implement as appropriate the zoom feature.
 *
 * Dark Shikari links to http://wiki.videolan.org/SoC_x264_2010#GPU_Motion_Estimation_2
 * for an algorithm similar to what could be used here to get the gmv
 * It requires only a couple diamond searches + fast downscaling
 *
 * Special thanks to Jason Kotenko for his help with the algorithm and my
 * inability to see simple errors in C code.
 */

#include "avfilter.h"
#include "libavutil/avstring.h"
#include "libavutil/common.h"
#include "libavutil/mem.h"
#include "libavutil/pixdesc.h"
#include "libavcodec/dsputil.h"
#include "libavutil/colorspace.h"
#include "libavutil/opt.h"

#include "transform.h"
#include "libavutil/exper01.h"     // EXPER01
#include "libavfilter/vf_deshake.h"

#if defined(EXPER01) && defined(USE_AVOPTION)
#define OFFSET(x) offsetof(DeshakeContext, x)
// Names chosen to match man page, except for multiple-option strings
static const AVOption deshake_options[]= {
    {"opts"          , "old-style option string"      , OFFSET(extra.oldopts)           , AV_OPT_TYPE_STRING, {.str=NULL                     },  CHAR_MIN, CHAR_MAX             },
    {"cx"            , "search area: left edge"       , OFFSET(cx)                      , AV_OPT_TYPE_INT,    {     -1                       },  -1, INT_MAX                    },
    {"cy"            , "search area: top edge"        , OFFSET(cy)                      , AV_OPT_TYPE_INT,    {     -1                       },  -1, INT_MAX                    },
    {"cw"            , "search area: width"           , OFFSET(cw)                      , AV_OPT_TYPE_INT,    {     -1                       },  -1, INT_MAX                    },
    {"ch"            , "search area: height"          , OFFSET(ch)                      , AV_OPT_TYPE_INT,    {     -1                       },  -1, INT_MAX                    },
    {"rx"            , "extent of x SAD search"       , OFFSET(rx)                      , AV_OPT_TYPE_INT,    {.dbl=RX_DEFAULT               },  RX_MIN, RX_MAX                 },
    {"ry"            , "extent of y SAD search"       , OFFSET(ry)                      , AV_OPT_TYPE_INT,    {.dbl=RY_DEFAULT               },  RY_MIN, RY_MAX                 },
    {"blocksize"     , "block size"                   , OFFSET(blocksize)               , AV_OPT_TYPE_INT,    {.dbl=BLOCKSIZE_DEFAULT        },  BLOCKSIZE_MIN, BLOCKSIZE_MAX   },
    {"edge"          , "edge style"                   , OFFSET(edge)                    , AV_OPT_TYPE_INT,    {.dbl=FILL_DEFAULT             },  0, FILL_COUNT-1                },
    {"contrast"      , "minimum contrast"             , OFFSET(contrast)                , AV_OPT_TYPE_INT,    {.dbl=CONTRAST_DEFAULT         },  CONTRAST_MIN, CONTRAST_MAX     },
    {"search"        , "search type"                  , OFFSET(search)                  , AV_OPT_TYPE_INT,    {.dbl=SEARCH_DEFAULT           },  EXHAUSTIVE, SEARCH_COUNT-1     },
    {"filename"      , "optional log file"            , OFFSET(extra.logfile)           , AV_OPT_TYPE_STRING, {.str=NULL                     },  CHAR_MIN, CHAR_MAX             },
    /* The following are newly-added items for test. */
    {"optmask"       , "option bitmask"               , OFFSET(extra.s_optmask)         , AV_OPT_TYPE_STRING, {.str=NULL                     },  CHAR_MIN, CHAR_MAX             },
    {"zoom"          , "test zoom factor"             , OFFSET(extra.zoom)              , AV_OPT_TYPE_FLOAT,  {.dbl=1.0                      },  0.0, 1000000.0                 },
    {"ref-frames"    , "subst. reference frame count" , OFFSET(reference_frames)        , AV_OPT_TYPE_INT,    {.dbl=REFERENCE_FRAMES_DEFAULT },  0, INT_MAX                     },
    {"diff-limit"    , "largest SAD diff"             , OFFSET(extra.diff_limit)        , AV_OPT_TYPE_INT,    {.dbl=DIFF_LIMIT_DEFAULT       },  DIFF_LIMIT_MIN, DIFF_LIMIT_MAX },
    {"interp-luma"   , "interpolation method, luma"   , OFFSET(extra.interpolate_luma)  , AV_OPT_TYPE_INT,    {.dbl=INTERP_LUMA_DEFAULT      },  0, INTERPOLATE_COUNT -1        },
    {"interp-chroma" , "interpolation method, chroma" , OFFSET(extra.interpolate_chroma), AV_OPT_TYPE_INT,    {.dbl=INTERP_CHROMA_DEFAULT    },  0, INTERPOLATE_COUNT -1        },
    {NULL}
};
#endif

static const char *get_deshake_name(void *ctx) {
    return "deshake";
}

/** Class description */
static const AVClass deshake_class = {
    .class_name  = "DeshakeContext",
    .item_name   = get_deshake_name,
#if defined(EXPER01) && defined(USE_AVOPTION)
    .option      = deshake_options,
#endif
};

static int cmp(const double *a, const double *b) {
    return *a < *b ? -1 : ( *a > *b ? 1 : 0 );
}

/** @name Support functions
 * @{*/

/**
 * Cleaned mean (cuts off 20% of values to remove outliers and then averages)
 *
 * @param values Array of values to process
 * @param count Number of values in the array
 * @return The calculated mean
 */
double clean_mean(double *values, int count)
{
    double mean = 0;
    int cut = count / 5;
    int x;

    if (count < 2)
        return(values[0]);
    qsort(values, count, sizeof(double), (void*)cmp);

    for (x = cut; x < count - cut; x++) {
        mean += values[x];
    }
    return mean / (count - cut * 2);
}

/**
 * Calculate the contrast for a given block.
 *
 * When searching for global motion we
 * really only care about the high contrast blocks, so using this method we
 * can actually skip blocks we don't care much about.
 * @param src Video luma data to process
 * @param x, y Location of the block
 * @param stride Distance within the data from one line to the next]
 * @param blocksize Size of the block
 * @param deshake Description of this instance of the filter
 * @return Calculated contrast value
 * @todo Check on the possibility that MMX could help here.
 */
int DEF_ADD_DESHAKE(block_contrast, uint8_t *src, int x, int y, int stride, int blocksize)
{
    int highest = 0;
    int lowest = 0;
    int i, j, pos;

    for (i = 0; i <= blocksize * 2; i++) {
        // We use a width of 16 here to match the libavcodec sad functions
        for (j = 0; i <= 15; i++) {
            pos = (y - i) * stride + (x - j);
            if (pos >= 0) {
                if (src[pos] < lowest)
                    lowest = src[pos];
                else if (src[pos] > highest) {
                    highest = src[pos];
                }
            }
        }
    }

    return highest - lowest;
}

/**
 * Find the rotation for a given block.
 * @param x, y Location of block
 * @param cx, cy (unused)
 * @param shift x and y displacements
 * @return rotation in radians
 */
double block_angle(int x, int y, int cx, int cy, IntMotionVector *shift)
{
    double a1, a2, diff;

    a1 = atan2(y - cy, x - cx);
    a2 = atan2(y - cy + shift->y, x - cx + shift->x);

    diff = a2 - a1;

    return (diff > M_PI)  ? diff - 2 * M_PI :
        (diff < -M_PI) ? diff + 2 * M_PI :
        diff;
}

/**@}*/

/** @name Motion search functions
 *  @{*/

/**
 * Find the most likely shift in motion between two frames for a given
 * macroblock.
 *
 * Tests each block against several shifts given by the rx
 * and ry attributes using a simple matrix of those shifts, and
 * chooses the most likely shift by the smallest difference in blocks.
 *
 * Search uses SAD methods (sum of absolute differences) to try to identify
 * areas which have moved.
 *
 * @param deshake Instance description
 * @param src1,src2 Data for the frames frame (luma plane)
 * @param bx, by Location of the block
 * @param stride Size of the data for one line within the data buffers
 * @param mv Motion vector struct to receive the results.
 * @see   libavcodec/dsputil.c and related files.
 */
static void find_block_motion(DeshakeContext *deshake, uint8_t *src1,
                              uint8_t *src2, int bx, int by, int stride,
                              IntMotionVector *mv)
{
    int x, y;
    int diff;
    int smallest = INT_MAX;
    int tmp, tmp2;

/** Call the previously-determined SAD routine to look for block displacements. */
#define CMP(i, j) deshake->c.sad[0](deshake, src1 + by * stride + bx,   \
                                    src2 + (j) * stride + (i), stride,  \
                                    deshake->blocksize)

    if (deshake->search == EXHAUSTIVE) {
        // Compare every possible position - this is sloooow!
        for (y = -deshake->ry; y <= deshake->ry; y++) {
            for (x = -deshake->rx; x <= deshake->rx; x++) {
                diff = CMP(bx - x, by - y);
                if (diff < smallest) {
                    smallest = diff;
                    mv->x = x;
                    mv->y = y;
                    LOG_IF_OPTMASK(OPT_LOG_FIND_BLOCK_MOTION_EXHAUSTIVE_LOOP, deshake,AV_LOG_ERROR,"%s %d: x=%4d y=%4d diff=%5d smallest=%5d\n", __func__,__LINE__,x, y, diff, smallest);
                }
            }
        }
    } else if (deshake->search == SMART_EXHAUSTIVE) {
        // Compare every other possible position and find the best match
        for (y = -deshake->ry + 1; y < deshake->ry - 2; y += 2) {
            for (x = -deshake->rx + 1; x < deshake->rx - 2; x += 2) {
                diff = CMP(bx - x, by - y);
                if (diff < smallest) {
                    smallest = diff;
                    mv->x = x;
                    mv->y = y;
                    LOG_IF_OPTMASK(OPT_LOG_FIND_BLOCK_MOTION_SMART_EXHAUSTIVE_LOOP, deshake,AV_LOG_ERROR,"%s %d: x=%4d y=%4d diff=%5d smallest=%5d\n", __func__,__LINE__,x, y, diff, smallest);
                }
            }
        }

        // Home in on the specific best match around the match we found above
        tmp = mv->x;
        tmp2 = mv->y;

        for (y = tmp2 - 1; y <= tmp2 + 1; y++) {
            for (x = tmp - 1; x <= tmp + 1; x++) {
                if (x == tmp && y == tmp2) {
                    continue;
                }
                diff = CMP(bx - x, by - y);
                if (diff < smallest) {
                    smallest = diff;
                    mv->x = x;
                    mv->y = y;
                    LOG_IF_OPTMASK(OPT_LOG_FIND_BLOCK_MOTION_SMART_EXHAUSTIVE_LOOP, deshake, AV_LOG_ERROR, "%s %d: x=%d y=%d diff=%d smallest=%d\n", __func__,__LINE__,x, y, diff, smallest);
                }
            }
        }
    }
    emms_c();  // Inform the CPU we're done doing MMX instructions.

    if (smallest > PARAM_IF(deshake->extra.diff_limit, 512)) {
        mv->x = -1;
        mv->y = -1;
    }
    LOG_IF_OPTMASK(OPT_LOG_FIND_BLOCK_MOTION_FINAL, NULL, AV_LOG_ERROR, "%s %d: Final: smallest =%4d  mv->x =%4d  mv->y =%4d\n", __func__, __LINE__, smallest, mv->x, mv->y);
    ADDTIME("exit","\n");
}

/**
 * Find estimated global motion for a scene.
 *
 * The global motion is estimated to be the
 * same as the motion of most blocks in the frame, so if most blocks
 * move one pixel to the right and two pixels down, this would yield a
 * motion vector (1, -2).
 * @param deshake Description of this instance
 * @param src1, src2  Frame data for comparison (luma plane)
 * @param width, height Dimensions of the images
 * @param stride Distance within the data buffer from one line to the next
 * @param t Transform struct to hold data for affine transform to be performed
 */

static void find_motion(DeshakeContext *deshake, uint8_t *src1, uint8_t *src2,
                        int width, int height, int stride, Transform *t)
{
    IntMotionVector mv = {0};
    int counts[BLOCKSIZE_MAX][BLOCKSIZE_MAX] = {{0}}, count_max_value = 0, contrast, pos, center_x = 0, center_y = 0, x, y, n;
    double *angles = av_malloc(sizeof(*angles) * width * height / (16 * deshake->blocksize)), p_x, p_y;

#ifdef EXPER01
    double pre_clip_x, pre_clip_y;
    ADDTIME("entry","\n");
    DESHAKE_WINNING_COUNT = -1;
#endif
    pos = 0;
    // Find motion for every block and store the motion vector in the counts
    for (y = deshake->ry; y < height - deshake->ry - (deshake->blocksize * 2); y += deshake->blocksize * 2) {
        // We use a width of 16 here to match the libavcodec SAD functions
        for (x = deshake->rx; x < width - deshake->rx - 16; x += 16) {
            // If the contrast is too low, just skip this block as it probably won't be very useful to us.
            contrast = CALL_ADD_DESHAKE(block_contrast, src2, x, y, stride, deshake->blocksize);
            if (contrast > deshake->contrast) {
                LOG_IF_OPTMASK(OPT_LOG_CALL_FIND_BLOCK_MOTION, deshake,AV_LOG_ERROR,"%s %d: (fcount =%3lu) calling find_block_motion:  x=%3d   y=%3d   stride=%3d   (contrast=%3d, src1 = %p, src2 = %p)\n",
                           __func__,__LINE__, fcount, x, y, stride, contrast, src1, src2);
                ADDTIME("find_block_motion call","x = %d  y = %d\n", x, y);
                find_block_motion(deshake, src1, src2, x, y, stride, &mv);
                if (mv.x != -1 && mv.y != -1) {
                    LOG_IF_OPTMASK(OPT_LOG_CALL_FIND_BLOCK_MOTION, NULL,AV_LOG_ERROR,"mv returns %d, %d\n", mv.x, mv.y);
                    if ((n = ++counts[mv.x + deshake->rx][mv.y + deshake->ry]) > count_max_value) {
                        count_max_value = n;
                        t->vector.x = mv.x;
                        t->vector.y = mv.y;
#ifdef EXPER01
                        DESHAKE_WINNING_COUNT = count_max_value;
                        DESHAKE_WINNING_MV.x = mv.x;
                        DESHAKE_WINNING_MV.y = mv.y;
#endif
                    }
                    if (x > deshake->rx && y > deshake->ry) {
                        angles[pos++] = block_angle(x, y, 0, 0, &mv);
                    }
                    center_x += mv.x;
                    center_y += mv.y;
                    CALL_ADD_DESHAKE(find_motion_generate_block_vectors, x, y, counts, &mv);
                }
            }
        }
    }
    ADDTIME("after motion search loop","\n");
    if (pos) {
        center_x /= pos;
        center_y /= pos;

        t->angle = clean_mean(angles, pos);
        if (t->angle < 0.001)
            t->angle = 0;
    } else {
        t->angle = 0;
    }

    p_x = (center_x - width / 2);
    p_y = (center_y - height / 2);
    t->vector.x += (cos(t->angle)-1)*p_x  -  sin(t->angle)   *p_y;
    t->vector.y +=  sin(t->angle)   *p_x  + (cos(t->angle)-1)*p_y;
#ifdef EXPER01
    pre_clip_x = t->vector.x;
    pre_clip_y = t->vector.y;
#endif
    // Clamp max shift & rotation?
    t->vector.x = av_clipf(t->vector.x, -deshake->rx * 2, deshake->rx * 2);
    t->vector.y = av_clipf(t->vector.y, -deshake->ry * 2, deshake->ry * 2);
    t->angle = av_clipf(t->angle, -0.1, 0.1);

    av_free(angles);
    LOG_IF_OPTMASK(OPT_LOG_FIND_MOTION_FINAL, deshake,AV_LOG_ERROR,
                   "%s %d: (Winner: count =%5d: x =%4d y =%4d) t->vector: { %8.3f %8.3f %8.3f } (pre-clip x and y: %8.3f %8.3f)  pos =%4d   center_x =%4d   center_y =%4d  px =%8.3f  py =%8.3f\n",
                   __func__,__LINE__,DESHAKE_WINNING_COUNT, DESHAKE_WINNING_MV.x, DESHAKE_WINNING_MV.y, t->vector.x,t->vector.y,t->angle, pre_clip_x, pre_clip_y,  pos, center_x, center_y, p_x, p_y);
    ADDTIME("exit","\n");
}
/**@}*/

/** @name System-defined processing functions
 * @{*/
/**
 * Draw a slice (unused);
 * @todo Docs say processing should happen here, not in end_frame; check up on this.
 */
static void draw_slice(AVFilterLink *link, int y, int h, int slice_dir)
{
}

/** All processing done here; see full description.
 *
 * Callback called by the system, normally after all slices have been sent.
 * This has an empty draw_slice and it appears that the output is being
 * deferred to this point.
 * @param link Definition of the link to the output of this filter instance.
 * @todo Check: Is this the right place for this? The docs seem to say that
 * processing should happen in draw_slice()
 */
static void end_frame(AVFilterLink *link)
{
    DeshakeContext *deshake = link->dst->priv;
    AVFilterBufferRef *in  = link->cur_buf;
    AVFilterBufferRef *out = link->dst->outputs[0]->out_buf;
    uint8_t *current_frame_data, *reference_frame_data;
    Transform t = {{0},0}, orig = {{0},0};
    float matrix[9];
    float alpha;
    char *statmsg;

#ifdef EXPER01
    MotionVector start, end;  // For arrows.
    ADDTIME("entry","\n");
#endif
    current_frame_data  = in->data[0];

    if (deshake->ref) {
        reference_frame_data = deshake->ref->data[0];
        alpha = 2.0 / deshake->reference_frames;

        // Find the most likely global motion for the current frame
        if (deshake->cx < 0 || deshake->cy < 0 || deshake->cw < 0 || deshake->ch < 0) {
            find_motion(deshake, reference_frame_data, in->data[0], link->w, link->h, in->linesize[0], &t);
        } else {
            deshake->cx = FFMIN(deshake->cx, link->w);
            deshake->cy = FFMIN(deshake->cy, link->h);
            if ((unsigned)deshake->cx + (unsigned)deshake->cw > link->w) deshake->cw = link->w - deshake->cx;
            if ((unsigned)deshake->cy + (unsigned)deshake->ch > link->h) deshake->ch = link->h - deshake->cy;
            // Quadword align right margin
            deshake->cw &= ~15;
            reference_frame_data += deshake->cy * in->linesize[0] + deshake->cx;
            current_frame_data += deshake->cy * in->linesize[0] + deshake->cx;
            find_motion(deshake, reference_frame_data, current_frame_data, deshake->cw, deshake->ch, in->linesize[0], &t);
        }
        ADDTIME("find motion done","\n");
        orig = t;   // Copy transform so we can output it later to compare to the smoothed value
        LOG_IF_OPTMASK(OPT_LOG_POST_FIND_MOTION_01, deshake,AV_LOG_ERROR,"%s %d: %8lu: %f %f %f   %f %f %f   %f %f %f   %f %f %f\n", \
                   __func__,__LINE__, fcount, orig.vector.x, deshake->avg.vector.x, t.vector.x, orig.vector.y, deshake->avg.vector.y, t.vector.y, orig.angle, deshake->avg.angle, t.angle, orig.zoom, deshake->avg.zoom, t.zoom);
        // Generate a one-sided moving exponential average
        deshake->avg.vector.x = alpha * t.vector.x + (1.0 - alpha) * deshake->avg.vector.x;
        deshake->avg.vector.y = alpha * t.vector.y + (1.0 - alpha) * deshake->avg.vector.y;
        deshake->avg.angle = alpha * t.angle + (1.0 - alpha) * deshake->avg.angle;
        deshake->avg.zoom = alpha * t.zoom + (1.0 - alpha) * deshake->avg.zoom;

#if defined(EXPER01)
        start.x = deshake->avg.vector.x;  // Arrows: store current points before transform.
        start.y = deshake->avg.vector.y;
        end.x = t.vector.x;
        end.y = t.vector.y;
        LOG_IF_OPTMASK(OPT_LOG_POST_FIND_MOTION_01, deshake,AV_LOG_ERROR,"%s %d: %8lu: %f %f %f   %f %f %f   %f %f %f   %f %f %f\n", \
                   __func__,__LINE__, fcount, orig.vector.x, deshake->avg.vector.x, t.vector.x, orig.vector.y, deshake->avg.vector.y, t.vector.y, orig.angle, deshake->avg.angle, t.angle, orig.zoom, deshake->avg.zoom, t.zoom);
#endif
        T_OP(t,-=,deshake->avg);  // Remove the average from the current motion to detect the motion that is not on purpose, just as jitter from bumping the camera

        t.vector.x *= -1;
        t.vector.y *= -1;
        t.angle *= -1;

        // Write statistics to file if requested.
        if (deshake->fp) {
#ifdef EXPER01
            statmsg = av_asprintf("%8lu: %6.2f  %6.2f  %6.2f    %6.2f  %6.2f  %6.2f    %6.2f  %6.2f  %6.2f    %6.2f  %6.2f  %6.2f\n", \
                                  fcount, orig.vector.x, orig.vector.y, deshake->avg.vector.x, deshake->avg.vector.y, t.vector.x, t.vector.y, orig.angle, deshake->avg.angle, t.angle, orig.zoom, deshake->avg.zoom, t.zoom);
#else
            statmsg = av_asprintf("          %6.2f  %6.2f  %6.2f    %6.2f  %6.2f  %6.2f    %6.2f  %6.2f  %6.2f    %6.2f  %6.2f  %6.2f\n", \
                                  orig.vector.x, orig.vector.y, deshake->avg.vector.x, deshake->avg.vector.y, t.vector.x, t.vector.y, orig.angle, deshake->avg.angle, t.angle, orig.zoom, deshake->avg.zoom, t.zoom);
#endif
            fwrite(statmsg, sizeof(char), strlen(statmsg), deshake->fp);
            av_freep(&statmsg);
        }

        T_OP(t,+=,deshake->last); // Turn relative current frame motion into absolute by adding it to the last absolute motion

        // Shrink motion by 10% to keep things centered in the camera frame
        t.vector.x *= 0.9;
        t.vector.y *= 0.9;
        t.angle *= 0.9;

        deshake->last = t;  // Store the last absolute motion information

#ifdef EXPER01
        // Generate a luma transformation matrix
        avfilter_get_matrix(t.vector.x, t.vector.y, t.angle, 1.0 + t.zoom / 100.0, matrix);
        // Transform the luma plane
        ADDTIME("before luma transform","\n");
        avfilter_transform(current_frame_data, out->data[0], in->linesize[0], out->linesize[0], link->w, link->h, matrix, 0, INTERPOLATE_METHOD_LUMA, deshake->edge, &deshake->extra);
        // Generate a chroma transformation matrix
        avfilter_get_matrix(t.vector.x / (link->w / CHROMA_WIDTH(link)), t.vector.y / (link->h / CHROMA_HEIGHT(link)), t.angle, 1.0 + t.zoom / 100.0, matrix);
        // Transform the chroma planes
        ADDTIME("before chroma transform","1");
        avfilter_transform(in->data[1], out->data[1], in->linesize[1], out->linesize[1], CHROMA_WIDTH(link), CHROMA_HEIGHT(link), matrix, 127, INTERPOLATE_METHOD_CHROMA, deshake->edge, &deshake->extra);
        ADDTIME("before chroma transform","2");
        avfilter_transform(in->data[2], out->data[2], in->linesize[2], out->linesize[2], CHROMA_WIDTH(link), CHROMA_HEIGHT(link), matrix, 127, INTERPOLATE_METHOD_CHROMA, deshake->edge, &deshake->extra);
        do_vectors(deshake, link, &t, &orig);
        ADDTIME("final processing","\n");
#else
        // Generate a luma transformation matrix
        avfilter_get_matrix(t.vector.x, t.vector.y, t.angle, 1.0 + t.zoom / 100.0, matrix);
        // Transform the luma plane
        avfilter_transform(current_frame_data, out->data[0], in->linesize[0], out->linesize[0], link->w, link->h, matrix, 0, INTERPOLATE_BILINEAR, deshake->edge);
        // Generate a chroma transformation matrix
        avfilter_get_matrix(t.vector.x / (link->w / CHROMA_WIDTH(link)), t.vector.y / (link->h / CHROMA_HEIGHT(link)), t.angle, 1.0 + t.zoom / 100.0, matrix);
        // Transform the chroma planes
        avfilter_transform(in->data[1], out->data[1], in->linesize[1], out->linesize[1], CHROMA_WIDTH(link), CHROMA_HEIGHT(link), matrix, 127, INTERPOLATE_BILINEAR, deshake->edge);
        avfilter_transform(in->data[2], out->data[2], in->linesize[2], out->linesize[2], CHROMA_WIDTH(link), CHROMA_HEIGHT(link), matrix, 127, INTERPOLATE_BILINEAR, deshake->edge);
#endif

        // Store the current frame as the reference frame for calculating the
        // motion of the next frame
        avfilter_unref_buffer(deshake->ref);
    } // if (deshake->ref)

    // Cleanup the old reference frame
    deshake->ref = in;
    // Draw the transformed frame information
    avfilter_draw_slice(link->dst->outputs[0], 0, link->h, 1);
    avfilter_end_frame(link->dst->outputs[0]);
    avfilter_unref_buffer(out);
#ifdef EXPER01
    fcount++;
    ADDTIME("done","\n");
    if (OPTMASK(OPT_DUMP_TIME_TRACK)) {
        dump_time_track(NULL);
    }
    delete_time_track();
#endif
}
/** @}*/


/** @name Setup and initialization
 * @{*/

/**
 * Set up a new instance.
 *
 * Called by the system to set up a new instance of the filter.
 *
 * @param ctx  Description of this instance
 * @param args  User args from the command line, if any
 * @param opaque Optionally used for filter-specific, arbitrary data.  Unused here.
 */
static av_cold int init(AVFilterContext *ctx, const char *args, void *opaque)
{
#define INIT_DEFAULTS(dummy)                    \
    do {                                        \
        deshake->rx = RX_DEFAULT;               \
        deshake->ry = RY_DEFAULT;               \
        deshake->edge = FILL_DEFAULT;           \
        deshake->blocksize = BLOCKSIZE_DEFAULT; \
        deshake->contrast = CONTRAST_DEFAULT;   \
        deshake->search = SEARCH_DEFAULT;       \
        deshake->cw = -1;                       \
        deshake->ch = -1;                       \
        deshake->cx = -1;                       \
        deshake->cy = -1;                       \
    }while(0)

#define INIT_EXPER01_DEFAULTS(dummy)                            \
    do {                                                        \
        deshake->extra.alpha = ALPHA_DEFAULT;                   \
        INTERPOLATE_METHOD_LUMA   = INTERP_LUMA_DEFAULT;        \
        INTERPOLATE_METHOD_CHROMA = INTERP_CHROMA_DEFAULT;      \
        deshake->reference_frames = REFERENCE_FRAMES_DEFAULT;   \
        deshake->extra.logfile    = filename;                   \
    }while(0)

    DeshakeContext *deshake = ctx->priv;
    char *statmsg;
    const char *argstring=args;
    char filename[PATH_MAX] = {0};

#ifdef EXPER01
    u_int32_t *p_32_01, *p_32_02, *p_32_03;
    int i, nopts = 0;
    OptmaskSelection *osp;

#  ifdef USE_AVOPTION
    int err;

    deshake->av_class = &deshake_class;  // Find out why this "discards pointer qualifiers"
    av_opt_set_defaults(deshake);

    if (args) {
        if (strncmp(args,"opts=",5)) {
            argstring = NULL;
            if ((err = (av_set_options_string(deshake, args, "=", ":"))) < 0) {
                av_log(ctx, AV_LOG_ERROR, "Error parsing options string: '%s'\n", args);
                return err;
            }
            deshake->extra.optmask = (deshake->extra.s_optmask ? strtoull(deshake->extra.s_optmask,NULL,0) : 0);
        } else {
            argstring = args+5;
        }
    } else {
        argstring = NULL;
    }
#  else /* ifdef USE_AVOPTION */

    INIT_DEFAULTS();
    INIT_EXPER01_DEFAULTS();

#  endif  /* ifdef USE_AVOPTION */

    icount++;

#else  /* ifdef EXPER01 */

    char filename[PATH_MAX] = {0};
    INIT_DEFAULTS();

#endif    /* if(n)def EXPER01 */

    if (argstring && *argstring) {
#ifdef EXPER01
        int nconv;
        nconv = sscanf(argstring, "%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%f:%Li:%f:%255s",
                       &deshake->cx, &deshake->cy, &deshake->cw, &deshake->ch,
                       &deshake->rx, &deshake->ry, (int *)&deshake->edge,
                       &deshake->blocksize, &deshake->contrast, (int *)&deshake->search, &DESHAKE_ZOOM, (long long int*)&deshake->extra.optmask, &DESHAKE_ALPHA, filename);
        if (DESHAKE_ALPHA >= 0) {
            DESHAKE_ALPHA = av_clipf(DESHAKE_ALPHA,0.01,0.99);
        }
        global_option_01 = OPTMASK(OPT_GLOBAL_01);
        global_option_02 = OPTMASK(OPT_GLOBAL_02);
#  ifdef USE_AVOPTION
        // System will run av_free on all string and binary elements of the context.
        av_assert0(LOGFILE==NULL);
        if (LOGFILE == NULL) {
            LOGFILE = av_strdup(filename);
        }
#  endif
#else
        sscanf(argstring, "%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%255s",
               &deshake->cx, &deshake->cy, &deshake->cw, &deshake->ch,
               &deshake->rx, &deshake->ry, (int *)&deshake->edge,
               &deshake->blocksize, &deshake->contrast, (int *)&deshake->search, filename);
        deshake->blocksize = av_clip(deshake->blocksize, BLOCKSIZE_MIN, BLOCKSIZE_MAX);
        deshake->rx = av_clip(deshake->rx, RX_MIN, RX_MAX);
        deshake->ry = av_clip(deshake->ry, RY_MIN, RY_MAX);
        deshake->edge = av_clip(deshake->edge, FILL_BLANK, FILL_COUNT - 1);
        deshake->contrast = av_clip(deshake->contrast, CONTRAST_MIN, CONTRAST_MAX);
        deshake->search = av_clip(deshake->search, EXHAUSTIVE, SEARCH_COUNT - 1);
#endif
    }
    deshake->blocksize /= 2;
    if (LOGFILE && *LOGFILE) {
        if ((deshake->fp = fopen(LOGFILE, "w")) != NULL) {
            statmsg = av_asprintf("          %7s %7s %7s   %7s %7s %7s   %7s %7s %7s   %7s %7s %7s\n\n",
                                  "Or x", "Or y", "Av x", "Av y", "Fin x", "Fin y", "Or ang", "Av ang", "Fin ang", "Or zm", "Av zm", "Fin zm");
            fwrite(statmsg, sizeof(char), strlen(statmsg), deshake->fp);
            av_freep(&statmsg);
        } else {
            av_log(deshake,AV_LOG_ERROR,"Failed to open stat file %s: %s\n", LOGFILE, strerror(errno));
        }
    }

    // Quadword align left edge of box for MMX code; adjust width if necessary
    // to keep right margin
    if (deshake->cx > 0) {
        deshake->cw += deshake->cx - (deshake->cx & ~15);
        deshake->cx &= ~15;
    }

#ifdef EXPER01
    p_32_01 = (u_int32_t*)&deshake->extra.optmask;
    av_log(ctx, AV_LOG_INFO, "cx: %d, cy: %d, cw: %d, ch: %d, rx: %d, ry: %d, edge: %d blocksize: %d contrast: %d search: %d  zoom: %f  option mask: 0x%08lx %08lx  alpha: %f  ref frames: %d  diff-limit: %d  interp: %d,%d%s%s\n",
           deshake->cx, deshake->cy, deshake->cw, deshake->ch,
           deshake->rx, deshake->ry, deshake->edge, deshake->blocksize * 2, deshake->contrast, deshake->search, DESHAKE_ZOOM,
           (unsigned long)p_32_01[1], (unsigned long)p_32_01[0], DESHAKE_ALPHA, deshake->reference_frames, deshake->extra.diff_limit, deshake->extra.interpolate_luma, deshake->extra.interpolate_chroma,
           (deshake->extra.logfile && *deshake->extra.logfile? "  log file: " : " oh "), (deshake->extra.logfile && *deshake->extra.logfile? deshake->extra.logfile : " no "));
    if (deshake->extra.optmask) {
        av_log(ctx,AV_LOG_VERBOSE,"Enabled deshake options: ");
        for (i = 0 ; i < get_n_optmask_selections() ; i++) {
            osp = &optmask_selections[i];
            if (deshake->extra.optmask & osp->mask) {
                av_log(NULL,AV_LOG_VERBOSE,"%s%s",(nopts? ", " : ""), osp->shortdescr);
                nopts++;
            }
        }
        av_log(NULL,AV_LOG_VERBOSE,"\n");
    }
#else
    av_log(ctx, AV_LOG_INFO, "cx: %d, cy: %d, cw: %d, ch: %d, rx: %d, ry: %d, edge: %d blocksize: %d contrast: %d search: %d  %s\n",
           deshake->cx, deshake->cy, deshake->cw, deshake->ch,
           deshake->rx, deshake->ry, deshake->edge, deshake->blocksize * 2, deshake->contrast, deshake->search, (*LOGFILE? LOGFILE : ""));
#endif

    return 0;
}

/** Inform the system of what formats and layouts are supported.
 *
 * Queries formats/layouts supported by the filter and its pads, and sets
 * the in_formats/in_chlayouts for links connected to its output pads,
 * and out_formats/out_chlayouts for links connected to its input pads.
 *
 * @param ctx Description of this instance
 * @return zero on success, a negative value corresponding to an
 * AVERROR code otherwise
 */
static int query_formats(AVFilterContext *ctx)
{
    enum PixelFormat pix_fmts[] = {
        PIX_FMT_YUV420P,  PIX_FMT_YUV422P,  PIX_FMT_YUV444P,  PIX_FMT_YUV410P,
        PIX_FMT_YUV411P,  PIX_FMT_YUV440P,  PIX_FMT_YUVJ420P, PIX_FMT_YUVJ422P,
        PIX_FMT_YUVJ444P, PIX_FMT_YUVJ440P, PIX_FMT_NONE
    };

    avfilter_set_common_pixel_formats(ctx, avfilter_make_format_list(pix_fmts));

    return 0;
}

/** Set up the properties for this filter instance.
 *
 * Obtains the proper SAD function to be used
 * for block searches.
 * @param link Description of the link between this filter and
 * the one it is connected to
 * @return Zero for success
 */
static int config_props(AVFilterLink *link)
{
    DeshakeContext *deshake = link->dst->priv;

    deshake->ref = NULL;
    T_SET(deshake->last, =,  0);
    if ((deshake->avctx = avcodec_alloc_context3(NULL)) == NULL) {
        av_log(deshake,AV_LOG_ERROR,"%s %d: Allocation of codec context failed.\n", __func__, __LINE__);
        return -1;
    }
    dsputil_init(&deshake->c, deshake->avctx);
    return 0;
}

/** Shut down the shop */
static av_cold void uninit(AVFilterContext *ctx)
{
    DeshakeContext *deshake = ctx->priv;

    avfilter_unref_buffer(deshake->ref);
    if (deshake->fp)
        fclose(deshake->fp);
#if defined(EXPER01) && defined(USE_AVOPTION)
    av_opt_free(deshake);
#endif
}

/** Interface to the system */
AVFilter avfilter_vf_deshake = {
    .name      = "deshake",
    .description = NULL_IF_CONFIG_SMALL("Stabilize shaky video."),

    .priv_size = sizeof(DeshakeContext),

    .init = init,
    .uninit = uninit,
    .query_formats = query_formats,

    .inputs    = (const AVFilterPad[]) {{ .name       = "default",
                                          .type             = AVMEDIA_TYPE_VIDEO,
                                          .draw_slice       = draw_slice,
                                          .end_frame        = end_frame,
                                          .config_props     = config_props,
                                          .min_perms        = AV_PERM_READ, },
                                        { .name = NULL}},

    .outputs   = (const AVFilterPad[]) {{ .name       = "default",
                                          .type             = AVMEDIA_TYPE_VIDEO, },
                                        { .name = NULL}},
};
/** @}*/
