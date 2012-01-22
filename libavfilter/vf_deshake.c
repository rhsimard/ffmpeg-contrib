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
 * SAD block-matching motion compensation to fix small changes in
 * horizontal and/or vertical shift. This filter helps remove camera shake
 * from hand-holding a camera, bumping a tripod, moving on a vehicle, etc.
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
 * @TODO:
 *   - Fill frame edges based on previous/next reference frames
 *   - Fill frame edges by stretching image near the edges?
 *       - Can this be done quickly and look decent?

 * @todo: Near term: Investigate and implement as appropriate the zoom feature.
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

#include "transform.h"
#include "libavutil/exper01.h"     // EXPER01


// Parameter limits per man page
#define RX_MAX             (64)
#define RX_MIN             (0)
#define RX_DEFAULT         (16)
#define RY_MAX             (64)
#define RY_MIN             (0)
#define RY_DEFAULT         (16)
#define BLOCKSIZE_MAX      (128)
// Note: man page says this is 4, but in the existing code it's 8.  Staying with the code for now.
#define BLOCKSIZE_MIN      (8)
#define BLOCKSIZE_DEFAULT  (8)
#define CONTRAST_MAX       (255)
#define CONTRAST_MIN       (1)
#define CONTRAST_DEFAULT   (125)
#define SEARCH_DEFAULT     (EXHAUSTIVE)
#define ALPHA_MAX          (0.99)
#define ALPHA_DEFAULT      (-1.0)

#define CHROMA_WIDTH(link)  -((-link->w) >> av_pix_fmt_descriptors[link->format].log2_chroma_w)
#define CHROMA_HEIGHT(link) -((-link->h) >> av_pix_fmt_descriptors[link->format].log2_chroma_h)

enum SearchMethod {
    EXHAUSTIVE,        ///< Search all possible positions
    SMART_EXHAUSTIVE,  ///< Search most possible positions (faster)
    SEARCH_COUNT
};

#if 0
typedef struct {
    int x;             ///< Horizontal shift
    int y;             ///< Vertical shift
} IntMotionVector;
#endif

typedef struct {
    double x;             ///< Horizontal shift
    double y;             ///< Vertical shift
} MotionVector;

typedef struct {
    MotionVector vector;  ///< Motion vector
    double angle;         ///< Angle of rotation
    double zoom;          ///< Zoom percentage
} Transform;

#define T_OP(dest,op,src)                       \
     do {                                       \
          dest.vector.x op src.vector.x;        \
          dest.vector.y op src.vector.y;        \
          dest.angle    op src.angle;           \
          dest.zoom     op src.zoom;            \
     } while(0)

#define T_SET(dest,op,val)                      \
     do {                                       \
          dest.vector.x op val;                 \
          dest.vector.y op val;                 \
          dest.angle    op val;                 \
          dest.zoom     op val;                 \
     } while(0)

typedef struct {
    AVClass av_class;
    AVFilterBufferRef *ref;    ///< Previous frame
    int rx;                    ///< Maximum horizontal shift
    int ry;                    ///< Maximum vertical shift
    enum FillMethod edge;      ///< Edge fill method
    int blocksize;             ///< Size of blocks to compare
    int contrast;              ///< Contrast threshold
    enum SearchMethod search;  ///< Motion search method
    AVCodecContext *avctx;
    DSPContext c;              ///< Context providing optimized SAD methods
    Transform avg, last;       ///< Transforms: running average and last frame
    int reference_frames;              ///< Number of reference frames (defines averaging window)
    FILE *fp;
    int cw;                    ///< Crop motion search to this box
    int ch;
    int cx;
    int cy;
#ifdef EXPER01
     DeshakeContextExtra  extra;
#endif
}DeshakeContext;

#ifdef EXPER01
static void draw_vectors(DeshakeContext *deshake, AVFilterBufferRef *avbuf, int w, int h, int stride, Transform *t, Transform *orig, int normalizing_scale, int color, int highlight_color);
static void draw_vectors_r(DeshakeContext *deshake, const ArrowAnnotation *root, AVFilterBufferRef *avbuf, int w, int h, int stride, int normalizing_scale, int color, int highlight_color);

ArrowAnnotation *arrow_root = NULL;
#endif

static int cmp(const double *a, const double *b)
{
    return *a < *b ? -1 : ( *a > *b ? 1 : 0 );
}

/**
 * Cleaned mean (cuts off 20% of values to remove outliers and then averages)
 */
static double clean_mean(double *values, int count)
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
 * Find the contrast of a given block. When searching for global motion we
 * really only care about the high contrast blocks, so using this method we
 * can actually skip blocks we don't care much about.
 */
#ifdef EXPER01
static int block_contrast(uint8_t *src, int x, int y, int stride, int blocksize, DeshakeContext *deshake)
#else
static int block_contrast(uint8_t *src, int x, int y, int stride, int blocksize, DeshakeContext)
#endif
{
    int highest = 0;
    int lowest = 0;
    int i, j, pos;

    for (i = 0; i <= blocksize * 2; i++) {
        // We use a width of 16 here to match the libavcodec sad functions
        for (j = 0; i <= 15; i++) {
            pos = (y - i) * stride + (x - j);
            if (src[pos] < lowest)
                lowest = src[pos];
            else if (src[pos] > highest) {
                highest = src[pos];
            }
        }
    }

    return highest - lowest;
}

/**
 * Find the rotation for a given block.
 */
static double block_angle(int x, int y, int cx, int cy, IntMotionVector *shift)
{
    double a1, a2, diff;

    a1 = atan2(y - cy, x - cx);
    a2 = atan2(y - cy + shift->y, x - cx + shift->x);

    diff = a2 - a1;

    return (diff > M_PI)  ? diff - 2 * M_PI :
           (diff < -M_PI) ? diff + 2 * M_PI :
           diff;
}

/**
 * Find the most likely shift in motion between two frames for a given
 * macroblock. Test each block against several shifts given by the rx
 * and ry attributes. Searches using a simple matrix of those shifts and
 * chooses the most likely shift by the smallest difference in blocks.
 */
static void find_block_motion(DeshakeContext *deshake, uint8_t *src1,
                              uint8_t *src2, int cx, int cy, int stride,
                              IntMotionVector *mv)
{
    int x, y;
    int diff;
    int smallest = INT_MAX;
    int tmp, tmp2;

    #define CMP(i, j) deshake->c.sad[0](deshake, src1 + cy * stride + cx, \
                                        src2 + (j) * stride + (i), stride, \
                                        deshake->blocksize)

    if (deshake->search == EXHAUSTIVE) {
        // Compare every possible position - this is sloooow!
        for (y = -deshake->ry; y <= deshake->ry; y++) {
            for (x = -deshake->rx; x <= deshake->rx; x++) {
                diff = CMP(cx - x, cy - y);
                if (diff < smallest) {
                    smallest = diff;
                    mv->x = x;
                    mv->y = y;
#ifdef EXPER01
                    if (OPTMASK(OPT_LOG_SEARCH_LOOP)) {
                         av_log(deshake,AV_LOG_ERROR,"%s %d: x=%d y=%d diff=%d smallest=%d\n", __func__,__LINE__,x, y, diff, smallest);
                    }
#endif
                }
            }
        }
    } else if (deshake->search == SMART_EXHAUSTIVE) {
        // Compare every other possible position and find the best match
        for (y = -deshake->ry + 1; y < deshake->ry - 2; y += 2) {
            for (x = -deshake->rx + 1; x < deshake->rx - 2; x += 2) {
                diff = CMP(cx - x, cy - y);
                if (diff < smallest) {
                    smallest = diff;
                    mv->x = x;
                    mv->y = y;
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
#ifdef EXPER01
                    if (OPTMASK(OPT_LOG_SECONDARY_SEARCH_LOOP)) {
                         av_log(deshake,AV_LOG_ERROR,"%s %d: x=%d y=%d diff=%d smallest=%d\n", __func__,__LINE__,x, y, diff, smallest);
                    }
#endif

                diff = CMP(cx - x, cy - y);
                if (diff < smallest) {
                    smallest = diff;
                    mv->x = x;
                    mv->y = y;
#ifdef EXPER01
                    if (OPTMASK(OPT_LOG_SECONDARY_SEARCH_LOOP_END)) {
                         av_log(deshake,AV_LOG_ERROR,"%s %d: x=%d y=%d diff=%d smallest=%d\n", __func__,__LINE__,x, y, diff, smallest);
                    }
#endif
                }
            }
        }
    }

    if (smallest > 512) {
        mv->x = -1;
        mv->y = -1;
    }
    emms_c();
#ifdef EXPER01
    if (OPTMASK(OPT_LOG_FIND_BLOCK_MOTION_FINAL)) {
    av_log(NULL, AV_LOG_ERROR, "%s %d: Final: smallest =%4d  mv->x =%4d  mv->y =%4d\n", __func__, __LINE__, smallest, mv->x, mv->y);
    }
#endif
}

/**
 * Find the estimated global motion for a scene given the most likely shift
 * for each block in the frame. The global motion is estimated to be the
 * same as the motion from most blocks in the frame, so if most blocks
 * move one pixel to the right and two pixels down, this would yield a
 * motion vector (1, -2).
 */
#define COUNTS_SIZE_X (128)
#define COUNTS_SIZE_Y (128)

static void find_motion(DeshakeContext *deshake, uint8_t *src1, uint8_t *src2,
                        int width, int height, int stride, Transform *t)
{
    int x, y;
    IntMotionVector mv = {0, 0};
    int counts[COUNTS_SIZE_X][COUNTS_SIZE_Y] = {{0}};
    int count_max_value = 0;
    int contrast;
#ifdef EXPER01
    int arrow_index;
    double pre_clip_x, pre_clip_y;
    ArrowAnnotation *arrow, **a2;
    unsigned long counter = 0;
#endif

    int pos;
    double *angles = av_malloc(sizeof(*angles) * width * height / (16 * deshake->blocksize));
    int center_x = 0, center_y = 0;
    double p_x, p_y;

#ifdef EXPER01
    DESHAKE_WINNING_COUNT = -1;
#endif

    pos = 0;
    // Find motion for every block and store the motion vector in the counts
    for (y = deshake->ry; y < height - deshake->ry - (deshake->blocksize * 2); y += deshake->blocksize * 2) {
        // We use a width of 16 here to match the libavcodec sad functions
        for (x = deshake->rx; x < width - deshake->rx - 16; x += 16) {
            // If the contrast is too low, just skip this block as it probably
            // won't be very useful to us.
#ifdef EXPER01
             contrast = block_contrast(src2, x, y, stride, deshake->blocksize, deshake);
#else
             contrast = block_contrast(src2, x, y, stride, deshake->blocksize);
#endif
            if (contrast > deshake->contrast) {
                 // av_log(NULL, AV_LOG_ERROR, "contrast: %d\n", contrast);
                find_block_motion(deshake, src1, src2, x, y, stride, &mv);
                if (mv.x != -1 && mv.y != -1) {
                    counts[mv.x + deshake->rx][mv.y + deshake->ry]++;
                    if (x > deshake->rx && y > deshake->ry) {
                        angles[pos++] = block_angle(x, y, 0, 0, &mv);
                    }

                    center_x += mv.x;
                    center_y += mv.y;
#ifdef EXPER01
                    if (OPTMASK(OPT_BLOCK_VECTORS)) {
                         static int fuss=10;
                         if ((arrow = av_malloc(sizeof(ArrowAnnotation)))) {
                              float fx = 1.0, fy = 1.0;
                              arrow->startx = x;
                              arrow->starty = y;
                              if (OPTMASK(OPT_BLOCK_VECTORS_NORMALIZE)) {
                                   arrow->endx = x + (int)((float)mv.x * 8.0/deshake->rx); //+ deshake->rx; // Normalize for better visiblity.
                                   arrow->endy = y + (int)((float)mv.y * 8.0/deshake->ry);
                                   fx = (float)arrow->endx/(x + (float)mv.x + deshake->rx);
                                   fy = (float)arrow->endy/(y + (float)mv.y + deshake->ry);
                              } else {
                                   arrow->endx = x + mv.x;
                                   arrow->endy = y + mv.y;
                              }
                              if (fuss && (mv.x || mv.y)) {
//                                   av_log(deshake,AV_LOG_ERROR,"%s %d: fx=%f, fy=%f, x=%d, mv.x=%d, deshake->rx=%d, startx=%d,  endx=%d, y=%d, mv.y=%d, deshake->ry=%d, starty=%d, endy=%d\n",
//                                          __func__,__LINE__, fx, fy, x, mv.x, deshake->rx, arrow->startx, arrow->endx, y, mv.y, deshake->ry, arrow->starty, arrow->endy);
                                   fuss--;
                              }
                              arrow->count = counts[mv.x + deshake->rx][mv.y + deshake->ry];
                              arrow->highlight = 0;
                              arrow->next = NULL;
                              arrow->annotation = av_asprintf("%d %d %d %d counts=%d", x, y, mv.x, mv.y, counts[mv.x + deshake->rx][mv.y + deshake->ry]);
                              arrow_index=0;
                              for (a2 = &arrow_root ; *a2  ; a2 = &(*a2)->next, arrow_index++) {
                                   if (OPTMASK(OPT_LOG_BLOCK_VECTORS_INNER_LOOP)) {
//                                        av_log(deshake,AV_LOG_ERROR,"%s %d: index=%d a2 = %p  *a2 = %p  (*a2)->next = %p  arrow_root is at %p  arrow_root = %p  x = %4d  y = %4d  mv.x = %4d  mv.y = %4d  ...rx = %4d  ...ry = %4d\n",
                                        av_log(deshake,AV_LOG_ERROR,"%s %d: index=%d x=%d  y=%d  mv.x=%d mv.y=%d rx=%d ry=%d\n",
                                               __func__, __LINE__, ((*a2)? (*a2)->index : -1), /* a2, *a2, ((*a2)? (*a2)->next : NULL),  &arrow_root, arrow_root, */ x, y, mv.x, mv.y, deshake->rx, deshake->ry);
                                   }
                              }
                              if (OPTMASK(OPT_LOG_BLOCK_VECTORS_LOOP_FINAL)) {
                                   av_log(deshake,AV_LOG_ERROR,"%s %d: ar-index=%d start=%d,%d end=%d,%d count=%d x=%d y=%d mv.x=%d mv.y=%d rx=%d ry=%d) annotation=\"%s\"\n",
                                          __func__,__LINE__,arrow_index, arrow->startx, arrow->starty, arrow->endx, arrow->endy, arrow->count, x, y, mv.x, mv.y, deshake->rx, deshake->ry,  arrow->annotation);
                              }
                         arrow->index = arrow_index;
                         (*a2) = arrow;
                         } else {
                              av_log(deshake,AV_LOG_ERROR,"%s %d: arrow annotation alloc failure.\n", __func__, __LINE__);
                         }
                    }
#endif
                }
            }
        }
    }
    if (pos) {
         center_x /= pos;
         center_y /= pos;

         t->angle = clean_mean(angles, pos);
         if (t->angle < 0.001)
              t->angle = 0;
    } else {
         t->angle = 0;
    }

    // Find the most common motion vector in the frame and use it as the gmv
    for (y = deshake->ry * 2; y >= 0; y--) {
        for (x = 0; x < deshake->rx * 2 + 1; x++) {
            if (counts[x][y] > count_max_value) {
                t->vector.x = x - deshake->rx;
                t->vector.y = y - deshake->ry;
                count_max_value = counts[x][y];
#ifdef EXPER01
                DESHAKE_WINNING_COUNT = count_max_value;
                DESHAKE_WINNING_MV.x = x;
                DESHAKE_WINNING_MV.y = y;
#endif
            }
        }
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
#ifdef EXPER01
    if (OPTMASK(OPT_LOG_FIND_MOTION_FINAL)) {
         av_log(deshake,AV_LOG_ERROR,"%s %d: (Winner: count =%5d: x =%4d y =%4d) t->vector: { %8.3f %8.3f %8.3f } (pre-clip x and y: %8.3f %8.3f)  pos =%4d   center_x =%4d   center_y =%4d  px =%8.3f  py =%8.3f\n",__func__,__LINE__,DESHAKE_WINNING_COUNT, DESHAKE_WINNING_MV.x, DESHAKE_WINNING_MV.y,
                t->vector.x,t->vector.y,t->angle, pre_clip_x, pre_clip_y,  pos, center_x, center_y, p_x, p_y);
    }
#endif
}

static void end_frame(AVFilterLink *link)
{
    DeshakeContext *deshake = link->dst->priv;
    AVFilterBufferRef *in  = link->cur_buf;
    AVFilterBufferRef *out = link->dst->outputs[0]->out_buf;
//    int width, height, src_stride, dst_stride, chroma_width, chroma_height;
    uint8_t *src1, *src2;
    Transform t = {{0},0}, orig = {{0},0};
    float matrix[9];
    float alpha;
    char *statmsg;

#ifdef EXPER01
    int x, y;
    MotionVector start, end;  // For arrows.
    u_int32_t  *p_32_01, *p_32_02, *p_32_03;
    if (DESHAKE_ZOOM >= 0)
         t.zoom = orig.zoom = DESHAKE_ZOOM;
    static int fuss = 0;
    static int maxfuss = 5;
    if (maxfuss && link != NULL && link->dstpad != NULL) {
//         av_log(deshake,AV_LOG_ERROR,"%s %s %d: end_frame in link is at %p\n", __FILE__,__func__,__LINE__,link->dstpad->end_frame);
         maxfuss--;
    }
#endif

    src1  = (deshake->ref == NULL) ? in->data[0] : deshake->ref->data[0];
    src2  = in->data[0];
    alpha = (DESHAKE_ALPHA > 0)? DESHAKE_ALPHA : (deshake->reference_frames)? 2.0 / deshake->reference_frames : 0.5;

    if (!fuss) {
         av_log(deshake,AV_LOG_DEBUG,"%s %d: alpha =%4.2f  reference_frames = %d  deshake->ref =%p\n", __func__,__LINE__,alpha,deshake->reference_frames, deshake->ref);
         fuss++;
    }
    if (deshake->cx < 0 || deshake->cy < 0 || deshake->cw < 0 || deshake->ch < 0) {
         // Find the most likely global motion for the current frame
         find_motion(deshake, src1, in->data[0], link->w, link->h, in->linesize[0], &t);
    } else {
        deshake->cx = FFMIN(deshake->cx, link->w);
        deshake->cy = FFMIN(deshake->cy, link->h);

        if ((unsigned)deshake->cx + (unsigned)deshake->cw > link->w) deshake->cw = link->w - deshake->cx;
        if ((unsigned)deshake->cy + (unsigned)deshake->ch > link->h) deshake->ch = link->h - deshake->cy;

        // Quadword align right margin
        deshake->cw &= ~15;

        src1 += deshake->cy * in->linesize[0] + deshake->cx;
        src2 += deshake->cy * in->linesize[0] + deshake->cx;

        find_motion(deshake, src1, src2, deshake->cw, deshake->ch, in->linesize[0], &t);
    }

    orig = t;   // Copy transform so we can output it later to compare to the smoothed value

#ifdef EXPER01
    if (OPTMASK(OPT_LOG_BLOCK_VECTORS_LOOP_FINAL)) {
         unsigned long counter=0;
         av_log(deshake,AV_LOG_ERROR,"%s %d: %8lu: %f %f %f   %f %f %f   %f %f %f   %f %f %ff\n", \
                __func__,__LINE__,++counter, orig.vector.x, deshake->avg.vector.x, t.vector.x, orig.vector.y, deshake->avg.vector.y, t.vector.y, orig.angle, deshake->avg.angle, t.angle, orig.zoom, deshake->avg.zoom, t.zoom);
    }
#endif
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
    if (OPTMASK(OPT_LOG_BLOCK_VECTORS_LOOP_FINAL)) {
         unsigned long counter=0;
         av_log(deshake,AV_LOG_ERROR,"%s %d: %8lu: %f %f %f   %f %f %f   %f %f %f   %f %f %f\n", \
                __func__,__LINE__,++counter, orig.vector.x, deshake->avg.vector.x, t.vector.x, orig.vector.y, deshake->avg.vector.y, t.vector.y, orig.angle, deshake->avg.angle, t.angle, orig.zoom, deshake->avg.zoom, t.zoom);
    }
#endif
    T_OP(t,-=,deshake->avg);  // Remove the average from the current motion to detect the motion that is not on purpose, just as jitter from bumping the camera

    t.vector.x *= -1;
    t.vector.y *= -1;
    t.angle *= -1;

    // Write statistics to file if requested.
    if (deshake->fp) {
         static unsigned long counter = 0;
         statmsg = av_asprintf("%8lu: %12.4f  %12.4f  %12.4f    %12.4f  %12.4f  %12.4f    %12.4f  %12.4f  %12.4f    %12.4f  %12.4f  %12.4f\n", \
                               ++counter, orig.vector.x, deshake->avg.vector.x, t.vector.x, orig.vector.y, deshake->avg.vector.y, t.vector.y, orig.angle, deshake->avg.angle, t.angle, orig.zoom, deshake->avg.zoom, t.zoom);
        fwrite(statmsg, sizeof(char), strlen(statmsg), deshake->fp);
#ifdef EXPER01
        fflush(deshake->fp);
#endif
        av_free(statmsg);
    }

    T_OP(t,+=,deshake->last); // Turn relative current frame motion into absolute by adding it to the last absolute motion

    // Shrink motion by 10% to keep things centered in the camera frame
    t.vector.x *= 0.9;
    t.vector.y *= 0.9;
    t.angle *= 0.9;

    deshake->last = t;  // Store the last absolute motion information

    // Generate a luma transformation matrix
    avfilter_get_matrix(t.vector.x, t.vector.y, t.angle, 1.0 + t.zoom / 100.0, matrix);

    // Transform the luma plane
#ifdef EXPER01
    avfilter_transform(src2, out->data[0], in->linesize[0], out->linesize[0], link->w, link->h, matrix, 0, INTERPOLATE_BILINEAR, deshake->edge, &deshake->extra);
#else
    avfilter_transform(src2, out->data[0], in->linesize[0], out->linesize[0], link->w, link->h, matrix, 0, INTERPOLATE_BILINEAR, deshake->edge);
#endif
    // Generate a chroma transformation matrix
    avfilter_get_matrix(t.vector.x / (link->w / CHROMA_WIDTH(link)), t.vector.y / (link->h / CHROMA_HEIGHT(link)), t.angle, 1.0 + t.zoom / 100.0, matrix);

    // Transform the chroma planes
#ifdef EXPER01
    avfilter_transform(in->data[1], out->data[1], in->linesize[1], out->linesize[1], CHROMA_WIDTH(link), CHROMA_HEIGHT(link), matrix, 127, INTERPOLATE_BILINEAR, deshake->edge, &deshake->extra);
    avfilter_transform(in->data[2], out->data[2], in->linesize[2], out->linesize[2], CHROMA_WIDTH(link), CHROMA_HEIGHT(link), matrix, 127, INTERPOLATE_BILINEAR, deshake->edge, &deshake->extra);
#else
    avfilter_transform(in->data[1], out->data[1], in->linesize[1], out->linesize[1], CHROMA_WIDTH(link), CHROMA_HEIGHT(link), matrix, 127, INTERPOLATE_BILINEAR, deshake->edge);
    avfilter_transform(in->data[2], out->data[2], in->linesize[2], out->linesize[2], CHROMA_WIDTH(link), CHROMA_HEIGHT(link), matrix, 127, INTERPOLATE_BILINEAR, deshake->edge);
#endif

#ifdef EXPER01
    {
         int arrow_color = 50;
         int highlight_color = 150;
         if (OPTMASK(OPT_BLANK_FRAME)) {
              // Put the conversion macros here for convenience; someone will want to use them someday, no doubt.
              int yp = RGB_TO_Y_CCIR(0, 0, 0);
              int u = RGB_TO_U_CCIR(0, 0, 0, 0);
              int v = RGB_TO_V_CCIR(0, 0, 0, 0);
              unsigned long counter = 0;
              for (y = 0 ; y < CHROMA_HEIGHT(link) ; y++) {
                   for (x = 0 ; x < CHROMA_WIDTH(link) ; x++) {
                        out->data[1][y * out->linesize[1] + x] = u;
                        out->data[2][y * out->linesize[2] + x] = v;
                   }
              }
              for (y = 0 ; y < link->h ; y++) {
                   for (x = 0 ; x < link->w ; x++) {
                        out->data[0][y * out->linesize[0] + x] = yp;
                   }
              }
         }
         draw_vectors(deshake, out, link->w, link->h, out->linesize[0], &t, &orig, FFMAX(16/deshake->rx,1), arrow_color, highlight_color);
    }
#endif
    // Store the current frame as the reference frame for calculating the
    // motion of the next frame
    if (deshake->ref != NULL)
        avfilter_unref_buffer(deshake->ref);

    // Cleanup the old reference frame
    deshake->ref = in;
    // Draw the transformed frame information
    avfilter_draw_slice(link->dst->outputs[0], 0, link->h, 1);
    avfilter_end_frame(link->dst->outputs[0]);
    avfilter_unref_buffer(out);
}

static void draw_slice(AVFilterLink *link, int y, int h, int slice_dir)
{
}

static void draw_vectors(DeshakeContext *deshake, AVFilterBufferRef *avbuf, int w, int h, int stride, Transform *t, Transform *orig, int normalizing_scale, int color, int highlight_color)
{
     int i, j, final_vector_params[][2] = {
          { deshake->avg.vector.x,  deshake->avg.vector.y  },
          { orig->vector.x,         orig->vector.y         },
          { t->vector.x,            t->vector.y            }
     };

     if (OPTMASK(OPT_BLOCK_VECTORS)) {
          if (DESHAKE_WINNING_COUNT) { // Do this first.
               static int fuss = 8;
               if (fuss) {
                    // av_log(deshake,AV_LOG_ERROR,"%s %d: winning count: %d  winning x: %d  winning y: %d\n", __func__, __LINE__, DESHAKE_WINNING_COUNT, DESHAKE_WINNING_MV.x, DESHAKE_WINNING_MV.y);
                    fuss--;
               }
               exper01_draw_arrow(avbuf, DESHAKE_WINNING_MV.x, DESHAKE_WINNING_MV.y, 0, 0, w, h, stride, color); // Not highlight color
          }
          draw_vectors_r(deshake, arrow_root, avbuf, w, h, stride, normalizing_scale, color, highlight_color);
     }
     arrow_root = NULL;
     if (OPTMASK(OPT_FINAL_VECTOR_ORIG_TO_FINAL) || OPTMASK(OPT_FINAL_VECTOR_AVG_TO_FINAL)) {
          static int fuss = 10;
          if (fuss) {
               static unsigned long frame = 0;
               if (OPTMASK(OPT_FLOAT_03)) {
                    av_log(deshake,AV_LOG_ERROR,"%s %d: %lu: orig x=%d, orig y=%d, avg x=%d, avg y=%d, final x=%d, final y=%d\n",
                           __func__,__LINE__,frame,
                           final_vector_params[1][0], final_vector_params[1][1], final_vector_params[0][0], final_vector_params[0][1],
                           final_vector_params[2][0], final_vector_params[2][1]);
               }
               fuss--;
               frame++;
          }
          exper01_draw_line(avbuf, w/2 -10, h/2,     w/2 + 10, h/2,      w, h, stride, color); // Center cross
          exper01_draw_line(avbuf, w/2,     h/2 -10, w/2,      h/2 + 10, w, h, stride, color);
          // Normalizing
          for (i = 0 ; i < 3 ; i++) {
               for (j = 0 ; j < 2 ; j++) {
                    if (OPTMASK(OPT_FINAL_VECTORS_NORMALIZE)) {
                         // av_log(deshake,AV_LOG_ERROR,"%s %d: normalizing factor = %d\n", __func__,__LINE__,normalizing_scale);
                         final_vector_params[i][j] *= normalizing_scale;
                    }
                    final_vector_params[i][j] += (j) ? h/2 : w/2 ;
               }
          }
          if (OPTMASK(OPT_FINAL_VECTOR_ORIG_TO_FINAL)) {
               exper01_draw_arrow(avbuf, final_vector_params[2][0], final_vector_params[2][1], final_vector_params[1][0], final_vector_params[1][1], w, h, stride, highlight_color);
          }
          if (OPTMASK(OPT_FINAL_VECTOR_AVG_TO_FINAL)) {
               exper01_draw_arrow(avbuf, final_vector_params[2][0], final_vector_params[2][1], final_vector_params[0][0], final_vector_params[0][1], w, h, stride, highlight_color);
          }
     }
}

static void draw_vectors_r(DeshakeContext *deshake, const ArrowAnnotation *root, AVFilterBufferRef *avbuf, int w, int h, int stride, int normalizing_scale, int color, int highlight_color)
{
     if (root)
     {
          if (DESHAKE_WINNING_MV.x == root->startx && DESHAKE_WINNING_MV.y == root->starty) {
               exper01_draw_arrow(avbuf, /* root->startx, root->starty,*/ 0, 0, root->startx, root->starty, w, h, stride, color);
          }
          exper01_draw_arrow(avbuf, /* root->startx, root->starty,*/ root->endx, root->endy, root->startx, root->starty, w, h, stride, color);
          draw_vectors_r(deshake, root->next, avbuf, w, h, stride, normalizing_scale, color, highlight_color);
          av_free(root);
     }
}

static av_cold int init(AVFilterContext *ctx, const char *args, void *opaque)
{
    DeshakeContext *deshake = ctx->priv;
    char filename[256] = {0};
    char *statmsg;
#ifdef EXPER01
    u_int32_t *p_32_01, *p_32_02, *p_32_03;
    int i, nopts = 0;
    OptmaskSelection *osp;
#endif
    deshake->rx = RX_DEFAULT;
    deshake->ry = RY_DEFAULT;
    deshake->edge = FILL_MIRROR;
    deshake->blocksize = BLOCKSIZE_DEFAULT;
    deshake->contrast = CONTRAST_DEFAULT;
    deshake->search = SEARCH_DEFAULT;
    deshake->reference_frames = 20;

    deshake->cw = -1;
    deshake->ch = -1;
    deshake->cx = -1;
    deshake->cy = -1;
#ifdef EXPER01
    memset(&deshake->extra,0,sizeof(deshake->extra));
    deshake->extra.alpha = ALPHA_DEFAULT;
#endif

    if (args) {
#ifdef EXPER01
        sscanf(args, "%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%Li:%f:%255s",
               &deshake->cx, &deshake->cy, &deshake->cw, &deshake->ch,
               &deshake->rx, &deshake->ry, (int *)&deshake->edge,
               &deshake->blocksize, &deshake->contrast, (int *)&deshake->search, &DESHAKE_ZOOM, (long long int*)&deshake->extra.optmask, &DESHAKE_ALPHA, filename);
        if (DESHAKE_ALPHA >= 0) {
             DESHAKE_ALPHA = av_clipf(DESHAKE_ALPHA,0.01,0.99);
        }
        global_option_01 = (OPTMASK(OPT_GLOBAL_01));
        global_option_02 = (OPTMASK(OPT_GLOBAL_02));
#else
        sscanf(args, "%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%255s",
               &deshake->cx, &deshake->cy, &deshake->cw, &deshake->ch,
               &deshake->rx, &deshake->ry, (int *)&deshake->edge,
               &deshake->blocksize, &deshake->contrast, (int *)&deshake->search, filename);
#endif

        deshake->blocksize = av_clip(deshake->blocksize, BLOCKSIZE_MIN, BLOCKSIZE_MAX);
        deshake->blocksize /= 2;
        deshake->rx = av_clip(deshake->rx, RX_MIN, RX_MAX);
        deshake->ry = av_clip(deshake->ry, RY_MIN, RY_MAX);
        deshake->edge = av_clip(deshake->edge, FILL_BLANK, FILL_COUNT - 1);
        deshake->contrast = av_clip(deshake->contrast, CONTRAST_MIN, CONTRAST_MAX);
        deshake->search = av_clip(deshake->search, EXHAUSTIVE, SEARCH_COUNT - 1);
        T_SET(deshake->avg, =, 0);

    }
    if (*filename) {
         if ((deshake->fp = fopen(filename, "w")) != NULL) {
              statmsg = av_asprintf("          %12s  %12s  %12s    %12s  %12s  %12s    %12s  %12s  %12s    %12s  %12s  %12s\n\n",
                                    "Orig x", "Avg x", "Final x", "Orig y", "Avg y", "Final y", "Orig angle", "Avg angle", "Final angle", "Orig zoom", "Avg zoom", "Final zoom");
              fwrite(statmsg, sizeof(char), strlen(statmsg), deshake->fp);
#ifdef EXPER01
              fflush(deshake->fp);
#endif
              av_free(statmsg);
         } else {
              av_log(deshake,AV_LOG_ERROR,"Failed to open stat file %s: %s\n", filename, strerror(errno));
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
    av_log(ctx, AV_LOG_INFO, "cx: %d, cy: %d, cw: %d, ch: %d, rx: %d, ry: %d, edge: %d blocksize: %d contrast: %d search: %d  zoom: %d  option mask: 0x%08lx %08lx  alpha: %f%s%s\n",
           deshake->cx, deshake->cy, deshake->cw, deshake->ch,
           deshake->rx, deshake->ry, deshake->edge, deshake->blocksize * 2, deshake->contrast, deshake->search, DESHAKE_ZOOM,
           (unsigned long)p_32_01[1], (unsigned long)p_32_01[0], DESHAKE_ALPHA, (*filename? "  log file: " : " oh "), (*filename? filename : " no "));
    if (deshake->extra.optmask) {
         av_log(NULL,AV_LOG_VERBOSE,"Enabled deshake options: ");
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
           deshake->rx, deshake->ry, deshake->edge, deshake->blocksize * 2, deshake->contrast, deshake->search, (*filename? filename : ""));
#endif

    return 0;
}

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

static int config_props(AVFilterLink *link)
{
    DeshakeContext *deshake = link->dst->priv;

    deshake->ref = NULL;
    deshake->last.vector.x = 0;
    deshake->last.vector.y = 0;
    deshake->last.angle = 0;
    deshake->last.zoom = 0;

    deshake->avctx = avcodec_alloc_context3(NULL);
    dsputil_init(&deshake->c, deshake->avctx);

    return 0;
}

static av_cold void uninit(AVFilterContext *ctx)
{
    DeshakeContext *deshake = ctx->priv;

    avfilter_unref_buffer(deshake->ref);
    if (deshake->fp)
        fclose(deshake->fp);
}


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
