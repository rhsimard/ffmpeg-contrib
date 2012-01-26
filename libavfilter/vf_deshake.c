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

#include "transform.h"
#include "libavutil/exper01.h"     // EXPER01

#ifdef EXPER01
#include <time.h>
static unsigned long fcount = 0;
static unsigned long icount = 0;
#endif

/** @name rxry
 *  Limits and default of user option for the maximum extent of movement in x and y directions
 *  @{*/
#define RX_DEFAULT         (16)
#define RX_MAX             (64)
#define RX_MIN             (0)
#define RY_DEFAULT         (16)
#define RY_MAX             (64)
#define RY_MIN             (0)
/** @}*/
/** default of user option for the method to fill image areas vacated by tranformation */
#define FILL_DEFAULT       (FILL_MIRROR)
/** @name blocksize
 *  Limits and default of user option for motion-search blocksize.
 *  @note Existing code sets this at 8, but the man page says 4.
 *  @{*/
#define BLOCKSIZE_DEFAULT  (8)
#define BLOCKSIZE_MAX      (128)
#define BLOCKSIZE_MIN      (8)
/** @}*/
/** @name contrast
 * Limits and default of user option for the minimum contrast a block must have to be considered for motion estimation.
 *  @{*/
#define CONTRAST_DEFAULT   (125)
#define CONTRAST_MAX       (255)
#define CONTRAST_MIN       (1)
/** @}*/
/** Default search type; see man page for details. */
#define SEARCH_DEFAULT     (EXHAUSTIVE)
/** @name alpha
 *  Limits and default of user option for alpha value for exponential average.
 *  A negative value leaves existing default based on the number of reference frames. (New, for test, may not stay.)
 *  @{*/
#define ALPHA_DEFAULT      (-1.0)
#define ALPHA_MAX          (0.99)
#define ALPHA_MIN          (.001)
/** @}*/
/** @name Interpolation methods
 * @{ */
#define INTERPOLATE_METHOD_CHROMA_DEFAULT (INTERPOLATE_BILINEAR)
#define INTERPOLATE_METHOD_LUMA_DEFAULT (INTERPOLATE_BILINEAR)
/** @} */
/** @name Macros to obtain the dimensions of the chroma planes
    @{*/
#define CHROMA_HEIGHT(link) -((-link->h) >> av_pix_fmt_descriptors[link->format].log2_chroma_h)
#define CHROMA_WIDTH(link)  -((-link->w) >> av_pix_fmt_descriptors[link->format].log2_chroma_w)
/** @}*/

/** Options for motion searches. */
enum SearchMethod {
    EXHAUSTIVE,        ///< Search all possible positions
    SMART_EXHAUSTIVE,  ///< Search most possible positions (faster)
    SEARCH_COUNT
};

#if 0  // Moved to exper01.h for now.
typedef struct {
    int x;             ///< Horizontal shift
    int y;             ///< Vertical shift
} IntMotionVector;
#endif

/** Description of a particular displacement.
 * @note There is an identical struct but with
 * integer members, IntMotionVector, normally here but
 * currently temporarily relocated to
 * exper01.h for test and development work.
 */
typedef struct {
    double x;             ///< Horizontal shift
    double y;             ///< Vertical shift
} MotionVector;

/** Description of a transform.
 *
 * Describes a transform performed, or which could be performed, by the deshake
 * filter on a frame.
 * @todo Research needed into the workings of the zoom value.
 */
typedef struct {
    MotionVector vector;  /** X and Y shifts       */
    double angle;         /** Rotation             */
    double zoom;          /** Zoom percentage      */
} Transform;

/** @name Transform struct macros.
 * These condense operations (other than assignment) with Transform strucures.  Usage is
 * designed to mimic the syntax of ordinary operations, for example,
 * T_OP( foo, +=, bar) is foo.(each member) += bar.(corresponding member)
 * @{*/
/** Operations between two Transform structs */
#define T_OP(dest,op,src)                       \
    do {                                        \
        dest.vector.x op src.vector.x;          \
        dest.vector.y op src.vector.y;          \
        dest.angle    op src.angle;             \
        dest.zoom     op src.zoom;              \
    } while(0)

/** Scalars */
#define T_SET(dest,op,val)                      \
    do {                                        \
        dest.vector.x op val;                   \
        dest.vector.y op val;                   \
        dest.angle    op val;                   \
        dest.zoom     op val;                   \
    } while(0)
/** @}*/

/** Description of an instance of the deshake filter.
 * See description above in defines for explanantions for
 * most of the items here. */
typedef struct {
    AVClass av_class;
    AVFilterBufferRef *ref;    ///< Previous frame
    AVCodecContext *avctx;
    DSPContext c;              ///< Context providing optimized SAD methods
    int rx;                    ///< Maximum horizontal shift
    int ry;                    ///< Maximum vertical shift
    enum FillMethod edge;      ///< Edge fill method
    int blocksize;             ///< Size of blocks to compare
    int contrast;              ///< Contrast threshold
    enum SearchMethod search;  ///< Motion search method
/** Holds transforms for the exponential average and the last ...*/
    Transform avg, last;       ///< Transforms: running average and last frame
/** Number of reference frames used for the exponential average */
    int reference_frames;              ///< Number of reference frames (defines averaging window)
/** FILE pointer; non-null if user has requested the file in the options. */
    FILE *fp;
/** User-settable oundaries of frame area within which to search for motion. */
    int cw;                    ///< Crop motion search to this box
    int ch;
    int cx;
    int cy;
#ifdef EXPER01
    enum InterpolateMethod interpolate_method_luma, interpolate_method_chroma;
/** Extra stuff for development. */
    DeshakeContextExtra  extra;
#endif
}DeshakeContext;

static double block_angle(int x, int y, int cx, int cy, IntMotionVector *shift);
static double clean_mean(double *values, int count);

#ifdef EXPER01
static int block_contrast(uint8_t *src, int x, int y, int stride, int blocksize, DeshakeContext *deshake);
static void draw_vectors(DeshakeContext *deshake, AVFilterBufferRef *avbuf, int w, int h, int stride, Transform *t, Transform *orig, int normalizing_scale, int color, int highlight_color);
static void draw_vectors_r(DeshakeContext *deshake, const ArrowAnnotation *root, AVFilterBufferRef *avbuf, int w, int h, int stride, int normalizing_scale, int color, int highlight_color);
/** Root of linked list of vector arrows to draw.
 * @see draw_vectors() */
ArrowAnnotation *arrow_root = NULL;
#else
static int block_contrast(uint8_t *src, int x, int y, int stride, int blocksize, DeshakeContext);
#endif

static int cmp(const double *a, const double *b)
{
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
 * Calculate the contrast for a given block.
 *
 * When searching for global motion we
 * really only care about the high contrast blocks, so using this method we
 * can actually skip blocks we don't care much about.
 * @param src Video data to process (luma)
 * @param x, y Location of the block
 * @param stride Distance within the data from one line to the next]
 * @param blocksize Size of the block
 * @param deshake Description of this instance of the filter
 * @return Calculated contrast value
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
 * <<<<<<<< DO THIS
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
 * @param src1,src2 data for the frames frame (luma plane)
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
#ifdef EXPER01
                    if (OPTMASK(OPT_LOG_FIND_BLOCK_MOTION_EXHAUSTIVE_LOOP)) {
                        av_log(deshake,AV_LOG_ERROR,"%s %d: x=%4d y=%4d diff=%5d smallest=%5d\n", __func__,__LINE__,x, y, diff, smallest);
                    }
#endif
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
#ifdef EXPER01
                    if (OPTMASK(OPT_LOG_FIND_BLOCK_MOTION_SMART_EXHAUSTIVE_LOOP)) {
                        av_log(deshake,AV_LOG_ERROR,"%s %d: x=%4d y=%4d diff=%5d smallest=%5d\n", __func__,__LINE__,x, y, diff, smallest);
                    }
#endif
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
#ifdef EXPER01
                    if (OPTMASK(OPT_LOG_FIND_BLOCK_MOTION_SMART_EXHAUSTIVE_LOOP)) {
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
    ADDTIME("exit","\n");
#endif
}

/**
 * Find estimated global motion for a scene.
 *
 * Find the estimated global motion given the most likely shift
 * for each block in the frame. The global motion is estimated to be the
 * same as the motion from most blocks in the frame, so if most blocks
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
    int x, y, n;
    IntMotionVector mv = {0, 0};
    int counts[BLOCKSIZE_MAX][BLOCKSIZE_MAX] = {{0}};
    int count_max_value = 0;
    int contrast;
#ifdef EXPER01
    int arrow_index;
    double pre_clip_x, pre_clip_y;
    ArrowAnnotation *arrow, **a2;
#endif

    int pos;
    double *angles = av_malloc(sizeof(*angles) * width * height / (16 * deshake->blocksize));
    int center_x = 0, center_y = 0;
    double p_x, p_y;

#ifdef EXPER01
    ADDTIME("entry","\n");
    DESHAKE_WINNING_COUNT = -1;
#endif

    pos = 0;
    // Find motion for every block and store the motion vector in the counts
    for (y = deshake->ry; y < height - deshake->ry - (deshake->blocksize * 2); y += deshake->blocksize * 2) {
        // We use a width of 16 here to match the libavcodec SAD functions
        for (x = deshake->rx; x < width - deshake->rx - 16; x += 16) {
            // If the contrast is too low, just skip this block as it probably won't be very useful to us.
#ifdef EXPER01
            contrast = block_contrast(src2, x, y, stride, deshake->blocksize, deshake);
#else
            contrast = block_contrast(src2, x, y, stride, deshake->blocksize);
#endif
            if (contrast > deshake->contrast) {
#ifdef EXPER01
                if (OPTMASK(OPT_LOG_CALL_FIND_BLOCK_MOTION)) {
                    av_log(deshake,AV_LOG_ERROR,"%s %d: (fcount =%3lu) calling find_block_motion:  x=%3d   y=%3d   stride=%3d   (contrast=%3d, src1 = %p, src2 = %p)\n",
                           __func__,__LINE__, fcount, x, y, stride, contrast, src1, src2);
                }
                ADDTIME("find_block_motion call","x = %d  y = %d\n", x, y);
#endif
                find_block_motion(deshake, src1, src2, x, y, stride, &mv);
                if (mv.x != -1 && mv.y != -1) {
#ifdef EXPER01
                    if (OPTMASK(OPT_LOG_CALL_FIND_BLOCK_MOTION)) {
                        av_log(NULL,AV_LOG_ERROR,"mv returns %d, %d\n", mv.x, mv.y);
                    }
#endif
                    if ((n = ++counts[mv.x + deshake->rx][mv.y + deshake->ry]) > count_max_value) {
                        count_max_value = n;
                        t->vector.x = mv.x;
                        t->vector.y = mv.y;
                        DESHAKE_WINNING_COUNT = count_max_value;
                        DESHAKE_WINNING_MV.x = mv.x;
                        DESHAKE_WINNING_MV.y = mv.y;
                    }
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
                                    av_log(deshake,AV_LOG_ERROR,"%s %d: index=%2d  x=%3d  y=%3d  mv.x=%2d  mv.y=%2d  rx=%3d  ry=%3d\n",
                                           __func__, __LINE__, ((*a2)? (*a2)->index : -1), /* a2, *a2, ((*a2)? (*a2)->next : NULL),  &arrow_root, arrow_root, */ x, y, mv.x, mv.y, deshake->rx, deshake->ry);
                                }
                            }
                            if (OPTMASK(OPT_LOG_BLOCK_VECTORS_LOOP_FINAL)) {
                                av_log(deshake,AV_LOG_ERROR,"%s %d: arw.index=%2d start=%3d,%3d end=%3d,%3d count=%3d x=%3d y=%3d mv.x=%3d mv.y=%3d rx=%3d ry=%3d) annotation=\"%s\"\n",
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
#ifdef EXPER01
    if (OPTMASK(OPT_LOG_FIND_MOTION_FINAL)) {
        av_log(deshake,AV_LOG_ERROR,"%s %d: (Winner: count =%5d: x =%4d y =%4d) t->vector: { %8.3f %8.3f %8.3f } (pre-clip x and y: %8.3f %8.3f)  pos =%4d   center_x =%4d   center_y =%4d  px =%8.3f  py =%8.3f\n",
               __func__,__LINE__,DESHAKE_WINNING_COUNT, DESHAKE_WINNING_MV.x, DESHAKE_WINNING_MV.y, t->vector.x,t->vector.y,t->angle, pre_clip_x, pre_clip_y,  pos, center_x, center_y, p_x, p_y);
    }
    ADDTIME("exit","\n");
#endif
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
    static int fuss = 0;
    static int maxfuss = 5;
    if (DESHAKE_ZOOM >= 0)
        t.zoom = orig.zoom = DESHAKE_ZOOM;
    if (maxfuss && link != NULL && link->dstpad != NULL) {
//         av_log(deshake,AV_LOG_ERROR,"%s %s %d: end_frame in link is at %p\n", __FILE__,__func__,__LINE__,link->dstpad->end_frame);
        maxfuss--;
    }
#endif

    ADDTIME("entry","\n");
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
    ADDTIME("find motion done","\n");
    orig = t;   // Copy transform so we can output it later to compare to the smoothed value

#ifdef EXPER01
    if (OPTMASK(OPT_LOG_POST_FIND_MOTION_01)) {
        av_log(deshake,AV_LOG_ERROR,"%s %d: %8lu: %f %f %f   %f %f %f   %f %f %f   %f %f %f\n", \
               __func__,__LINE__, fcount, orig.vector.x, deshake->avg.vector.x, t.vector.x, orig.vector.y, deshake->avg.vector.y, t.vector.y, orig.angle, deshake->avg.angle, t.angle, orig.zoom, deshake->avg.zoom, t.zoom);
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
    if (OPTMASK(OPT_LOG_POST_FIND_MOTION_01)) {
        av_log(deshake,AV_LOG_ERROR,"%s %d: %8lu: %f %f %f   %f %f %f   %f %f %f   %f %f %f\n", \
               __func__,__LINE__, fcount, orig.vector.x, deshake->avg.vector.x, t.vector.x, orig.vector.y, deshake->avg.vector.y, t.vector.y, orig.angle, deshake->avg.angle, t.angle, orig.zoom, deshake->avg.zoom, t.zoom);
    }
#endif
    T_OP(t,-=,deshake->avg);  // Remove the average from the current motion to detect the motion that is not on purpose, just as jitter from bumping the camera

    t.vector.x *= -1;
    t.vector.y *= -1;
    t.angle *= -1;

    // Write statistics to file if requested.
    if (deshake->fp) {
        statmsg = av_asprintf("%8lu: %6.2f  %6.2f  %6.2f    %6.2f  %6.2f  %6.2f    %6.2f  %6.2f  %6.2f    %6.2f  %6.2f  %6.2f\n", \
                              fcount, orig.vector.x, orig.vector.y, deshake->avg.vector.x, deshake->avg.vector.y, t.vector.x, t.vector.y, orig.angle, deshake->avg.angle, t.angle, orig.zoom, deshake->avg.zoom, t.zoom);
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
    ADDTIME("before luma transform","\n");
    avfilter_transform(src2, out->data[0], in->linesize[0], out->linesize[0], link->w, link->h, matrix, 0, deshake->interpolate_method_luma, deshake->edge, &deshake->extra);
#else
    avfilter_transform(src2, out->data[0], in->linesize[0], out->linesize[0], link->w, link->h, matrix, 0, deshake->interpolate_method_luma, deshake->edge);
#endif
    // Generate a chroma transformation matrix
    avfilter_get_matrix(t.vector.x / (link->w / CHROMA_WIDTH(link)), t.vector.y / (link->h / CHROMA_HEIGHT(link)), t.angle, 1.0 + t.zoom / 100.0, matrix);

    // Transform the chroma planes
#ifdef EXPER01
    ADDTIME("before chroma transform","1");
    avfilter_transform(in->data[1], out->data[1], in->linesize[1], out->linesize[1], CHROMA_WIDTH(link), CHROMA_HEIGHT(link), matrix, 127, deshake->interpolate_method_chroma, deshake->edge, &deshake->extra);
    ADDTIME("before chroma transform","2");
    avfilter_transform(in->data[2], out->data[2], in->linesize[2], out->linesize[2], CHROMA_WIDTH(link), CHROMA_HEIGHT(link), matrix, 127, deshake->interpolate_method_chroma, deshake->edge, &deshake->extra);
#else
    avfilter_transform(in->data[1], out->data[1], in->linesize[1], out->linesize[1], CHROMA_WIDTH(link), CHROMA_HEIGHT(link), matrix, 127, deshake->interpolate_method_chroma, deshake->edge);
    avfilter_transform(in->data[2], out->data[2], in->linesize[2], out->linesize[2], CHROMA_WIDTH(link), CHROMA_HEIGHT(link), matrix, 127, deshake->interpolate_method_chroma, deshake->edge);
#endif

#ifdef EXPER01
    {
        int arrow_color = 224;
        int highlight_color = 255;
        if (OPTMASK(OPT_BLANK_FRAME)) {
            // Put the conversion macros here for convenience; someone will want to use them someday, no doubt.
            int yp = RGB_TO_Y_CCIR(0, 0, 0);
            int u = RGB_TO_U_CCIR(0, 0, 0, 0);
            int v = RGB_TO_V_CCIR(0, 0, 0, 0);
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
            arrow_color = highlight_color = 255;
        }
        draw_vectors(deshake, out, link->w, link->h, out->linesize[0], &t, &orig, FFMAX(16/deshake->rx,1), arrow_color, highlight_color);
    }
#endif
    ADDTIME("final processing","\n");

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

/** @name Special development-support functions
 *
 * Generally intended for testing and development, but some may also find it useful, or at least entertaining.
 * @{*/

/** Draw (optionally) per-block and final vectors; Part of test/development code.
 *
 * Draw vectors over the output video to indicate what movement has been detected by the
 * motion-search functions, and also vectors showing the results of the actual transform relative
 * to the original position and also the current value of the exponential average.
 *
 * These operations are individually controllable by the command-line option-mask parameter.
 *
 * The individual block vectors are determined during the scanning and analysis in find_block_motion()
 * and stored in a linked list that can thus be drawn over the transformed video later.
 * @note There are test options available to blank the frame or do a null transform (preserving all functions except the actual video data transfer).
 * This allows creating video of only the vectors, and also of the vectors overlaying the original video from which they were calculated, rather than
 * the transformed video.
 * @see find_block_motion()
 * @param deshake Description of this instance of the filter
 * @param avbuf Reference to the buffer containing the video data
 * @param w, h  Width and height of the frame
 * @param stride Distance within the video data from one line to the next
 * @param t Description of the affine tranform that has been performed on the data
 * @param orig Tranform parameters before the transform is performed
 * @param normalizing_scale Scale factor to apply when user has requested normalization
 * @param color Value to add to luma data pixels when drawing the vector
 * @param highlight_color An alternative used to set a particular item apart from the others.
 * @note When blanking the frame is enabled, both regular and hightlight colors are set to max (255).
 */
static void draw_vectors(DeshakeContext *deshake, AVFilterBufferRef *avbuf, int w, int h, int stride, Transform *t, Transform *orig, int normalizing_scale, int color, int highlight_color)
{
    int i, j, final_vector_params[][2] = {
        { deshake->avg.vector.x,  deshake->avg.vector.y  },
        { orig->vector.x,         orig->vector.y         },
        { t->vector.x,            t->vector.y            }
    };

/** convenience macros for accessing final_vector_params[][] */
#define AVG_X (final_vector_params[0][0])
#define AVG_Y (final_vector_params[0][1])
#define ORIG_X (final_vector_params[1][0])
#define ORIG_Y (final_vector_params[1][1])
#define T_X (final_vector_params[2][0])
#define T_Y (final_vector_params[2][1])


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
        if (fuss || OPTMASK(OPT_FLOAT_04)) {
            static unsigned long frame = 0;
            if (OPTMASK(OPT_FLOAT_03)) {
                av_log(deshake,AV_LOG_ERROR,"%s %4d: %6lu: orig x=%4d, orig y=%4d, avg x=%4d, avg y=%4d, final x=%4d, final y=%4d fcount =%6lu  icount =%6lu\n",
                       __func__,__LINE__,frame,
                       final_vector_params[1][0], final_vector_params[1][1], final_vector_params[0][0], final_vector_params[0][1],
                       final_vector_params[2][0], final_vector_params[2][1], fcount, icount);
            }
            fuss--;
            frame++;
        }
        exper01_draw_line(avbuf, w/2 -10, h/2,     w/2 + 10, h/2,      w, h, stride, color/4); // Center cross
        exper01_draw_line(avbuf, w/2,     h/2 -10, w/2,      h/2 + 10, w, h, stride, color/4);
        // Scaling, sign, normalizing
        for (i = 0 ; i < 3 ; i++) {
            for (j = 0 ; j < 2 ; j++) {
                final_vector_params[i][j] *= (OPTMASK(OPT_FINAL_VECTORS_NORMALIZE)) ? -normalizing_scale : -1;
                final_vector_params[i][j] += (j) ? h/2 : w/2 ;
            }
        }
        if (OPTMASK(OPT_FINAL_VECTOR_ORIG_TO_FINAL)) {
            exper01_draw_arrow(avbuf, T_X, T_Y, ORIG_X, ORIG_Y, w, h, stride, highlight_color);
        }
        if (OPTMASK(OPT_FINAL_VECTOR_AVG_TO_FINAL)) {
            exper01_draw_arrow(avbuf, T_X, T_Y, AVG_X, AVG_Y, w, h, stride, highlight_color);
        }
    }
}

/** Recursive element for vector drawing */
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
    deshake->edge = FILL_DEFAULT;
    deshake->blocksize = BLOCKSIZE_DEFAULT;
    deshake->contrast = CONTRAST_DEFAULT;
    deshake->search = SEARCH_DEFAULT;
    deshake->interpolate_method_luma = INTERPOLATE_METHOD_LUMA_DEFAULT
        deshake->interpolate_method_chroma = INTERPOLATE_METHOD_CHROMA_DEFAULT
        deshake->reference_frames = 20;

    deshake->cw = -1;
    deshake->ch = -1;
    deshake->cx = -1;
    deshake->cy = -1;
#ifdef EXPER01
    icount++;
    memset(&deshake->extra,0,sizeof(deshake->extra));
    deshake->extra.alpha = ALPHA_DEFAULT;
#endif

    if (args) {
#ifdef EXPER01
/*
  static const AVOption drawtext_options[]= {
  {"search-area" , "restricted search area"   ,  OFFSET(search_area),           AV_OPT_TYPE_STRING, {.str=NULL},  CHAR_MIN, CHAR_MAX },
  {"blocksize"   , "block size"               ,  OFFSET(blocksize),             AV_OPT_TYPE_INT,    {.dbl=8   },  8, 128             },
  {NULL}
  };
*/
        sscanf(args, "%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%d:%Li:%f:%255s",
               &deshake->cx, &deshake->cy, &deshake->cw, &deshake->ch,
               &deshake->rx, &deshake->ry, (int *)&deshake->edge,
               &deshake->blocksize, &deshake->contrast, (int *)&deshake->search, &DESHAKE_ZOOM, (long long int*)&deshake->extra.optmask, &DESHAKE_ALPHA, filename);
        if (DESHAKE_ALPHA >= 0) {
            DESHAKE_ALPHA = av_clipf(DESHAKE_ALPHA,0.01,0.99);
        }
        global_option_01 = OPTMASK(OPT_GLOBAL_01);
        global_option_02 = OPTMASK(OPT_GLOBAL_02);
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
            statmsg = av_asprintf("          %7s %7s %7s   %7s %7s %7s   %7s %7s %7s   %7s %7s %7s\n\n",
                                  "Or x", "Or y", "Av x", "Av y", "Fin x", "Fin y", "Or ang", "Av ang", "Fin ang", "Or zm", "Av zm", "Fin zm");
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
           deshake->rx, deshake->ry, deshake->edge, deshake->blocksize * 2, deshake->contrast, deshake->search, (*filename? filename : ""));
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
    deshake->avctx = avcodec_alloc_context3(NULL);
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
