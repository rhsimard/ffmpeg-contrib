/** @file
 *  Extra stuff for my development work.
 */

#include "avfilter.h"
#include "libavutil/avstring.h"
#include "libavutil/common.h"
#include "libavutil/mem.h"
#include "libavutil/pixdesc.h"
#include "libavcodec/dsputil.h"
#include "libavutil/colorspace.h"
#include "libavutil/opt.h"

////////// HELLO

#include "transform.h"
#include "libavutil/exper01.h"     // EXPER01
#include "libavfilter/vf_deshake.h"

/** @name Special development-support functions
 *
 * Generally intended for testing and development, but some may also find it useful, or at least entertaining.
 * @{*/

ArrowAnnotation *arrow_root = NULL;
static void draw_vectors_r(DeshakeContext *deshake, const ArrowAnnotation *root, AVFilterBufferRef *avbuf, int w, int h, int stride, int normalizing_scale, int color, int highlight_color);


/** Generate the vector list for later display
 *
 */
void find_motion_generate_block_vectors(DeshakeContext *deshake, int x, int y, int (*counts)[BLOCKSIZE_MAX], IntMotionVector *mv)
{
#define VECTOR_MIN (1)
     int arrow_index;
     ArrowAnnotation *arrow, **a2;
     if (OPTMASK(OPT_BLOCK_VECTORS)) {
          static int fuss=10;
          if (mv->x >= VECTOR_MIN || mv->y >= VECTOR_MIN){
               if ((arrow = av_malloc(sizeof(ArrowAnnotation)))) {
                    float fx = 1.0, fy = 1.0;
                    arrow->startx = x + deshake->blocksize/2;
                    arrow->starty = y + deshake->blocksize/2;
                    if (OPTMASK(OPT_BLOCK_VECTORS_NORMALIZE)) {
                         arrow->endx = arrow->startx + (int)((float)mv->x * 8.0/deshake->rx);
                         arrow->endy = arrow->starty + (int)((float)mv->y * 8.0/deshake->ry);
                         fx = (float)arrow->endx/(x + (float)mv->x + deshake->rx);
                         fy = (float)arrow->endy/(y + (float)mv->y + deshake->ry);
                    } else {
                         arrow->endx = arrow->startx + mv->x;
                         arrow->endy = arrow->starty + mv->y;
                    }
                    if (fuss && (mv->x || mv->y)) {
//                                   av_log(deshake,AV_LOG_ERROR,"%s %d: fx=%f, fy=%f, x=%d, mv->x=%d, deshake->rx=%d, startx=%d,  endx=%d, y=%d, mv->y=%d, deshake->ry=%d, starty=%d, endy=%d\n",
//                                          __func__,__LINE__, fx, fy, x, mv->x, deshake->rx, arrow->startx, arrow->endx, y, mv->y, deshake->ry, arrow->starty, arrow->endy);
                         fuss--;
                    }
                    arrow->count = counts[mv->x + deshake->rx][mv->y + deshake->ry];
                    arrow->highlight = 0;
                    arrow->next = NULL;
                    arrow->annotation = av_asprintf("%d %d %d %d counts=%d", x, y, mv->x, mv->y, counts[mv->x + deshake->rx][mv->y + deshake->ry]);
                    arrow_index=0;
                    for (a2 = &arrow_root ; *a2  ; a2 = &(*a2)->next, arrow_index++) {
                         if (OPTMASK(OPT_LOG_BLOCK_VECTORS_INNER_LOOP)) {
//                                        av_log(deshake,AV_LOG_ERROR,"%s %d: index=%d a2 = %p  *a2 = %p  (*a2)->next = %p  arrow_root is at %p  arrow_root = %p  x = %4d  y = %4d  mv->x = %4d  mv->y = %4d  ...rx = %4d  ...ry = %4d\n",
                              av_log(deshake,AV_LOG_ERROR,"%s %d: index=%2d  x=%3d  y=%3d  mv->x=%2d  mv->y=%2d  rx=%3d  ry=%3d\n",
                                     __func__, __LINE__, ((*a2)? (*a2)->index : -1), /* a2, *a2, ((*a2)? (*a2)->next : NULL),  &arrow_root, arrow_root, */ x, y, mv->x, mv->y, deshake->rx, deshake->ry);
                         }
                    }
                    if (OPTMASK(OPT_LOG_BLOCK_VECTORS_LOOP_FINAL)) {
                         av_log(deshake,AV_LOG_ERROR,"%s %d: arw.index=%2d start=%3d,%3d end=%3d,%3d count=%3d x=%3d y=%3d mv->x=%3d mv->y=%3d rx=%3d ry=%3d) annotation=\"%s\"\n",
                                __func__,__LINE__,arrow_index, arrow->startx, arrow->starty, arrow->endx, arrow->endy, arrow->count, x, y, mv->x, mv->y, deshake->rx, deshake->ry,  arrow->annotation);
                    }
                    arrow->index = arrow_index;
                    (*a2) = arrow;
               } else {
                    av_log(deshake,AV_LOG_ERROR,"%s %d: arrow annotation alloc failure.\n", __func__, __LINE__);
               }
          }
     }
}

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
void draw_vectors(DeshakeContext *deshake, AVFilterBufferRef *avbuf, int w, int h, int stride, Transform *t, Transform *orig, int normalizing_scale, int color, int highlight_color)
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
