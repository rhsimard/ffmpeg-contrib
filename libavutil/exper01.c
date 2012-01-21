#include "libavutil/intmath.h"
#include "libavutil/imgutils.h"
/*
  #include "avcodec.h"
  #include "dsputil.h"
  #include "internal.h"
  #include "mpegvideo.h"
  #include "mpegvideo_common.h"
  #include "mjpegenc.h"
  #include "msmpeg4.h"
  #include "faandct.h"
  #include "xvmc_internal.h"
  #include "thread.h"
*/
#include <limits.h>


#include "libavfilter/avfilter.h"
#include "libavutil/exper01.h"

OptmaskSelection optmask_selections[] =
{
     { 0x0000000000000001, "null transform"                   , "null transform"                                       },
     { 0x0000000000000002, "blank frame"                      , "blank frame"                                          },
     { 0x0000000000000004, "block vectors"                    , "draw block vectors"                                   },
     { 0x0000000000000008, "block vector normalize"           , "normalize block vectors"                              },
     { 0x0000000000000010, "final vector orig to final"       , "draw final vector from orig.vector to t.vector"       },
     { 0x0000000000000020, "final vector avg to final"        , "draw final vector from avg.vector to t.vector"        },
     { 0x0000000000000040, "final vector normalize"           , "normalize final vectors"                              },
     { 0x0000000000000080, "log block vectors inner loop"     , "log inner loop of block-vector function"              },
     { 0x0000000000000100, "log block vectors final"          , "log final stage of block-vector generation"           },
     { 0x0000000000000200, "log block motion final"           , "log find_block_motion final"                          },
     { 0x0000000000000400, "log exhaustive loop"              , "log exhaustive-check search loop"                     },
     { 0x0000000000000800, "log smart search loop"            , "log smart-exhaustive-check search loop"               },
     { 0x0000000000001000, "log exhaustive loop final"        , "log smart-exhaustive-check loop final"                },
     { 0x0010000000000000, "global option 01"                 , "enable global option 01"                              },
     { 0x0020000000000000, "global option 02"                 , "enable global option 02"                              },
     { 0x0040000000000000, "global option 03"                 , "enable global option 03"                              },
     { 0x0080000000000000, "global option 04"                 , "enable global option 04"                              },
     { 0x0100000000000000, "floating switch 01"               , "floating switch 01"                                   },
     { 0x0200000000000000, "floating switch 02"               , "floating switch 02"                                   },
     { 0x0400000000000000, "floating switch 03"               , "floating switch 03"                                   },
     { 0x0800000000000000, "floating switch 04"               , "floating switch 04"                                   },
     { 0x1000000000000000, "floating switch 05"               , "floating switch 05"                                   },
     { 0x2000000000000000, "floating switch 06"               , "floating switch 06"                                   },
     { 0x4000000000000000, "floating switch 07"               , "floating switch 07"                                   },
     { 0x8000000000000000, "floating switch 08"               , "floating switch 08"                                   }
};

static int n_optmask_selections = sizeof(optmask_selections)/sizeof(OptmaskSelection);

int global_option_01;
int global_option_02;
int global_option_03;
int global_option_04;

#if 0
/**
 * Draw a line from (ex, ey) -> (sx, sy).
 * @param avbuf reference to the buffer on which to draw
 * @param w width of the image
 * @param h height of the image
 * @param stride stride/linesize of the image
 * @param color color of the arrow
 * @todo  Change color arg to packed RGB levels; handle all pix formats.
 * @note  This is a highly efficient algorithm, using only addition, subtraction,
 * increment and comparison operations in the drawing loop.
 */
#endif

void exper01_draw_line(AVFilterBufferRef *avbuf, int sx, int sy, int ex, int ey,
                       int w, int h, int stride, int color)
{
     int *p_smin, *p_emin, *p_smax, *p_emax, incr_min, incr_max, deltamin, abs_deltamin, deltamax, tmin;
     uint8_t *buf;

     // Set up as follows:
     //  p_smax           : pointer to a variable containing the lesser coordinate of the longer of the two axes
     //  p_emax           : pointer to a variable containing the greater coordinate of the longer of the two axes
     //  p_smin, p_smin   : similar, but for the shorter axis
     //  deltamax         : 2 * distance from end to end along the longer axis; always positive
     //  deltamin         : 2 * distance from end to end along the shorter axis; sign depends on line slope
     //  incr_max         : increment within the data buffer for a 1-pixel change along the longer axis
     //  incr_min         : similar, but for the shorter axis
     //  buf              : point to the place in the data buffer where this line begins
     //  tmin             : initialized to one-half of deltamax

     sx = av_clip(sx, 0, w - 1);
     sy = av_clip(sy, 0, h - 1);
     ex = av_clip(ex, 0, w - 1);
     ey = av_clip(ey, 0, h - 1);

     if (FFABS(ey - sy) > FFABS(ex - sx)) {
          p_smax = &sy;
          p_emax = &ey;
          p_smin = &sx;
          p_emin = &ex;
          incr_min = 1;
          incr_max = stride;
     } else {
          p_smax = &sx;
          p_emax = &ex;
          p_smin = &sy;
          p_emin = &ey;
          incr_min = stride;
          incr_max = 1;
     }
     if (*p_smax > *p_emax) {
          FFSWAP(int*, p_smax, p_emax);
          FFSWAP(int*, p_smin, p_emin);
     }
     deltamax = *p_emax - *p_smax;
     deltamin = *p_emin - *p_smin;
     buf = avbuf->data[0] + (*p_smax) * incr_max + (*p_smin) * incr_min;
     tmin = deltamax;
     deltamin <<= 1;
     deltamax <<= 1;
     abs_deltamin = abs(deltamin);
     incr_min *= FFSIGN(deltamin);
     for ( ; *p_smax <= *p_emax ; (*p_smax)++) {
          *buf   += color;
          if ((tmin += abs_deltamin) >= deltamax) {
               tmin -= deltamax;
               buf += incr_min;
          }
          buf += incr_max;
     }
}

#if 0
/**
 * Draw an arrow from (ex, ey) -> (sx, sy).
 * @param avbuf reference to the buffer on which to draw
 * @param w width of the image
 * @param h height of the image
 * @param stride stride/linesize of the image
 * @param color color of the arrow
 * @note Stolen from libavcodec/mpegvideo.c
 */
#endif

void exper01_draw_arrow(AVFilterBufferRef *avbuf, int sx, int sy, int ex,
                        int ey, int w, int h, int stride, int color)
{
     int dx,dy;

#define ARROWHEAD_SIZE (5)

     sx = av_clip(sx, -100, w + 100);
     sy = av_clip(sy, -100, h + 100);
     ex = av_clip(ex, -100, w + 100);
     ey = av_clip(ey, -100, h + 100);

     dx = ex - sx;
     dy = ey - sy;

     if (dx * dx + dy * dy > ARROWHEAD_SIZE * ARROWHEAD_SIZE) {
          int rx =  dx + dy;
          int ry = -dx + dy;
          int length = ff_sqrt((rx * rx + ry * ry) << 8);

          // FIXME subpixel accuracy
          rx = ROUNDED_DIV(rx * ARROWHEAD_SIZE << 4, length);
          ry = ROUNDED_DIV(ry * ARROWHEAD_SIZE << 4, length);

          exper01_draw_line(avbuf, sx, sy, sx + rx, sy + ry, w, h, stride, color);
          exper01_draw_line(avbuf, sx, sy, sx - ry, sy + rx, w, h, stride, color);
     }
     exper01_draw_line(avbuf, sx, sy, ex, ey, w, h, stride, color);
}


#undef printf
int opt_exper01_options(const char *opt, const char *arg)
{
     int i;
     u_int32_t *p32;

     printf("Option mask:\n");
     for (i = 0 ; i < sizeof(optmask_selections)/sizeof(OptmaskSelection) ; i++) {
          OptmaskSelection *os = &optmask_selections[i];
          p32 = (u_int32_t*) &os->mask;
          printf("%08lx %08lx  %s\n", (unsigned long)p32[1], (unsigned long)p32[0], os->descr);
     }
     return 0;
}

int get_n_optmask_selections(void)
{
     return n_optmask_selections;
}

