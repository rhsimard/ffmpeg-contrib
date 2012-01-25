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
#include <sys/types.h>
#include <limits.h>


#include "libavfilter/avfilter.h"
#include "libavutil/avstring.h"
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
     { 0x0000000000000080, "log call find_block_motion"       , "log call find_block_motion, returned motion vector"   },
     { 0x0000000000000100, "log block vectors inner loop"     , "log inner loop of block-vector function"              },
     { 0x0000000000000200, "log block vectors final"          , "log final stage of block-vector generation"           },
     { 0x0000000000000400, "log find_motion final"            , "log find_motion final"                                },
     { 0x0000000000000800, "log exhaustive loop"              , "log exhaustive search loop"                           },
     { 0x0000000000001000, "log smart-exhaustive loop"        , "log smart-exhaustive search loop"                     },
     { 0x0000000000002000, "log find_block_motion final"      , "log find_block_motion final"                          },
     { 0x0000000000004000, "log post find motion 01"          , "log processing following find_motion pt. 01"          },
     { 0x0000000000008000, "log post find motion 02"          , "log processing following find_motion pt. 02"          },
     { 0x0000000000010000, "log post find motion 03"          , "log processing following find_motion pt. 03"          },
     { 0x0000000000020000, "log post find motion 04"          , "log processing following find_motion pt. 04"          },
     { 0x0000000000040000, "use time track"                   , "enable the generation of time markers"                },
     { 0x0000000000080000, "dump time track"                  , "dump the time track after processing"                 },
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
 * @todo  Shadow/background
 * @todo  Draw modes: add, invert, solid, etc.
 * @todo  Styles: dashed, dotted, solid, similar to CSS.
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
     color = av_clip(color, 0, 255);

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
          *buf   = color;
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

     int sxm = sx-1, sym = sy+1, exm=ex-1, eym=ey+1, rx, ry, length=0;

     sx = av_clip(sx, 2, w-2);
     sy = av_clip(sy, 2, h-2);
     ex = av_clip(ex, 2, w-2);
     ey = av_clip(ey, 2, h-2);
     color = av_clip(color,0,255);

     dx = ex - sx;
     dy = ey - sy;

     if (dx * dx + dy * dy > ARROWHEAD_SIZE * ARROWHEAD_SIZE) {
          rx =  dx + dy;
          ry = -dx + dy;
          length = ff_sqrt((rx * rx + ry * ry) << 8);

          // FIXME subpixel accuracy
          rx = ROUNDED_DIV(rx * ARROWHEAD_SIZE << 4, length);
          ry = ROUNDED_DIV(ry * ARROWHEAD_SIZE << 4, length);

          // Draw the arrowhead shadow.
          exper01_draw_line(avbuf, sxm, sym, sxm + rx, sym + ry, w, h, stride, 256-color);
          exper01_draw_line(avbuf, sxm, sym, sxm - ry, sym + rx, w, h, stride, 256-color);
     }
     exper01_draw_line(avbuf, sxm, sym, exm, eym, w, h, stride, 256-color);  // Draw shadow
     if (length) {
          exper01_draw_line(avbuf, sx, sy, sx + rx, sy + ry, w, h, stride, color); // Draw arrowhead.
          exper01_draw_line(avbuf, sx, sy, sx - ry, sy + rx, w, h, stride, color);
     }
     exper01_draw_line(avbuf, sx, sy, ex, ey, w, h, stride, color);          // Draw arrow
}


#undef printf
int opt_exper01_options(const char *opt, const char *arg)
{
     int i;
     u_int32_t *p32;

     printf("Options: %s\n","x:y:w:h:rx:ry:edge:blocksize:contrast:search:zoom:option-mask:alpha:filename");
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

////////////// Time markers
static TimeTrack *timetrack_root;
int use_time_track = 0;

TimeTrack *add_time_marker(const char *filename, const char* func, int line, const char* label, const char* fmt, ...)
{
     TimeTrack *retval = NULL;

     if (use_time_track) {
          va_list va;
          va_start(va,fmt);
          retval = vadd_time_marker(filename, func, line, label, fmt, va);
          va_end(va);
     }
     return(retval);
}

TimeTrack *vadd_time_marker(const char *filename, const char* func, int line, const char* label, const char* fmt, va_list va)
{
     TimeTrack *retval=NULL, *troot, *prev = NULL, **prevlink;
     int n_marker = 1;

     if (use_time_track) {
          prevlink = &timetrack_root;
          for (troot = timetrack_root ; troot ; troot=troot->next) {
               n_marker++;
               prev = troot;
               prevlink = &troot->next;
          }
          if ((retval = (TimeTrack*) av_malloc(sizeof(TimeTrack))) == NULL) {
               av_log(NULL,AV_LOG_ERROR,"Memory allocation failure in %s %s %d: %s\n", __FILE__, __func__, __LINE__, strerror(errno));
               return NULL;
          }
          *prevlink = retval;
          memset(retval,0,sizeof(TimeTrack));
          if (gettimeofday(&retval->tv,NULL) < 0) {
               av_log(NULL,AV_LOG_ERROR,"%s %s %d: gettimeofday() failed: %s\n", __FILE__, __func__, __LINE__, strerror(errno));
               return NULL;
          }
          retval->previous = prev;
          retval->marker_number = n_marker;
          retval->file = filename;
          retval->func = func;
          retval->line = line;
          retval->label = label;
          retval->descr = av_vasprintf(fmt,va);
          //av_log(NULL,AV_LOG_ERROR,"%s %d: Time record %d for %s %s %d added at %p parent = %p  time = %lu\n",
          //       __func__, __LINE__, n_marker, filename, func, line, retval, prevlink, (unsigned long)retval->tv.tv_usec);
     }
     return(retval);
}

void dump_time_marker(const TimeTrack *marker, const struct timeval *tv)
{
     struct timeval ltv = {0,0}, dtv;

     if (tv) {
          ltv = *tv;
     }
     timersub(&marker->tv, &ltv, &dtv);
     if (marker) {
          av_log(NULL,AV_LOG_INFO,"Time marker %6d  %30s:  time = %8ld   file: %s function: %s  line: %4d    %s",
                 marker->marker_number, (marker->label? marker->label : ""), dtv.tv_sec * 1000000 + dtv.tv_usec, marker->file, marker->func, marker->line, marker->descr);
     }
}

void dump_time_track(const TimeTrack *tt)
{
     const TimeTrack *tti = (tt == NULL)? timetrack_root : tt;
     struct timeval start_tv = { 0, 0 };

     if (tti) {
          start_tv = tti->tv;
          av_log(NULL,AV_LOG_INFO,"%s %d: Dumping time track:\n", __func__,__LINE__);
          for (  ; tti ; tti = tti->next) {
               dump_time_marker(tti, &start_tv);
          }
     }
}

static int delete_time_track_r(TimeTrack *root, int n);
void delete_time_track(void)
{
     delete_time_track_r(timetrack_root, 0);
     timetrack_root = NULL;
}

static int delete_time_track_r(TimeTrack *root, int n)
{
     int rn = -1;
     if (root != NULL)
     {
          rn = delete_time_track_r(root->next, n+1);
          if (root->descr)
               av_free(root->descr);
          av_free(root);
     }

     return rn;
}

