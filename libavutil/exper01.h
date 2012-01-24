#ifndef _EXPER01_H__
#define _EXPER01_H__

/**
 * @file  Sundry goodies for my experimentation
 *
 * An exciting collection of wondrous things-or something like that.
 */


#define EXPER01
#include <stdint.h>
#include <sys/types.h>
#include <time.h>
#include <sys/time.h>
#include "libavfilter/avfilter.h"

/**
 * Draw a line on the image in the specified buffer, for now, simply plane 0;
 * @param avbuf Reference to the image buffer
 * @param sx Start x
 * @param sy Start y
 * @param ex End x
 * @param ey End y
 * @param w Image width
 * @param h Image height
 * @param stride Stride
 * @param color TBDescribed
 * @note For now, simply draws on plane 0.  Later on, fancy color jazz is planned.
 */
void exper01_draw_line(AVFilterBufferRef *avbuf, int f_sx, int f_sy, int f_ex, int f_ey,
                       int f_w, int f_h, int f_stride, int f_color);

/**
 * Draw an arrow on the image in the specified buffer, for now, simply plane 0;
 * @param avbuf Reference to the image buffer
 * @param sx Start x
 * @param sy Start y
 * @param ex End x
 * @param ey End y
 * @param w Image width
 * @param h Image height
 * @param stride Stride
 * @param color TBDescribed
 * @note For now, simply draws on plane 0.  Later on, fancy color jazz is planned.
 */
void exper01_draw_arrow(AVFilterBufferRef *avbuf, int f_sx, int f_sy, int f_ex, int f_ey,
                        int f_w, int f_h, int f_stride, int f_color);

enum opt_select { OPT_NULL_TRANSFORM,
                  OPT_BLANK_FRAME,
                  OPT_BLOCK_VECTORS,
                  OPT_BLOCK_VECTORS_NORMALIZE,
                  OPT_FINAL_VECTOR_ORIG_TO_FINAL,
                  OPT_FINAL_VECTOR_AVG_TO_FINAL,
                  OPT_FINAL_VECTORS_NORMALIZE,
                  OPT_LOG_CALL_FIND_BLOCK_MOTION,
                  OPT_LOG_BLOCK_VECTORS_INNER_LOOP,
                  OPT_LOG_BLOCK_VECTORS_LOOP_FINAL,
                  OPT_LOG_FIND_MOTION_FINAL,
                  OPT_LOG_FIND_BLOCK_MOTION_EXHAUSTIVE_LOOP,
                  OPT_LOG_FIND_BLOCK_MOTION_SMART_EXHAUSTIVE_LOOP,
                  OPT_LOG_FIND_BLOCK_MOTION_FINAL,
                  OPT_LOG_POST_FIND_MOTION_01,
                  OPT_LOG_POST_FIND_MOTION_02,
                  OPT_LOG_POST_FIND_MOTION_03,
                  OPT_LOG_POST_FIND_MOTION_04,
                  OPT_USE_TIME_TRACK,
                  OPT_DUMP_TIME_TRACK,
                  OPT_GLOBAL_01,
                  OPT_GLOBAL_02,
                  OPT_GLOBAL_03,
                  OPT_GLOBAL_04,
                  OPT_FLOAT_01,
                  OPT_FLOAT_02,
                  OPT_FLOAT_03,
                  OPT_FLOAT_04,
                  OPT_FLOAT_05,
                  OPT_FLOAT_06,
                  OPT_FLOAT_07,
                  OPT_FLOAT_08
};

/**
 * Bit-switch test options
 *
 */
typedef struct
{
     /** bit mask; individual bits control various options. */
     u_int64_t  mask;
     /** short description */
     const char *shortdescr;
     /** full description */
     const char *descr;
}OptmaskSelection;

extern OptmaskSelection optmask_selections[];

#define OPTMASK(select) ((deshake->extra.optmask & optmask_selections[select].mask) != 0)
#define OPTMASK_DESCR(select) (optmask_selections[select].descr)

extern int global_option_01;
extern int global_option_02;
extern int global_option_03;
extern int global_option_04;


//#define DESHAKE_OPTMASK(x) dummy_optmask
//#define DESHAKE_ZOOM(x) dummy_zoom
#define DESHAKE_ZOOM (deshake->extra.zoom)
#define DESHAKE_WINNING_COUNT (deshake->extra.winning_count)
#define DESHAKE_WINNING_MV (deshake->extra.imvs[0])
#define DESHAKE_ALPHA (deshake->extra.alpha)

// Normally in vf_deshake.c
typedef struct {
    int x;             ///< Horizontal shift
    int y;             ///< Vertical shift
} IntMotionVector;


/** Extras tacked onto DeshakeContext for testing
 * @param zoom A user-settable zoom to override the default
 * @param optmask Mask to control individual functions throughout the code
 * @param imvs fribbble
 * @param n_valid_imvs snarfle
 * @param 
 */
typedef struct {
     int zoom;   // Manually-set test value using integer percentages.
     u_int64_t  optmask;  // Bit mask for on-off devel/debug options.
     IntMotionVector imvs[10];             // motion vector data from scans to use for final vectors
     int n_valid_imvs;                     // Number of valid motion vectors in invs
     int winning_count;                    // The count found in find_motion that is chosen for the gmv
     float  alpha;                         // User-specified alpha for exponential average, if any; overrides default.
} DeshakeContextExtra;

//extern u_int64_t dummy_optmask;
//extern int dummy_zoom;

/**
 * Descriptor for an arrow.
 *
 * Describes an arrow and related annotation, with provision for implementation
 * in a linked list.
 */
typedef struct s_arrow {
     int  index, count, startx, starty, endx, endy, highlight;  // Arrow
     char* annotation;
     struct s_arrow *next;
}ArrowAnnotation;

extern ArrowAnnotation *arrow_root;

/**
 * Print the list of available switch options.
 *
 * Called when the command-line argument is specified.
 * @param opt Option name, not used
 * @param arg Argument, not used
 * @return Unused
 */
int opt_exper01_options(const char *opt, const char *arg);

/**
 * Retreieve the number of implemented option-mask selections.
 *
 * @return Number of selections
 */
int get_n_optmask_selections(void);

/**
 * Element of linked list of time markers.
 *
 * The time at various stages of the process is measured and stored for later
 * display and analysis.
 */
typedef struct s_timetrack {
     struct timeval      tv;                   /** timestamp                               */
     const  char        *func;                 /** function name                           */
     const  char        *file;                 /** filename                                */
     int                 line;                 /** line number                             */
     const char         *label, *descr;        /** an identifying label, optional descriptive information */
     struct s_timetrack *next, *previous;      /** links                                   */
}TimeTrack;

extern int use_time_track;

/** Add a time marker to the linked list.
 *
 * Adds a record of the time and optional text
 * @param filename     Name of source file
 * @param func         Name of function
 * @param line         Line number
 * @param label        An identifying label
 * @param fmt          printf-style format string
 * @param ...          Variadic args
 * @return             Pointer to the object
 */
TimeTrack *add_time_marker(const char *filename, const char* func, int line, const char* label, const char* fmt, ...);
/** Add a time marker to the linked list (va_list version). */
TimeTrack *vadd_time_marker(const char *filename, const char* func, int line, const char* label, const char* fmt, va_list va);

/** Log the contents of a time marker.
 *
 * @param marker  Pointer to the time marker
 */
void dump_time_marker(const TimeTrack *marker);

/** Log the markers in the time track.
 *
 * @param tt  Pointer to the first marker to dump.  If null, dump the entire track.
 */
void dump_time_track(const TimeTrack *tt);

/** Delete the time track and free memory allocated to it. */
void delete_time_track(void);

#define ADDTIME(label,fmt,...) add_time_marker(__FILE__,__func__,__LINE__,label,fmt, ##__VA_ARGS__)
#endif
