#ifndef _EXPER01_H__
#define _EXPER01_H__

/**
 * @file
 *
 *  Sundry goodies for my experimentation
 *
 * An exciting collection of wondrous things-or something like that.
 */


#define EXPER01
#define USE_AVOPTION
#include <stdint.h>
#include <sys/types.h>
#include <time.h>
#include <sys/time.h>
#include "libavfilter/avfilter.h"
#include "libavutil/avassert.h"

/** @name Macros for test hacks.
 * These are intended to avoid the plethora of ifdef/endifs
 * for test alterations, to keep test logging, including logging
 * controlled by the option mask, on a single line, and
 * to produce the original code when EXPER01 is not defined.
 * @{*/
#ifdef EXPER01
# define LOG_IF(cond,ctx,level,fmt,...)            \
    do { if (cond) {                               \
        av_log(ctx, level, fmt, ##__VA_ARGS__);      \
        }} while(0)                                    ///< Produce the log message if the condtion is met and EXPER01 is defined.
#  define LOG_IF_OPTMASK(opt,ctx,level,fmt,...) LOG_IF(OPTMASK(opt), ctx, level, fmt, ##__VA_ARGS__)  ///< Shortcut for optmask cases.
#  define CALL_ADD_DESHAKE(func,...) func(deshake, ##__VA_ARGS__)                 ///< Adds the deshake parameter to function calls
#  define DEF_ADD_DESHAKE(func,...) func(DeshakeContext *deshake, ##__VA_ARGS__) ///< Adds the deshake parameter to prototypes and defintions.
#  define PARAM_IF(if_exper01,if_not) (if_exper01)
#else
#  define LOG_IF(cond, ctx, level, fmt, ...) do{}while(0)          ///< If EXPER01 is not defined
#  define LOG_IF_OPTMASK(opt, ctx, level, fmt, ...) do{}while(0)   ///< If EXPER01 is not defined
#  define LOG_ADD_DESHAKE(func,...) func(__VA_ARGS__)              ///< If EXPER01 is not defined
#  define DEF_ADD_DESHAKE(func,...) func(##__VA_ARGS__)            ///< If EXPER01 is not defined
#  define PARAM_IF(if_exper01,if_not) (if_not)
#endif
/** @} */

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
void exper01_draw_line(AVFilterBufferRef *avbuf, int sx, int sy, int ex, int ey,
                       int w, int h, int stride, int color);

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
void exper01_draw_arrow(AVFilterBufferRef *avbuf, int sx, int sy, int ex, int ey,
                        int w, int h, int stride, int color);

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
                  OPT_USE_TIME_TRACK_LINKED_LIST,
                  OPT_DUMP_TIME_TRACK,
                  OPT_LOG_ENTERING_TRANSFORM,
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
    u_int64_t  mask;          /** bit mask; individual bits control various options. */
    const char *shortdescr;   /** short description */
    const char *descr;        /** full description */
}OptmaskSelection;

extern OptmaskSelection optmask_selections[];

#define OPTMASK_VAL(select) (optmask_selections[select].mask)
#define OPTMASK_G(maskval,select)  ((maskval & OPTMASK_VAL(select)) == OPTMASK_VAL(select))
#define OPTMASK(select) OPTMASK_G(deshake->extra.optmask,select)
#define OPTMASK_DESCR(select) (optmask_selections[select].descr)

extern int global_option_01;
extern int global_option_02;
extern int global_option_03;
extern int global_option_04;

#ifdef EXPER01
#  define DESHAKE_WINNING_COUNT (deshake->extra.winning_count)
#  define DESHAKE_WINNING_MV (deshake->extra.imvs[0])
#else
#  define DESHAKE_WINNING_COUNT
#  define DESHAKE_WINNING_MV
#endif

// Normally in vf_deshake.c
typedef struct {
    int x;             ///< Horizontal shift
    int y;             ///< Vertical shift
} IntMotionVector;


/** Extras tacked onto DeshakeContext for testing */
typedef struct {
    u_int64_t           optmask;          /** Bit mask for on-off devel/debug options.                   */
    char               *s_optmask;        /** String returned by option code                             */
    char               *search_area;      /** Search area string from options code                       */
    IntMotionVector     imvs[10];         /** motion vector data from scans to use for final vectors     */
    int                 n_valid_imvs;     /** Number of valid motion vectors in invs                     */
    int                 winning_count;    /** The count found in find_motion that is chosen for the gmv  */
    struct timeval      tvs[16];          /** Timevals for general use; first is reserved for the time track functions. */
    int                 diff_limit;       /** Maximum diff from SAD that can still be accepted           */
    char               *logfile;          /** Optional log file                                          */
    char               *oldopts;          /** Option string when user chooses old style                  */
    int                 interpolate_luma, interpolate_chroma; /** Interpolation methods */
} DeshakeContextExtra;

/**
 * Descriptor for an arrow.
 *
 * Describes an arrow and related annotation, with provision for implementation
 * in a linked list.
 */
typedef struct s_arrow {
    int  index, count, startx, starty, endx, endy, highlight, block_x, block_y;  // Arrow
    char* annotation;
    struct s_arrow *next;
}ArrowAnnotation;

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
    int                 marker_number;        /** Number of this marker                   */
    struct s_timetrack *next, *previous;      /** links                                   */
}TimeTrack;

extern int use_time_track;

/** Add a time marker, optionally to the linked list.
 *
 * If the mask option OPT_USE_TIME_TRACK_LINKED_LIST is true,
 * adds a TimeTrack record to the end of the linked list for later
 * display; otherwise, the value of the marker is optionally displayed immediately (see below).
 * @param dcx          Pointer to deshake context extras, where the time of first marker is stored
 * @param filename     Name of source file
 * @param func         Name of function
 * @param line         Line number
 * @param label        An identifying label
 * @param fmt          printf-style format string
 * @param ...          Variadic args
 * @return             If an object was created, a pointer to the object is returned, otherwise a pointer
 * to an ephemeral object is returned.  If OPT_DUMP_TIME_TRACK is true, the value of the marker
 * is logged.
 */
const TimeTrack *add_time_marker(DeshakeContextExtra *dcx, const char *filename, const char* func, int line, const char* label, const char* fmt, ...);
/** Add a time marker to the linked list (va_list version). */
const TimeTrack *vadd_time_marker(DeshakeContextExtra *dcx, const char *filename, const char* func, int line, const char* label, const char* fmt, va_list va);

/** Log the contents of a time marker.
 *
 * Each marker consists of a label, the file, function and line number of its creation,
 * printf-style text and the time, with microsecond resolution. Times are relative to the time
 * at which the first marker was created.
 * @param marker  Pointer to the time marker
 */
void dump_time_marker(const TimeTrack *marker);

/** Log the markers in the time track.
 *
 * @param tt  Pointer to the first marker to dump.  If NULL, dump the entire track.
 */
void dump_time_track(const TimeTrack *tt);

/** Delete the time track and free memory allocated to it. */
void delete_time_track(void);

#  ifdef EXPER01
#    define ADDTIME(label,fmt,...) add_time_marker(&deshake->extra,__FILE__,__func__,__LINE__,label,fmt, ##__VA_ARGS__)
#  else
#    define ADDTIME(label,fmt,...) do {}while(0)
#  endif
#endif
