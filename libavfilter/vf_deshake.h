#ifndef VF_DESHAKE_H__
#define VF_DESHAKE_H__

/** @file
 * Header file for deshake filter
 */

#ifdef EXPER01
#include <time.h>
#  define LOGFILE (deshake->extra.logfile)
#else
#  define LOGFILE filename
#endif

/** @name Parameter defaults and user-option limits.
 * Limits provided to the AVOption system.
 * @{ */
#define RX_DEFAULT         (16)  ///< X-axis search extent default
#define RX_MAX             (63)  ///< X-axis search extent max
#define RX_MIN             (0)   ///< X-axis search extent min
#define RY_DEFAULT         (16)  ///< Y-axis search extent default
#define RY_MAX             (64)  ///< Y-axis search extent max
#define RY_MIN             (0)   ///< Y-axis search extent min
#define FILL_DEFAULT       (FILL_MIRROR)  ///< Default fill method
#define BLOCKSIZE_DEFAULT  (8)    ///<  Default block size
#define BLOCKSIZE_MAX      (128)  ///< Max block size
#define BLOCKSIZE_MIN      (8)    ///< Min block size
#define CONTRAST_DEFAULT   (125)  ///< Contrast threshold default
#define CONTRAST_MAX       (255)  ///< Contrast threshold max
#define CONTRAST_MIN       (1)    ///< Contrast threshold min
#define SEARCH_DEFAULT     (EXHAUSTIVE)  ///< Default search type
#define INTERPOLATE_METHOD_LUMA           (deshake->extra.interpolate_luma)
#define INTERPOLATE_METHOD_CHROMA         (deshake->extra.interpolate_chroma)
#define INTERP_LUMA_DEFAULT               (INTERPOLATE_DEFAULT)  ///< Default interpolation, luma
#define INTERP_CHROMA_DEFAULT             (INTERPOLATE_DEFAULT)  ///< Default interpolation, chroma
#define DIFF_LIMIT_DEFAULT     (512)      ///< Default limit for SAD diff returns
#define DIFF_LIMIT_MIN         (INT_MIN)  ///< Min limit for SAD diff returns
#define DIFF_LIMIT_MAX         (INT_MAX)  ///< Max limit for SAD diff returns
#define REFERENCE_FRAMES_DEFAULT (20)     ///< Default number of reference frames for average
/** @} */

#define CHROMA_HEIGHT(link) -((-link->h) >> av_pix_fmt_descriptors[link->format].log2_chroma_h)
#define CHROMA_WIDTH(link)  -((-link->w) >> av_pix_fmt_descriptors[link->format].log2_chroma_w)

/** Options for motion searches. */
enum SearchMethod {
    EXHAUSTIVE,        ///< Search all possible positions
    SMART_EXHAUSTIVE,  ///< Search most possible positions (faster)
    SEARCH_COUNT
};

/** Description of a particular displacement.
 *  Versions for integer and double types  */
typedef struct {
    double x;             ///< Horizontal shift
    double y;             ///< Vertical shift
} MotionVector;

/* Temporarily in exper01.h
   typedef struct {
   int x;             ///< Horizontal shift
   int y;             ///< Vertical shift
   } IntMotionVector;
*/

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
#ifdef EXPER01
    AVClass  *av_class;
#else
    AVClass av_class;
#endif
// In same order as option string; easier debugging that way.
    int cx;                    ///< cx, cy, cw, ch: Restrict motion search to this area
    int cy;
    int cw;
    int ch;
    int rx;                    ///< Horizontal search extent
    int ry;                    ///< Vertical search extent
    enum FillMethod edge;      ///< Edge fill method
    int blocksize;             ///< Size of blocks to compare
    int contrast;              ///< Contrast threshold
    enum SearchMethod search;  ///< Motion search method
// End of the standard user options.
    int   reference_frames;    ///< Number of reference frames (defines averaging window)
#ifdef EXPER01
    DeshakeContextExtra  extra;  ///< Extra stuff for development.
#endif
    FILE *fp;                  ///< FILE pointer; non-null if user has requested the file in the options.
    Transform avg, last;       ///< Transforms: running average and last frame
    AVFilterBufferRef *ref;    ///< Previous frame
    AVCodecContext *avctx;
    DSPContext c;              ///< Context providing optimized SAD methods
}DeshakeContext;

#ifdef EXPER01
//void draw_vectors(DeshakeContext *deshake, AVFilterBufferRef *avbuf, int w, int h, int stride, Transform *t, Transform *orig, int normalizing_scale, int color, int highlight_color);
void do_vectors(DeshakeContext *deshake, AVFilterLink *link, Transform *t, Transform *orig);
void find_motion_generate_block_vectors(DeshakeContext *deshake, int x, int y, int (*counts)[BLOCKSIZE_MAX], IntMotionVector *mv);
int DEF_ADD_DESHAKE(block_contrast,uint8_t *src, int x, int y, int stride, int blocksize);
/** Root of linked list of vector arrows to draw.
 * @see draw_vectors() */
extern ArrowAnnotation *arrow_root;
extern unsigned long fcount;
extern unsigned long icount;
#else
static int block_contrast(uint8_t *src, int x, int y, int stride, int blocksize);
#endif

double block_angle(int x, int y, int cx, int cy, IntMotionVector *shift);
double clean_mean(double *values, int count);
#endif
