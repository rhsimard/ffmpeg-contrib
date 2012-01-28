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

/** name search area limits
 * Limits and default of user option for the search area
 * @{ */
#define SEARCH_AREA_DEFAULT  "-1:-1:-1:-1" ///< not right here
/** @} */
/** @name rxry
 *  Limits and default of user option for the maximum extent of movement in x and y directions
 *  @{*/
#define RX_DEFAULT         (16)
#define RX_MAX             (63)
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
/** @name alpha.
 *  Limits and default of user option for alpha value for exponential average.
 *  A negative value leaves existing default based on the number of reference frames. (New, for test, may not stay.)
 *  @{*/
#define ALPHA_DEFAULT      (-1.0)
#define ALPHA_MAX          (0.99)
#define ALPHA_MIN          (0.01)
/** @}*/
/** @name Interpolation methods.
 * @{ */
#define INTERPOLATE_METHOD_LUMA           (deshake->extra.interpolate_luma)
#define INTERPOLATE_METHOD_CHROMA         (deshake->extra.interpolate_chroma)
#define INTERP_LUMA_DEFAULT               (INTERPOLATE_DEFAULT)
#define INTERP_CHROMA_DEFAULT             (INTERPOLATE_DEFAULT)
/** @} */
/** @name diff limit.
 * Maximum diff from SAD before a seach is considered failed
 * @{ */
#define DIFF_LIMIT_DEFAULT     (512)
#define DIFF_LIMIT_MIN         (INT_MIN)
#define DIFF_LIMIT_MAX         (INT_MAX)
/** @} */
/** @name reference frames
 * Number of reference frames to use in exponential average calculation
 * @{ */
#define REFERENCE_FRAMES_DEFAULT (20)
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
    int cx;
    int cy;
    int cw;                    ///< Crop motion search to this box
    int ch;
    int rx;                    ///< Maximum horizontal shift
    int ry;                    ///< Maximum vertical shift
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
int block_contrast(uint8_t *src, int x, int y, int stride, int blocksize, DeshakeContext *deshake);
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
