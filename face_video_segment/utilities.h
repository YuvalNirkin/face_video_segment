/** @file
@brief Sequence face landmarks utility functions.
*/

#ifndef __FVG_UTILITIES__
#define __FVG_UTILITIES__

#include <segment_util/segmentation_util.h>

namespace fvs
{
    /** Render the boundaries of the segmentation regions.
        @param img The image that boundaries will be rendered on.
        @hierarchy_level The level of the hierarchy.
        0 denotes the over-segmentation. >= 1 : denotes a hierarchical level.
        @desc The segmentation descriptor which contains the segmentations data.
        @seg_hier The hierarchy information.
        @color Boundaries color.
    */
    void renderBoundaries(cv::Mat& img, int hierarchy_level,
        const segmentation::SegmentationDesc& desc,
        const segmentation::Hierarchy* seg_hier,
        const cv::Scalar& color = cv::Scalar(255, 255, 255));

}   // namespace fvs

#endif	// __FVG_UTILITIES__
