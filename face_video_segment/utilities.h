/** @file
@brief Sequence face landmarks utility functions.
*/

#ifndef __FVG_UTILITIES__
#define __FVG_UTILITIES__

#include <segment_util/segmentation_util.h>
#include <face_video_segment.pb.h>

namespace fvs
{
    void createContours(const segmentation::VectorMesh& mesh,
        const segmentation::SegmentationDesc_Polygon& poly,
        std::vector<std::vector<cv::Point>>& contours);

    cv::Mat calcSegmentation(cv::Size size,
        const google::protobuf::Map<google::protobuf::uint32, Region>& regions,
        const segmentation::SegmentationDesc& seg_desc);

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

    void renderSegmentationBlend(cv::Mat& img, const cv::Mat& seg,
        const cv::Scalar& full_color = cv::Scalar(0, 0, 255),
        const cv::Scalar& intersection_color = cv::Scalar(255, 0, 0));

}   // namespace fvs

#endif	// __FVG_UTILITIES__
