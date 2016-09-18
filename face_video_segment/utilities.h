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

    cv::Mat calcSegmentation(const cv::Mat& face_map,
        const google::protobuf::Map<google::protobuf::uint32, Region>& regions,
        const segmentation::SegmentationDesc& seg_desc);

    void postprocessSegmentation(cv::Mat& seg);

//    void createFullFace(const std::vector<cv::Point>& landmarks,
//        std::vector<cv::Point>& full_face);

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

    void renderSegmentation(cv::Mat& img, const cv::Mat& seg, uchar color);

    /** Render segmentation blended with image
    @param img The image that the segmentation will be blended with.
    @seg The segmentation as an 8-bit image.
    Values of 255 denote full regions and values of 128 denote intersection regions.
    @alpha Blending weight [0, 1].
    0 means completely transparent and 1 means completely opaque.
    @full_color The color to be used for full regions.
    @intersection_color The color to be used for intersection regions.
    */
    void renderSegmentationBlend(cv::Mat& img, const cv::Mat& seg, float alpha = 0.5f,
        const cv::Scalar& full_color = cv::Scalar(0, 0, 255),
        const cv::Scalar& intersection_color = cv::Scalar(255, 0, 0));

    float getFaceDominantSide(const std::vector<cv::Point>& landmarks);

    /** @brief Get face bounding box from segmentation.
    @param seg The segmentation.
    @param square Make the bounding box square (limited to segmentation boundaries).
    */
    cv::Rect getFaceBBoxFromSegmentation(const cv::Mat& seg, bool square);

}   // namespace fvs

#endif	// __FVG_UTILITIES__
