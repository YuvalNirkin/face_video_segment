#include "utilities.h"

#include <segment_util/segmentation_render.h>

// OpenCV
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>  // debug

using std::runtime_error;
using namespace segmentation;

namespace fvs
{
    void renderBoundaries(cv::Mat& img, int hierarchy_level,
        const SegmentationDesc& desc,
        const Hierarchy* seg_hier,
        const cv::Scalar& color)
    {
        cv::Point3_<uchar> bgr((uchar)color[0], (uchar)color[1], (uchar)color[2]);
        cv::Mat tmp(img.size(), img.type());
        RenderRegionsRandomColor(hierarchy_level, false, false, desc, seg_hier, &tmp);
        //cv::imshow("img", img);
        //cv::imshow("temp", tmp);
        //cv::waitKey(0);

        // Edge highlight post-process.
        const int height = img.rows;
        const int width = img.cols;
        const int width_step = img.step[0];
        const int channels = img.channels();
        for (int i = 0; i < height - 1; ++i) {
            uint8_t* row_ptr = tmp.ptr<uint8_t>(i);
            uint8_t* out_row_ptr = img.ptr<uint8_t>(i);
            for (int j = 0; j < width - 1; ++j, row_ptr += channels, out_row_ptr += channels)
            {
                if (ColorDiff_L1(row_ptr, row_ptr + channels) != 0 ||
                    ColorDiff_L1(row_ptr, row_ptr + width_step) != 0)
                {
                    out_row_ptr[0] = bgr.x;
                    out_row_ptr[1] = bgr.y;
                    out_row_ptr[2] = bgr.z;
                }
            }

            // Last column.
            if (ColorDiff_L1(row_ptr, row_ptr + width_step) != 0)
            {
                out_row_ptr[0] = bgr.x;
                out_row_ptr[1] = bgr.y;
                out_row_ptr[2] = bgr.z;
            }
        }

        // Last row.
        uint8_t* row_ptr = tmp.ptr<uint8_t>(height - 1);
        uint8_t* out_row_ptr = img.ptr<uint8_t>(height - 1);
        for (int j = 0; j < width - 1; ++j, row_ptr += channels) {
            if (ColorDiff_L1(row_ptr, row_ptr + channels) != 0)
            {
                out_row_ptr[0] = bgr.x;
                out_row_ptr[1] = bgr.y;
                out_row_ptr[2] = bgr.z;
            }
        }
    }

}   // namespace fvs

