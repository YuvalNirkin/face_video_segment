#include "utilities.h"

#include <segment_util/segmentation_render.h>

// OpenCV
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>  // debug

using std::runtime_error;
using namespace segmentation;

namespace fvs
{
    void createContours(const VectorMesh& mesh,
        const SegmentationDesc_Polygon& poly,
        std::vector<std::vector<cv::Point>>& contours)
    {
        contours.emplace_back();
        std::vector<cv::Point>& contour = contours.back();

        // For each coordinate
        contour.resize(poly.coord_idx_size() - 1);
        for (int c = 0; c < contour.size(); ++c)
        {
            int idx = poly.coord_idx(c);
            contour[c] = cv::Point(mesh.coord(idx), mesh.coord(idx + 1));
        }
    }

    cv::Mat calcSegmentation(cv::Size size, 
        const google::protobuf::Map<google::protobuf::uint32, Region>& regions,
        const segmentation::SegmentationDesc & seg_desc)
    {
        cv::Mat face_map = cv::Mat::zeros(size, CV_8U);
        return calcSegmentation(face_map, regions, seg_desc);
    }

    cv::Mat calcSegmentation(const cv::Mat& face_map, 
        const google::protobuf::Map<google::protobuf::uint32, Region>& regions,
        const segmentation::SegmentationDesc& seg_desc)
    {
        cv::Mat seg = cv::Mat::zeros(face_map.size(), CV_8U);
        const VectorMesh& mesh = seg_desc.vector_mesh();

        // For each region
        cv::Mat poly_map = cv::Mat::zeros(face_map.size(), CV_8U);
        cv::Scalar region_color;
        for (const auto& r : seg_desc.region())
        {
            if (r.vectorization().polygon().empty()) continue;
            auto& face_region = regions.find((unsigned int)r.id());
            if (face_region == regions.end()) continue;
            RegionType type = face_region->second.type();
            if (type == FULL) region_color = cv::Scalar(255, 255, 255);
            else region_color = cv::Scalar(128, 128, 128);

            // Find holes
            std::vector<std::vector<cv::Point>> holes;
            for (const auto& poly : r.vectorization().polygon())
            {
                if (!poly.hole()) continue;
                if (poly.coord_idx_size() == 0) continue;
                createContours(mesh, poly, holes);
            }

            // For each polygon
            for (const auto& poly : r.vectorization().polygon())
            {
                if (poly.hole()) continue;
                if (poly.coord_idx_size() == 0) continue;
                std::vector<std::vector<cv::Point>> contours;
                createContours(mesh, poly, contours);

                if (!contours.empty())
                {
                    // Render polygon
                    cv::drawContours(poly_map, contours, 0, region_color, CV_FILLED);

                    // Remove holes
                    cv::drawContours(poly_map, holes, 0, cv::Scalar(0, 0, 0), CV_FILLED);

                    // Add to segmentation
                    unsigned char *seg_data = seg.data, *poly_map_data = poly_map.data;
                    unsigned char *face_map_data = face_map.data;
                    unsigned char pv = 0;
                    for (size_t i = 0; i < seg.total(); ++i)
                    {
                        pv = *poly_map_data++;
                        if (pv == 255 || (pv == 128 && *face_map_data > 0))
                            *seg_data = pv;
                        ++seg_data;
                        ++face_map_data;
                    }

                    // Clear map
                    cv::drawContours(poly_map, contours, 0, cv::Scalar(0, 0, 0), CV_FILLED);
                }
            }
        }

        return seg;
    }

    void createFullFace(const std::vector<cv::Point>& landmarks, std::vector<cv::Point>& full_face)
    {
        if (landmarks.size() != 68) return;

        // Jaw line
        full_face = {
            { landmarks[0] },
            { landmarks[1] },
            { landmarks[2] },
            { landmarks[3] },
            { landmarks[4] },
            { landmarks[5] },
            { landmarks[6] },
            { landmarks[7] },
            { landmarks[8] },
            { landmarks[9] },
            { landmarks[10] },
            { landmarks[11] },
            { landmarks[12] },
            { landmarks[13] },
            { landmarks[14] },
            { landmarks[15] },
            { landmarks[16] }
        };

        // Forehead
        cv::Point dir = (landmarks[27] - landmarks[30]);
        if (landmarks[26].x > landmarks[16].x) full_face.push_back(landmarks[26]);
        full_face.push_back(landmarks[26] + dir);
        full_face.push_back(landmarks[24] + dir);
        full_face.push_back(landmarks[19] + dir);
        full_face.push_back(landmarks[17] + dir);
        if (landmarks[17].x < landmarks[0].x) full_face.push_back(landmarks[17]);
    }

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

    void renderSegmentationBlend(cv::Mat& img, const cv::Mat& seg, float alpha,
        const cv::Scalar& full_color, const cv::Scalar& intersection_color)
    {
        cv::Point3_<uchar> full_bgr((uchar)full_color[0], (uchar)full_color[1],
            (uchar)full_color[2]);
        cv::Point3_<uchar> intersection_bgr((uchar)intersection_color[0],
            (uchar)intersection_color[1], (uchar)intersection_color[2]);

        int r, c;
        cv::Point3_<uchar>* img_data = (cv::Point3_<uchar>*)img.data;
        unsigned char* seg_data = seg.data;
        unsigned char s;
        for (r = 0; r < img.rows; ++r)
        {
            for (c = 0; c < img.cols; ++c)
            {
                s = *seg_data++;
                if (s == 255)
                {
                    img_data->x = (unsigned char)std::round(full_bgr.x * alpha + img_data->x*(1 - alpha));
                    img_data->y = (unsigned char)std::round(full_bgr.y * alpha + img_data->y*(1 - alpha));
                    img_data->z = (unsigned char)std::round(full_bgr.z * alpha + img_data->z*(1 - alpha));
                }
                else if (s == 128)
                {
                    img_data->x = (unsigned char)std::round(intersection_bgr.x * alpha + img_data->x*(1 - alpha));
                    img_data->y = (unsigned char)std::round(intersection_bgr.y * alpha + img_data->y*(1 - alpha));
                    img_data->z = (unsigned char)std::round(intersection_bgr.z * alpha + img_data->z*(1 - alpha));
                }
                ++img_data;
            }
        }
    }

}   // namespace fvs

