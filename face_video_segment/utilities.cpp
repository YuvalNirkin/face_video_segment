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
        cv::Scalar poly_color;
        for (const auto& r : seg_desc.region())
        {
            if (r.vectorization().polygon().empty()) continue;
            auto& face_region = regions.find((unsigned int)r.id());
            if (face_region == regions.end()) continue;

            // Find holes
            std::vector<std::vector<cv::Point>> holes;
            for (const auto& poly : r.vectorization().polygon())
            {
                if (!poly.hole()) continue;
                if (poly.coord_idx_size() == 0) continue;
                createContours(mesh, poly, holes);
            }

            // For each polygon
            int poly_ind = 0;
            for (const auto& poly : r.vectorization().polygon())
            {
                if (poly.hole()) continue;
                if (poly.coord_idx_size() == 0) continue;
                PolygonType type = face_region->second.polygons(poly_ind++);
                if (type == EMPTY) continue;
                std::vector<std::vector<cv::Point>> contours;
                createContours(mesh, poly, contours);

                if (!contours.empty())
                {
                    // Set polygon color
                    if (type == FULL) poly_color = cv::Scalar(255, 255, 255);
                    else poly_color = cv::Scalar(128, 128, 128);

                    // Render polygon
                    cv::drawContours(poly_map, contours, 0, poly_color, CV_FILLED);

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

    void removeSmallerComponents(cv::Mat& seg)
    {
        cv::Mat labels;
        cv::Mat stats, centroids;
        cv::connectedComponentsWithStats(seg, labels, stats, centroids);
        if (stats.rows <= 2) return;

        // Find the label of the connected component with maximum area
        cv::Mat areas = stats.colRange(4, 5).clone();
        int* areas_data = (int*)areas.data;
        int max_label = std::distance(areas_data,
            std::max_element(areas_data + 1, areas_data + stats.rows));

        // Clear smaller components
        unsigned char* seg_data = seg.data;
        int* labels_data = (int*)labels.data;
        for (size_t i = 0; i < seg.total(); ++i, ++seg_data)
            if (*labels_data++ != max_label) *seg_data = 0;
    }

    void smoothFlaws(cv::Mat& seg, int smooth_iterations = 1, int smooth_kernel_radius = 2)
    {
        int kernel_size = smooth_kernel_radius * 2 + 1;
        cv::Mat kernel = cv::getStructuringElement(
            cv::MorphShapes::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
//        for (int i = 0; i < smooth_iterations; ++i)
        {
            cv::morphologyEx(seg, seg, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), smooth_iterations);
            cv::morphologyEx(seg, seg, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), smooth_iterations);
        }
    }

    void fillHoles(cv::Mat& seg)
    {
        double min_val, max_val;
        cv::minMaxLoc(seg, &min_val, &max_val);
        cv::Mat holes = seg.clone();
        cv::floodFill(holes, cv::Point2i(0, 0), cv::Scalar(max_val));
        for (size_t i = 0; i < seg.total(); ++i)
        {
            if (holes.data[i] == 0)
                seg.data[i] = (unsigned char)max_val;
        }
    }

    void postprocessSegmentation(cv::Mat & seg, bool disconnected,
        bool holes, bool smooth, int smooth_iterations, int smooth_kernel_radius)
    {
        if(disconnected) removeSmallerComponents(seg);
        if(holes) fillHoles(seg);
        if(smooth) smoothFlaws(seg, smooth_iterations, smooth_kernel_radius);
        if(disconnected) removeSmallerComponents(seg);
        if(holes) fillHoles(seg);
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

    void renderSegmentation(cv::Mat& img, const cv::Mat& seg, uchar color)
    {
        if (img.size() != seg.size()) img.create(seg.size(), seg.type());

        int r, c;
        unsigned char* img_data = img.data;
        const unsigned char* seg_data = seg.data;
        for (r = 0; r < img.rows; ++r)
        {
            seg_data = seg.ptr<uchar>(r);
            img_data = img.ptr<uchar>(r);
            for (c = 0; c < img.cols; ++c)
            {
                if (*seg_data++ > 0) *img_data++ = color;
                else *img_data++ = 0;
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

    float getFaceDominantSide(const std::vector<cv::Point>& landmarks)
    {
        if (landmarks.size() != 68) return 0;

        const cv::Point& center = landmarks[27];
        const cv::Point& left_eye = landmarks[42];
        const cv::Point& right_eye = landmarks[39];
        float left_dist = cv::norm(center - left_eye);
        float right_dist = cv::norm(center - right_eye);

        return left_dist / (left_dist + right_dist);
    }

    cv::Rect getFaceBBoxFromSegmentation(const cv::Mat& seg, bool square)
    {
        int xmin(std::numeric_limits<int>::max()), ymin(std::numeric_limits<int>::max()),
            xmax(-1), ymax(-1), sumx(0), sumy(0);
        int r, c, foreground_count = 0;

        // For each foreground point
        unsigned char* seg_data = seg.data;
        for (r = 0; r < seg.rows; ++r)
        {
            for (c = 0; c < seg.cols; ++c)
            {
                if (*seg_data++ == 0) continue;
                xmin = std::min(xmin, c);
                ymin = std::min(ymin, r);
                xmax = std::max(xmax, c);
                ymax = std::max(ymax, r);
                sumx += c;
                sumy += r;
                ++foreground_count;
            }
        }

        int width = xmax - xmin + 1;
        int height = ymax - ymin + 1;
        int centerx = (xmin + xmax) / 2;
        int centery = (ymin + ymax) / 2;
        int avgx = (int)std::round(sumx / foreground_count);
        int avgy = (int)std::round(sumy / foreground_count);
        int devx = centerx - avgx;
        int devy = centery - avgy;
        int dleft = (int)std::round(0.1*width) + abs(devx < 0 ? devx : 0);
        int dtop = (int)std::round(height*(std::max(float(width) / height, 1.0f) * 1.5f - 1)) + abs(devy < 0 ? devy : 0);
        int dright = (int)std::round(0.1*width) + abs(devx > 0 ? devx : 0);
        int dbottom = (int)std::round(0.1*height) + abs(devy > 0 ? devy : 0);

        // Limit to frame boundaries
        xmin = std::max(0, xmin - dleft);
        ymin = std::max(0, ymin - dtop);
        xmax = std::min(seg.cols - 1, xmax + dright);
        ymax = std::min(seg.rows - 1, ymax + dbottom);

        // Make square
        if (square)
        {
            int sq_width = std::max(xmax - xmin + 1, ymax - ymin + 1);
            centerx = (xmin + xmax) / 2;
            centery = (ymin + ymax) / 2;
            xmin = centerx - ((sq_width - 1) / 2);
            ymin = centery - ((sq_width - 1) / 2);
            xmax = xmin + sq_width - 1;
            ymax = ymin + sq_width - 1;

            // Limit to frame boundaries
            xmin = std::max(0, xmin);
            ymin = std::max(0, ymin);
            xmax = std::min(seg.cols - 1, xmax);
            ymax = std::min(seg.rows - 1, ymax);
        }

        return cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax));
    }

}   // namespace fvs

