// Copyright (c) 2010-2014, The Video Segmentation Project
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the The Video Segmentation Project nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// ---

#include "face_regions.h"
#include "utilities.h"

// std
#include <iostream> // debug

// OpenCV
#include <opencv2/imgproc.hpp>

using namespace segmentation;

namespace fvs 
{
    FaceRegions::FaceRegions()
    {
    }

    void FaceRegions::addFrame(const SegmentationDesc& seg_desc,
        const sfl::Frame& sfl_frame, Frame& fvs_frame)
    {
        // If it is the first frame, save hierarchy.
        if (m_seg_hier == nullptr) m_seg_hier.reset(new SegmentationDesc(seg_desc));
        else if (seg_desc.hierarchy_size() > 0)		// Update hierarchy when one present.
            *m_seg_hier = seg_desc;

        // For each sfl face
        for (auto& sfl_face : sfl_frame.faces)
            addFace(seg_desc, *sfl_face, fvs_frame);
    }

    void FaceRegions::addFace(const segmentation::SegmentationDesc& seg_desc,
        const sfl::Face& sfl_face, Frame& fvs_frame)
    {
        auto& faces = *fvs_frame.mutable_faces();
        Face& fvs_face = faces[sfl_face.id];
        auto& regions = *fvs_face.mutable_regions();
        const VectorMesh& mesh = seg_desc.vector_mesh();
        cv::Mat face_map = createFaceMap(
            cv::Size(fvs_frame.width(), fvs_frame.height()), sfl_face.landmarks);

        // Calculate total face area and jaw area
        unsigned char *face_map_data = face_map.data;
        unsigned int total_face_area = 0, total_jaw_area = 0;
        for (size_t i = 0; i < face_map.total(); ++i, ++face_map_data)
        {
            if (*face_map_data == 255) ++total_face_area;
            else if (*face_map_data == 128) ++total_jaw_area;
        }
        
        // For each region
        cv::Mat poly_map = cv::Mat::zeros(face_map.size(), CV_8U);
        for (const auto& r : seg_desc.region())
        {
            if (r.vectorization().polygon().empty()) continue;
            std::vector<PolygonType> poly_types;
            poly_types.reserve(r.vectorization().polygon_size());

            // Find holes
            std::vector<std::vector<cv::Point>> holes;
            for (const auto& poly : r.vectorization().polygon())
            {
                if (!poly.hole()) continue;
                if (poly.coord_idx_size() == 0) continue;
                createContours(mesh, poly, holes);
            }

            // For each polygon
            PolygonType poly_type;
            for (const auto& poly : r.vectorization().polygon())
            {
                if (poly.hole()) continue;
                if (poly.coord_idx_size() == 0) continue;

                poly_type = EMPTY;
                std::vector<std::vector<cv::Point>> contours;
                createContours(mesh, poly, contours);

                if (!contours.empty())
                {
                    // Render polygon
                    cv::drawContours(poly_map, contours, 0, cv::Scalar(255, 255, 255), CV_FILLED);

                    // Remove holes
                    cv::drawContours(poly_map, holes, 0, cv::Scalar(0, 0, 0), CV_FILLED);

                    // Compare maps
                    unsigned char *face_map_data = face_map.data, *poly_map_data = poly_map.data;
                    unsigned char fp;
                    unsigned int face_area = 0, total_area = 0, jaw_area = 0;
                    int pr, pc;
                    float avg_out_r = 0, avg_out_c = 0;
                    for (pr = 0; pr < face_map.rows; ++pr)
                    {
                        for (pc = 0; pc < face_map.cols; ++pc)
                        {
                            if (*poly_map_data++ > 0)
                            {
                                ++total_area;
                                fp = *face_map_data++;
                                if (fp == 255) ++face_area;
                                else	// Outside face
                                {
                                    if (fp == 128) ++jaw_area;	// Jaw outline
                                    avg_out_r += (float)pr;
                                    avg_out_c += (float)pc;
                                }

                            }
                            else ++face_map_data;
                        }
                    }
                    unsigned int out_area = total_area - face_area;
                    float cos_a = 0;
                    if (out_area > 0)
                    {
                        avg_out_r /= out_area;
                        avg_out_c /= out_area;
                        cv::Point2f poly_center(std::round(avg_out_c), std::round(avg_out_r));

                        cv::Point2f face_dir = sfl_face.landmarks[8] - sfl_face.landmarks[27];
                        cv::Point2f poly_dir = poly_center - cv::Point2f(sfl_face.landmarks[27]);
                        face_dir /= cv::norm(face_dir);
                        poly_dir /= cv::norm(poly_dir);
                        cos_a = face_dir.dot(poly_dir);
                    }

                    // Test against threshold
                    if (total_area > 0)
                    {
                        float in_ratio = float(face_area) / total_area;
                        float out_ratio = float(out_area) / total_area;
                        float jaw_poly_ratio = float(jaw_area) / total_area;
                        float jaw_ratio = float(jaw_area) / total_jaw_area;
                        float in_face_ratio = float(face_area) / total_face_area;
                        float out_face_ratio = float(out_area) / total_face_area;

                        if (in_face_ratio > 0.01f && out_face_ratio > 0.01f && (jaw_ratio > 0.05f && jaw_poly_ratio > 0.02f && jaw_poly_ratio < 0.5f))
                        {
                            // Found neck region
                            poly_type = INTERSECTION;
                        }
                        else if (in_ratio > 0.5f) poly_type = FULL;
                    }

                    // Clear map
                    cv::drawContours(poly_map, contours, 0, cv::Scalar(0, 0, 0), CV_FILLED);
                }

                // Add polygon type
                poly_types.push_back(poly_type);
            }

            // Check whether there is at least one non empty polygon
            bool add_region = false;
            for (PolygonType type : poly_types)
                if (type != EMPTY) add_region = true;

            // Add new face region
            if (add_region)
            {
                Region& fvs_region = regions[r.id()];
                for (PolygonType type : poly_types)
                    fvs_region.add_polygons(type);
            }
        }
    }

    cv::Mat FaceRegions::createFaceMap(const cv::Size& size,
        const std::vector<cv::Point>& landmarks)
    {
        std::vector<std::vector<cv::Point>> face(1);
        createFullFace(landmarks, face.back());
        cv::Mat face_map = cv::Mat::zeros(size, CV_8U);

        // Fill jaw
        float jaw_width = cv::norm(landmarks[2] - landmarks[14]);
        float side = getFaceDominantSide(landmarks);
        const float min_side = 0.3f, max_side = 0.7f;
        side = std::max(std::min(side, max_side), min_side);
        side = (side - min_side) / (max_side - min_side);
        float right_ratio = std::max(side - 0.5f, 0.0f)*2.0f;
        float left_ratio = std::min(side, 0.5f)*2.0f;
        int right_ind = 3 + (int)std::round(right_ratio * 5);
        int left_ind = 8 + (int)std::round(left_ratio * 5);
        int jaw_thickness = (int)std::round(0.1f*jaw_width);
        for (size_t i = right_ind + 1; i <= left_ind; ++i)
            cv::line(face_map, landmarks[i], landmarks[i - 1],
                cv::Scalar(128, 128, 128), jaw_thickness);

        // Fill face
        cv::drawContours(face_map, face, 0, cv::Scalar(255, 255, 255), CV_FILLED);

        /// Debug ///
        //std::cout << "side = " << side << std::endl;
        //std::cout << "right_ratio = " << right_ratio << std::endl;
        //std::cout << "left_ratio = " << left_ratio << std::endl;
        /////////////

        return face_map;
    }

}  // namespace fvs
