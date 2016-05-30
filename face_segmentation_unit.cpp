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

#include "face_segmentation_unit.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "base/base_impl.h"
#include <segmentation/segmentation.h>

using namespace video_framework;

namespace segmentation
{
	FaceSegmentationUnit::FaceSegmentationUnit(const FaceSegmentationOptions& options)
		: options_(options)
	{
	}

	FaceSegmentationUnit::~FaceSegmentationUnit() {
	}

	bool FaceSegmentationUnit::OpenStreams(StreamSet* set)
	{
		// Get landmarks stream
		landmarks_stream_idx_ = FindStreamIdx(options_.landmarks_stream_name, set);
		if (landmarks_stream_idx_ < 0) {
			LOG(ERROR) << "SegmentationRenderUnit::OpenStreams: "
				<< "Could not find landmarks stream!\n";
			return false;
		}

		// Get segmentation stream.
		seg_stream_idx_ = FindStreamIdx(options_.segment_stream_name, set);
		if (seg_stream_idx_ < 0) {
			LOG(ERROR) << "SegmentationRenderUnit::OpenStreams: "
				<< "Could not find Segmentation stream!\n";
			return false;
		}

		// Add stream.
		set->push_back(shared_ptr<DataStream>(new DataStream(options_.stream_name)));

		return true;
	}

	void FaceSegmentationUnit::ProcessFrame(FrameSetPtr input, std::list<FrameSetPtr>* output)
	{
		// Retrieve landmarks
		const PointerFrame<ring_t>& landmarks_frame =
			input->at(landmarks_stream_idx_)->As<PointerFrame<ring_t>>();

		const ring_t& landmarks = landmarks_frame.Ref();

		// Retrieve Segmentation.
		const PointerFrame<SegmentationDesc>& seg_frame =
			input->at(seg_stream_idx_)->As<PointerFrame<SegmentationDesc>>();

		const SegmentationDesc& desc = seg_frame.Ref();

		// If a face has been found
		if (landmarks.size() == 68)
		{
			// Create face ring
			cv::Point2f p19(landmarks[19].x(), landmarks[19].y());
			//cv::Point2f p21(landmarks[21].x(), landmarks[21].y());
			//cv::Point2f p22(landmarks[22].x(), landmarks[22].y());
			cv::Point2f p24(landmarks[24].x(), landmarks[24].y());
			cv::Point2f p27(landmarks[27].x(), landmarks[27].y());
			cv::Point2f p34(landmarks[34].x(), landmarks[34].y());
			cv::Point2f dir = (p27 - p34);
			cv::Point2f ptop_l = p19 + dir;
			cv::Point2f ptop_r = p24 + dir;
			ring_t face{
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
				{ landmarks[16] },
				{ landmarks[26] },
				{ ptop_r.x, ptop_r.y },
				{ ptop_l.x, ptop_l.y },
				{ landmarks[17] },
				{ landmarks[0] }
			};
			bg::correct(face);

			// For each region
			const VectorMesh& mesh = desc.vector_mesh();
			for (const auto& r : desc.region())
			{
				if (r.vectorization().polygon().empty()) continue;
				mpoly_t mpoly;

				// For each polygon
				for (const auto& poly : r.vectorization().polygon())
				{
					if (poly.coord_idx_size() == 0) continue;
					if (poly.hole())
					{
						ring_t ring;
						createRing(mesh, poly, ring);

						// For each polygon already added to the multipolygon 
						// (assumes that all holes come after the outer polygons)
						for (poly_t& bpoly : mpoly)
						{
							if (bg::covered_by(ring, bpoly))
							{
								bpoly.inners().push_back(ring);
								break;
							}
						}
					}
					else
					{
						mpoly.resize(mpoly.size() + 1);
						createRing(mesh, poly, mpoly.back().outer());
					}
				}

				// Calculate intersection area ratio
				mpoly_t out;
				bg::intersection(face, mpoly, out);
				double ratio = (bg::area(out) / bg::area(mpoly));
				std::cout << "ratio = " << ratio << std::endl;
				const double t = 0.5;
			}
		}

		// Forward input
		/*input->push_back(std::shared_ptr<PointerFrame<ring_t>>(
			new PointerFrame<ring_t>(std::move(landmarks_ring))));*/

		output->push_back(input);
	}

	bool FaceSegmentationUnit::PostProcess(list<FrameSetPtr>* append)
	{
		return false;
	}

	void FaceSegmentationUnit::createRing(const VectorMesh& mesh,
		const SegmentationDesc_Polygon& poly, ring_t& ring)
	{
		// For each coordinate
		ring.resize(poly.coord_idx_size() - 1);
		for (int c = 0; c < ring.size(); ++c)
		{
			int idx = poly.coord_idx(c);
			ring[c] = point_t(mesh.coord(idx), mesh.coord(idx + 1));
		}

		bg::correct(ring);
	}
	/*
	void FaceSegmentationUnit::renderMultiPolygon(cv::Mat& img, const mpoly_t& mpoly)
	{
		cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		//cv::Scalar color(0, 255, 0);
		for (const poly_t& poly : mpoly)
		{
			renderRing(img, poly.outer(), color);
			for (const ring_t ring : poly.inners())
				renderRing(img, poly.outer(), color);
		}
	}

	void FaceSegmentationUnit::renderRing(cv::Mat& img, const ring_t& ring,
		const cv::Scalar& color)
	{
		if (ring.empty()) return;

		cv::putText(img, std::to_string(0), cv::Point(ring[0].x(), ring[0].y()),
			cv::FONT_HERSHEY_PLAIN, 0.5, color, 1.0);
		for (int c = 1; c < ring.size(); ++c)
		{
			cv::line(img, cv::Point(ring[c].x(), ring[c].y()),
				cv::Point(ring[c - 1].x(), ring[c - 1].y()), color);
			cv::putText(img, std::to_string(c), cv::Point(ring[c].x(), ring[c].y()),
				cv::FONT_HERSHEY_PLAIN, 0.5, color, 1.0);
		}
	}
	*/

}  // namespace video_framework.
