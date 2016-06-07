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
		// Find video stream idx.
		video_stream_idx_ = FindStreamIdx(options_.video_stream_name, set);

		if (video_stream_idx_ < 0) {
			LOG(ERROR) << "Could not find Video stream!\n";
			return false;
		}

		const VideoStream& vid_stream = set->at(video_stream_idx_)->As<VideoStream>();

		frame_width_ = vid_stream.frame_width();
		frame_height_ = vid_stream.frame_height();

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
		const SegmentationDesc& seg_desc = seg_frame.Ref();

		// If a face has been found
		std::unique_ptr<std::vector<int>> output_ids(new std::vector<int>());
		//geometricFaceSeg(landmarks, seg_desc, *output_ids);
		rasterFaceSeg(landmarks, seg_desc, *output_ids);

		// Forward input
		input->push_back(std::shared_ptr<PointerFrame<std::vector<int>>>(
			new PointerFrame<std::vector<int>>(std::move(output_ids))));

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
	
	void FaceSegmentationUnit::renderMultiPolygon(cv::Mat& img, const mpoly_t& mpoly,
		const cv::Scalar& color)
	{
		//cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		//cv::Scalar color(0, 255, 0);
		for (const poly_t& poly : mpoly)
		{
			renderRing(img, poly.outer(), color);
			for (const ring_t ring : poly.inners())
				renderRing(img, ring, color);
		}
	}

	void FaceSegmentationUnit::renderRing(cv::Mat& img, const ring_t& ring,
		const cv::Scalar& color)
	{
		if (ring.empty()) return;

		//cv::putText(img, std::to_string(0), cv::Point(ring[0].x(), ring[0].y()),
		//	cv::FONT_HERSHEY_PLAIN, 0.5, color, 1.0);
		for (int c = 1; c < ring.size(); ++c)
		{
			cv::line(img, cv::Point(ring[c].x(), ring[c].y()),
				cv::Point(ring[c - 1].x(), ring[c - 1].y()), color);
			//cv::putText(img, std::to_string(c), cv::Point(ring[c].x(), ring[c].y()),
			//	cv::FONT_HERSHEY_PLAIN, 0.5, color, 1.0);
		}
	}
	
	void FaceSegmentationUnit::geometricFaceSeg(const ring_t& landmarks,
		const SegmentationDesc& seg_desc, std::vector<int>& output_ids)
	{
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
			poly_t face{ {
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
				} };
			//bg::correct(face);

			// For each region
			const VectorMesh& mesh = seg_desc.vector_mesh();
			for (const auto& r : seg_desc.region())
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
				//bg::correct(mpoly);
				/*
				// Test for self intersection
				//if (bg::intersects(mpoly))
				if (!bg::is_valid(mpoly))
				{
				std::cout << "Found self intersection: " << std::endl;
				std::cout << bg::wkt(mpoly) << std::endl;
				for (int i = 0; i < mpoly.size(); ++i)
				//if(bg::intersects(mpoly[i]))
				if (!bg::is_valid(mpoly[i]))
				std::cout << "poly " << i << " self intersects" << std::endl;

				continue;
				}*/

				// Calculate intersection area ratio
				try
				{
					mpoly_t out;
					bg::intersection(face, mpoly, out);
					double ratio = (bg::area(out) / bg::area(mpoly));
					//std::cout << "ratio = " << ratio << std::endl;
					const double t = 0.5;

					//
					if (ratio > t)
					{
						output_ids.push_back(r.id());

						///// Debug ///
						//cv::Mat img = cv::Mat::zeros(720, 1280, CV_8UC3);
						//renderMultiPolygon(img, collection);
						////renderMultiPolygon(img, mpoly, cv::Scalar(0, 0, 255));
						//cv::imshow("debug", img);
						//cv::waitKey(0);
						///////////////
					}
				}
				catch (std::exception& e)
				{
					std::cout << e.what() << std::endl;
					std::cout << bg::wkt(mpoly) << std::endl;
				}
			}
		}
	}

	void FaceSegmentationUnit::rasterFaceSeg(const ring_t& landmarks,
		const SegmentationDesc& seg_desc, std::vector<int>& output_ids)
	{
		if (landmarks.size() != 68) return;

		// Create face contour
		cv::Point2f p19(landmarks[19].x(), landmarks[19].y());
		//cv::Point2f p21(landmarks[21].x(), landmarks[21].y());
		//cv::Point2f p22(landmarks[22].x(), landmarks[22].y());
		cv::Point2f p24(landmarks[24].x(), landmarks[24].y());
		cv::Point2f p27(landmarks[27].x(), landmarks[27].y());
		cv::Point2f p34(landmarks[34].x(), landmarks[34].y());
		cv::Point2f dir = (p27 - p34);
		cv::Point2f ptop_l = p19 + dir;
		cv::Point2f ptop_r = p24 + dir;
		std::vector<std::vector<cv::Point>> face{ {
			{ (int)landmarks[0].x(), (int)landmarks[0].y() },
			{ (int)landmarks[1].x(), (int)landmarks[1].y() },
			{ (int)landmarks[2].x(), (int)landmarks[2].y() },
			{ (int)landmarks[3].x(), (int)landmarks[3].y() },
			{ (int)landmarks[4].x(), (int)landmarks[4].y() },
			{ (int)landmarks[5].x(), (int)landmarks[5].y() },
			{ (int)landmarks[6].x(), (int)landmarks[6].y() },
			{ (int)landmarks[7].x(), (int)landmarks[7].y() },
			{ (int)landmarks[8].x(), (int)landmarks[8].y() },
			{ (int)landmarks[9].x(), (int)landmarks[9].y() },
			{ (int)landmarks[10].x(), (int)landmarks[10].y() },
			{ (int)landmarks[11].x(), (int)landmarks[11].y() },
			{ (int)landmarks[12].x(), (int)landmarks[12].y() },
			{ (int)landmarks[13].x(), (int)landmarks[13].y() },
			{ (int)landmarks[14].x(), (int)landmarks[14].y() },
			{ (int)landmarks[15].x(), (int)landmarks[15].y() },
			{ (int)landmarks[16].x(), (int)landmarks[16].y() },
			{ (int)landmarks[26].x(), (int)landmarks[26].y() },
			{ (int)ptop_r.x, (int)ptop_r.y },
			{ (int)ptop_l.x, (int)ptop_l.y },
			{ (int)landmarks[17].x(), (int)landmarks[0].y() }
			} };

		// Create face map
		cv::Mat face_map = cv::Mat::zeros(frame_height_, frame_width_, CV_8U);
		cv::drawContours(face_map, face, 0, cv::Scalar(255, 255, 255), CV_FILLED);
		//cv::imshow("face_map", face_map);
		//cv::waitKey(0);

		// Traverse regions.
		for (const auto& r : seg_desc.region())
		{
			unsigned int face_area = 0;
			unsigned int total_area = 0;
			for (const auto s : r.raster().scan_inter())
			{
				const int curr_y = s.y();
				uint8_t* out_ptr = face_map.ptr<uint8_t>(curr_y) + s.left_x();
				for (int j = 0, len = s.right_x() - s.left_x() + 1; j < len; ++j, ++out_ptr)
				{
					++total_area;
					if (*out_ptr > 0) ++face_area;
				}
			}

			// Test against threshold
			float ratio = float(face_area) / total_area;
			float t = 0.5f;
			if(ratio > t) output_ids.push_back(r.id());
		}
	}

	int FaceSegmentationRendererUnit::display_unit_count = 0;

	FaceSegmentationRendererUnit::FaceSegmentationRendererUnit(const FaceSegmentationRendererOptions& options)
		: options_(options), display_unit_id_(display_unit_count++)
	{
	}

	FaceSegmentationRendererUnit::~FaceSegmentationRendererUnit() {
	}

	bool FaceSegmentationRendererUnit::OpenStreams(StreamSet* set) {
		// Find video stream idx.
		video_stream_idx_ = FindStreamIdx(options_.video_stream_name, set);

		if (video_stream_idx_ < 0) {
			LOG(ERROR) << "Could not find Video stream!\n";
			return false;
		}

		const VideoStream& vid_stream = set->at(video_stream_idx_)->As<VideoStream>();

		const int frame_width = vid_stream.frame_width();
		const int frame_height = vid_stream.frame_height();

		// Get landmarks stream
		face_seg_stream_idx_ = FindStreamIdx(options_.face_segment_stream_name, set);
		if (face_seg_stream_idx_ < 0) {
			LOG(ERROR) << "SegmentationRenderUnit::OpenStreams: "
				<< "Could not find face segmentation stream!\n";
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
		//DataStream* landmarks_stream = new DataStream(options_.stream_name);
		//set->push_back(shared_ptr<DataStream>(landmarks_stream));

		// Open display window.
		//std::ostringstream os;
		//os << "LandmarksUnit_" << display_unit_id_;
		//window_name_ = os.str();

		window_name_ = "FaceSegmentationRendererUnit_" + std::to_string(display_unit_id_);
		cv::namedWindow(window_name_);
		cv::waitKey(10);
		return true;
	}

	void FaceSegmentationRendererUnit::ProcessFrame(FrameSetPtr input, std::list<FrameSetPtr>* output)
	{
		// Retrieve video frame
		const VideoFrame* frame = input->at(video_stream_idx_)->AsPtr<VideoFrame>();
		cv::Mat image;
		frame->MatView(&image);

		// Retrieve Segmentation.
		const PointerFrame<SegmentationDesc>& seg_frame =
			input->at(seg_stream_idx_)->As<PointerFrame<SegmentationDesc>>();

		const SegmentationDesc& seg_desc = seg_frame.Ref();

		// Retrieve selected regions
		const std::vector<int>& selected_regions = 
			static_cast<PointerFrame<const std::vector<int>>*>(input->at(face_seg_stream_idx_).get())->Ref();

		// Render
		renderSelectedRegions(image, seg_desc, selected_regions);

		// Forward input
		/*input->push_back(std::shared_ptr<PointerFrame<ring_t>>(
		new PointerFrame<ring_t>(std::move(landmarks_ring))));*/

		output->push_back(input);

		// Show image
		cv::imshow(window_name_.c_str(), image);
		cv::waitKey(1);
	}

	bool FaceSegmentationRendererUnit::PostProcess(list<FrameSetPtr>* append)
	{
		cv::destroyWindow(window_name_);
		return false;
	}

	void FaceSegmentationRendererUnit::renderSelectedRegions(cv::Mat& img,
		const SegmentationDesc& seg_desc, const std::vector<int>& region_ids,
		const cv::Scalar& color)
	{
		if (region_ids.empty()) return;

		int i = 0;
		int channels = img.channels();
		double a = 0.5;

		// Traverse regions.
		for (const auto& r : seg_desc.region()) 
		{
			if (r.id() != region_ids[i]) continue;
			++i;

			for (const auto s : r.raster().scan_inter()) 
			{
				const int curr_y = s.y();
				uint8_t* out_ptr = img.ptr<uint8_t>(curr_y) + channels * s.left_x();
				for (int j = 0, len = s.right_x() - s.left_x() + 1;
					j < len;
					++j, out_ptr += channels) {
					for (int c = 0; c < channels; ++c) {
						//out_ptr[c] = color[c];
						out_ptr[c] = (uint8_t)std::round(color[c] * a + out_ptr[c] *(1 - a));
					}
				}
			}

			if (i >= region_ids.size()) break;
		}
	}

}  // namespace video_framework.
