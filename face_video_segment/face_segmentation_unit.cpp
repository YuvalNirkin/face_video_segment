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
#include "utilities.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "base/base_impl.h"
#include <segmentation/segmentation.h>
#include <segment_util/segmentation_render.h>
#include <sfl/utilities.h>

using namespace video_framework;
using namespace segmentation;

namespace fvs
{
	// Utilities

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
		const PointerFrame<std::vector<cv::Point>>& landmarks_frame =
			input->at(landmarks_stream_idx_)->As<PointerFrame<std::vector<cv::Point>>>();

		const std::vector<cv::Point>& landmarks = landmarks_frame.Ref();

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

	void FaceSegmentationUnit::rasterFaceSeg(const std::vector<cv::Point>& landmarks,
		const SegmentationDesc& seg_desc, std::vector<int>& output_ids)
	{
		if (landmarks.size() != 68) return;

		// Create face contour
		cv::Point2f p19(landmarks[19].x, landmarks[19].y);
		//cv::Point2f p21(landmarks[21].x, landmarks[21].y);
		//cv::Point2f p22(landmarks[22].x, landmarks[22].y);
		cv::Point2f p24(landmarks[24].x, landmarks[24].y);
		cv::Point2f p27(landmarks[27].x, landmarks[27].y);
		cv::Point2f p34(landmarks[34].x, landmarks[34].y);
		cv::Point2f dir = (p27 - p34);
		cv::Point2f ptop_l = p19 + dir;
		cv::Point2f ptop_r = p24 + dir;
		std::vector<std::vector<cv::Point>> face{ {
			{ (int)landmarks[0].x, (int)landmarks[0].y },
			{ (int)landmarks[1].x, (int)landmarks[1].y },
			{ (int)landmarks[2].x, (int)landmarks[2].y },
			{ (int)landmarks[3].x, (int)landmarks[3].y },
			{ (int)landmarks[4].x, (int)landmarks[4].y },
			{ (int)landmarks[5].x, (int)landmarks[5].y },
			{ (int)landmarks[6].x, (int)landmarks[6].y },
			{ (int)landmarks[7].x, (int)landmarks[7].y },
			{ (int)landmarks[8].x, (int)landmarks[8].y },
			{ (int)landmarks[9].x, (int)landmarks[9].y },
			{ (int)landmarks[10].x, (int)landmarks[10].y },
			{ (int)landmarks[11].x, (int)landmarks[11].y },
			{ (int)landmarks[12].x, (int)landmarks[12].y },
			{ (int)landmarks[13].x, (int)landmarks[13].y },
			{ (int)landmarks[14].x, (int)landmarks[14].y },
			{ (int)landmarks[15].x, (int)landmarks[15].y },
			{ (int)landmarks[16].x, (int)landmarks[16].y },
			{ (int)landmarks[26].x, (int)landmarks[26].y },
			{ (int)ptop_r.x, (int)ptop_r.y },
			{ (int)ptop_l.x, (int)ptop_l.y },
			{ (int)landmarks[17].x, (int)landmarks[17].y }
			} };

		// Create face map
		cv::Mat face_map = cv::Mat::zeros(frame_height_, frame_width_, CV_8U);
		cv::drawContours(face_map, face, 0, cv::Scalar(255, 255, 255), CV_FILLED);
		cv::imshow("face_map", face_map);
		cv::waitKey(0);

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

	FaceSegGlobalUnit::FaceSegGlobalUnit(const FaceSegGlobalOptions& options)
		: options_(options)
	{
	}

	FaceSegGlobalUnit::~FaceSegGlobalUnit() {
	}

	bool FaceSegGlobalUnit::OpenStreams(StreamSet* set)
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
		//set->push_back(shared_ptr<DataStream>(new DataStream(options_.stream_name)));

		return true;
	}

	void FaceSegGlobalUnit::ProcessFrame(FrameSetPtr input, std::list<FrameSetPtr>* output)
	{
		// Retrieve video frame
		const VideoFrame* frame = input->at(video_stream_idx_)->AsPtr<VideoFrame>();
		frame->MatView(&frame_);

		// Retrieve landmarks
		const PointerFrame<std::vector<cv::Point>>& landmarks_frame =
			input->at(landmarks_stream_idx_)->As<PointerFrame<std::vector<cv::Point>>>();

		const std::vector<cv::Point>& landmarks = landmarks_frame.Ref();

		// Retrieve Segmentation.
		const PointerFrame<SegmentationDesc>& seg_frame =
			input->at(seg_stream_idx_)->As<PointerFrame<SegmentationDesc>>();
		const SegmentationDesc& seg_desc = seg_frame.Ref();

		// If a face has been found
		//std::unique_ptr<std::vector<int>> output_ids(new std::vector<int>());
		rasterFaceSeg(landmarks, seg_desc);
		/*
		// Forward input
		input->push_back(std::shared_ptr<PointerFrame<std::vector<int>>>(
			new PointerFrame<std::vector<int>>(std::move(output_ids))));
			*/

		/// Debug ///
		std::cout << "hierarchy_frame_idx " << seg_desc.hierarchy_frame_idx() << std::endl;
		std::cout << "frame_number " << frame_number_ << std::endl;
		for (int i = 0; i < seg_desc.hierarchy_size(); ++i)
		{
			const HierarchyLevel& hierarchy = seg_desc.hierarchy(i);
			std::cout << "region_size = " << hierarchy.region_size() << std::endl;
		}
		/*
		if (seg_desc.hierarchy_size() > 0)
		{
			const HierarchyLevel& hierarchy = seg_desc.hierarchy(0);
			for (size_t i = 0; i < hierarchy.region_size(); ++i)
			{
				const CompoundRegion& r = hierarchy.region(i);
				std::cout << "Region " << r.id() << ": ";
				for (size_t j = 0; j < r.neighbor_id_size(); j++)
					std::cout << r.neighbor_id(j) << " ";
				std::cout << std::endl;
			}
		}
		*/
		/////////////

		output->push_back(input);
		++frame_number_;
	}

	bool FaceSegGlobalUnit::PostProcess(list<FrameSetPtr>* append)
	{
		return false;
	}

	void FaceSegGlobalUnit::rasterFaceSeg(const std::vector<cv::Point>& landmarks,
		const SegmentationDesc& seg_desc)
	{
		if (landmarks.size() != 68) return;

		// Create face contour
		/*
		cv::Point2f p19(landmarks[19].x, landmarks[19].y);
		//cv::Point2f p21(landmarks[21].x, landmarks[21].y);
		//cv::Point2f p22(landmarks[22].x, landmarks[22].y);
		cv::Point2f p24(landmarks[24].x, landmarks[24].y);
		cv::Point2f p27(landmarks[27].x, landmarks[27].y);
		cv::Point2f p34(landmarks[34].x, landmarks[34].y);
		cv::Point2f dir = (p27 - p34);
		cv::Point2f ptop_l = p19 + dir;
		cv::Point2f ptop_r = p24 + dir;
		std::vector<std::vector<cv::Point>> face{ {
			{ (int)landmarks[0].x, (int)landmarks[0].y },
			{ (int)landmarks[1].x, (int)landmarks[1].y },
			{ (int)landmarks[2].x, (int)landmarks[2].y },
			{ (int)landmarks[3].x, (int)landmarks[3].y },
			{ (int)landmarks[4].x, (int)landmarks[4].y },
			{ (int)landmarks[5].x, (int)landmarks[5].y },
			{ (int)landmarks[6].x, (int)landmarks[6].y },
			{ (int)landmarks[7].x, (int)landmarks[7].y },
			{ (int)landmarks[8].x, (int)landmarks[8].y },
			{ (int)landmarks[9].x, (int)landmarks[9].y },
			{ (int)landmarks[10].x, (int)landmarks[10].y },
			{ (int)landmarks[11].x, (int)landmarks[11].y },
			{ (int)landmarks[12].x, (int)landmarks[12].y },
			{ (int)landmarks[13].x, (int)landmarks[13].y },
			{ (int)landmarks[14].x, (int)landmarks[14].y },
			{ (int)landmarks[15].x, (int)landmarks[15].y },
			{ (int)landmarks[16].x, (int)landmarks[16].y },
			{ (int)landmarks[26].x, (int)landmarks[26].y },
			{ (int)ptop_r.x, (int)ptop_r.y },
			{ (int)ptop_l.x, (int)ptop_l.y },
			{ (int)landmarks[17].x, (int)landmarks[17].y }
			} };
			
		cv::Point2f p19(landmarks[19].x, landmarks[19].y);
		//cv::Point2f p21(landmarks[21].x, landmarks[21].y);
		//cv::Point2f p22(landmarks[22].x, landmarks[22].y);
		cv::Point2f p24(landmarks[24].x, landmarks[24].y);
		cv::Point2f p27(landmarks[27].x, landmarks[27].y);
		cv::Point2f p34(landmarks[34].x, landmarks[34].y);
		cv::Point2f dir = (p27 - p34);
		dir.x = -dir.x;	// Invert dx
		cv::Point2f ptop_l = p19 + dir;
		cv::Point2f ptop_r = p24 + dir;
		*/

		cv::Point dir = (landmarks[27] - landmarks[34]);
		dir.x = -dir.x;	// Invert dx

		// Jaw line
		std::vector<std::vector<cv::Point>> face{ {
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
			} };

		if (landmarks[26].x > landmarks[16].x) face.back().push_back(landmarks[26]);
		face.back().push_back(landmarks[26] + dir);
		face.back().push_back(landmarks[24] + dir);
		face.back().push_back(landmarks[19] + dir);
		face.back().push_back(landmarks[17] + dir);
		if (landmarks[17].x < landmarks[0].x) face.back().push_back(landmarks[17]);
		/*
		// Debug face map
		cv::Mat face_map = frame_.clone();
		cv::drawContours(face_map, face, 0, cv::Scalar(0, 255, 0), 1);	// CV_FILLED
		//cv::line(face_map, landmarks[26], cv::Point2f(landmarks[26]) + dir, cv::Scalar(0, 0, 255));
		//cv::line(face_map, landmarks[17], cv::Point2f(landmarks[17]) + dir, cv::Scalar(0, 0, 255));
		cv::imshow("face_map", face_map);
		cv::waitKey(0);
		*/

		// Create face map
		cv::Mat face_map = cv::Mat::zeros(frame_height_, frame_width_, CV_8U);
		cv::drawContours(face_map, face, 0, cv::Scalar(255, 255, 255), CV_FILLED);

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
			/*
			if (total_area > 0)
			{
				float ratio = float(face_area) / total_area;
				region_stats_[r.id()].ratios.push_back(ratio);
			}
			else region_stats_[r.id()].ratios.push_back(0);
			*/
			if (total_area > 0)
			{
				float ratio = float(face_area) / total_area;
				if (ratio > 0)
				{
					RegionStat& region_stat = region_stats_[r.id()];
					region_stat.ratios.push_back(ratio);
					region_stat.frame_ids.push_back(frame_number_);
				}
			}
		}
		/*
		/// Debug ///
		for each (auto region_stat in region_stats_)
		{
			std::cout << region_stat.second.ratios.size() << " ";
		}
		std::cout << std::endl;
		/////////////
		*/
	}

	std::map<int, RegionStat>& FaceSegGlobalUnit::getRegionStats()
	{
		return region_stats_;
	}

	FaceSegLocalUnit::FaceSegLocalUnit(const FaceSegLocalOptions& options,
		const std::map<int, RegionStat>& region_stats)
		: options_(options), region_stats_(region_stats)
	{
	}

	FaceSegLocalUnit::~FaceSegLocalUnit() {
	}

	bool FaceSegLocalUnit::OpenStreams(StreamSet* set)
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

		// Get segmentation stream.
		seg_stream_idx_ = FindStreamIdx(options_.segment_stream_name, set);
		if (seg_stream_idx_ < 0) {
			LOG(ERROR) << "SegmentationRenderUnit::OpenStreams: "
				<< "Could not find Segmentation stream!\n";
			return false;
		}

		// Get landmarks stream
		landmarks_stream_idx_ = FindStreamIdx(options_.landmarks_stream_name, set);
		if (landmarks_stream_idx_ < 0) {
			LOG(ERROR) << "SegmentationRenderUnit::OpenStreams: "
				<< "Could not find landmarks stream!\n";
			return false;
		}

		// Add stream.
		set->push_back(shared_ptr<DataStream>(new DataStream(options_.stream_name)));

		return true;
	}

	void FaceSegLocalUnit::ProcessFrame(FrameSetPtr input, std::list<FrameSetPtr>* output)
	{
		// Retrieve video frame
		const VideoFrame* vid_frame = input->at(video_stream_idx_)->AsPtr<VideoFrame>();
		cv::Mat frame;
		vid_frame->MatView(&frame);

		// Retrieve Segmentation.
		const PointerFrame<SegmentationDesc>& seg_frame =
			input->at(seg_stream_idx_)->As<PointerFrame<SegmentationDesc>>();
		const SegmentationDesc& seg_desc = seg_frame.Ref();

		// If it is the first frame, save it into seg_hier_ and determine overall
		// hierarchy render level.
		if (seg_hier_ == nullptr)
			seg_hier_.reset(new SegmentationDesc(seg_desc));
		else if (seg_desc.hierarchy_size() > 0)		// Update hierarchy when one present.
			*seg_hier_ = seg_desc;

		// Retrieve landmarks
		const PointerFrame<std::vector<cv::Point>>& landmarks_frame =
			input->at(landmarks_stream_idx_)->As<PointerFrame<std::vector<cv::Point>>>();

		const std::vector<cv::Point>& landmarks = landmarks_frame.Ref();

		// If a face has been found
		//std::unique_ptr<std::vector<int>> output_ids(new std::vector<int>());
		std::unique_ptr<FaceSegLocalOutput> out_data(new FaceSegLocalOutput());
		findFaceRegions(seg_desc, out_data->region_ids);
		//calcSegmentation1(frame, seg_desc, out_data->region_ids, out_data->seg);
		calcSegmentation2(frame, seg_desc, landmarks, out_data->region_ids, out_data->seg);
	
		// Forward input
		input->push_back(std::shared_ptr<PointerFrame<FaceSegLocalOutput>>(
		new PointerFrame<FaceSegLocalOutput>(std::move(out_data))));

		output->push_back(input);
		++frame_number_;
	}

	bool FaceSegLocalUnit::PostProcess(list<FrameSetPtr>* append)
	{
		return false;
	}

	void FaceSegLocalUnit::findFaceRegions(const SegmentationDesc& seg_desc, 
		std::vector<int>& output_ids)
	{
		float t = 0.5f;

		// Traverse regions.
		std::map<int, RegionStat>::const_iterator it;
		for (const auto& r : seg_desc.region())
		{
			it = region_stats_.find(r.id());
			if (it == region_stats_.end()) continue;
			/*
			const std::vector<float>& ratios = it->second.ratios;

			// Calculate average
			float avg = 0;
			for (size_t i = 0; i < ratios.size(); ++i)
				avg += ratios[i];
			avg /= ratios.size();

			// Test against threshold
			
			if (avg > t) output_ids.push_back(r.id());
			*/
			//if(it->second.max_ratio > t) output_ids.push_back(r.id());

			//
			const std::vector<float>& ratios = it->second.ratios;
			const std::vector<int>& frame_ids = it->second.frame_ids;
			int range = 10;
			int w, weight_sum = 0;
			float weighted_avg = 0;
			for (size_t i = 0; i < frame_ids.size(); ++i)
			{
				int dr = frame_ids[i] - frame_number_;
				if (dr < range) continue;
				if (dr > range) break;
				w = range + 1 - std::abs(dr);
				weight_sum += w;
				weighted_avg += w*ratios[i];
			}
			if (weight_sum > 0) weighted_avg /= (float)weight_sum;
			if (weighted_avg > t) output_ids.push_back(r.id());
		}
	}

	void FaceSegLocalUnit::calcSegmentation1(const cv::Mat& frame, 
		const SegmentationDesc& seg_desc, const std::vector<int>& region_ids,
		cv::Mat& seg)
	{
		seg = cv::Mat_<unsigned char>::zeros(frame.size());

		if (region_ids.empty()) return;

		int i = 0;

		// Traverse regions.
		for (const auto& r : seg_desc.region())
		{
			if (r.id() != region_ids[i]) continue;
			++i;
			for (const auto s : r.raster().scan_inter())
			{
				const int curr_y = s.y();
				uint8_t* out_ptr = seg.ptr<uint8_t>(curr_y) + s.left_x();
				for (int j = 0, len = s.right_x() - s.left_x() + 1; j < len; ++j, ++out_ptr)
					*out_ptr = 255;
			}

			if (i >= region_ids.size()) break;
		}
	}

	void FaceSegLocalUnit::calcSegmentation2(const cv::Mat& frame,
		const SegmentationDesc& seg_desc, const std::vector<cv::Point>& landmarks,
		const std::vector<int>& region_ids, cv::Mat& seg)
	{
		seg = cv::Mat_<unsigned char>::zeros(frame.size());
		if (landmarks.empty()) return;
	
		const VectorMesh& mesh = seg_desc.vector_mesh();

		const float pi = std::acos(-1);
		const float cos_a_t = std::cos(pi / 3);
		const float out_t = 0.1f;

		// Create face map
		std::vector<std::vector<cv::Point>> face(1);
		createFullFace(landmarks, face.back());
		cv::Mat face_map = cv::Mat::zeros(frame_height_, frame_width_, CV_8U);

		// Draw jaw
		float jaw_width = cv::norm(landmarks[2] - landmarks[14]);
		float side = getFaceDominantSide(landmarks);
		std::cout << "side = " << side << std::endl;//
		const float min_side = 0.3f, max_side = 0.7f;
		side = std::max(std::min(side, max_side), min_side);
		side = (side - min_side) / (max_side - min_side);
		float right_ratio = std::max(side - 0.5f, 0.0f)*2.0f;
		float left_ratio = std::min(side, 0.5f)*2.0f;
		std::cout << "right_ratio = " << right_ratio << std::endl;//
		std::cout << "left_ratio = " << left_ratio << std::endl;//
		int right_ind = 3 + (int)std::round(right_ratio * 5);
		int left_ind = 8 + (int)std::round(left_ratio * 5);
		int jaw_thickness = (int)std::round(0.1f*jaw_width);
		for (size_t i = right_ind + 1; i <= left_ind; ++i)
			cv::line(face_map, landmarks[i], landmarks[i - 1],
				cv::Scalar(128, 128, 128), jaw_thickness);

		// Draw face
		cv::drawContours(face_map, face, 0, cv::Scalar(255, 255, 255), CV_FILLED);	

		// Calculate total jaw area
		unsigned char *face_map_data = face_map.data;
		unsigned int total_jaw_area = 0;
		for (size_t i = 0; i < face_map.total(); ++i)
			if (*face_map_data++ == 128) ++total_jaw_area;

		// Calculate total face area
		face_map_data = face_map.data;
		unsigned int total_face_area = 0;
		for (size_t i = 0; i < face_map.total(); ++i)
			if (*face_map_data++ == 255) ++total_face_area;

		/// Debug face map ///
		cv::imshow("face_map", face_map);
		cv::waitKey(1);
		//////////////////////

		// For each region
		cv::Mat poly_map = cv::Mat::zeros(frame_height_, frame_width_, CV_8U);
		for (const auto& r : seg_desc.region())
		{
			if (r.vectorization().polygon().empty()) continue;

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
					cv::drawContours(poly_map, contours, 0, cv::Scalar(255, 255, 255), CV_FILLED);

					// Remove holes
					cv::drawContours(poly_map, holes, 0, cv::Scalar(0, 0, 0), CV_FILLED);

					/// Debug ///
					//cv::imshow("poly_map", poly_map);
					//cv::waitKey(1);
					/////////////

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

						cv::Point2f face_dir = landmarks[8] - landmarks[27];
						cv::Point2f poly_dir = poly_center - cv::Point2f(landmarks[27]);
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

						//if (in_ratio > 0.2f && out_ratio > 0.2f && cos_a > cos_a_t)
						//if (in_ratio > 0.1f && out_ratio > 0.1f && (jaw_ratio > 0.05f && jaw_poly_ratio < 0.5f))
						if (in_face_ratio > 0.01f && out_face_ratio > 0.01f && (jaw_ratio > 0.05f && jaw_poly_ratio > 0.02f && jaw_poly_ratio < 0.5f))
						{
							// Found neck region
							//cv::drawContours(seg, contours, 0, cv::Scalar(128, 128, 128), CV_FILLED);

							//std::cout << "cos_a = " << cos_a << ", in_ratio = " << in_ratio << ", out_ratio = " << out_ratio << std::endl;
//							std::cout << "jaw_ratio = " << jaw_ratio << ", in_ratio = " << in_ratio << ", out_ratio = " << out_ratio << std::endl;
							//std::cout << "Found neck!" << std::endl;

							// Fill intersection
							unsigned char *face_map_data = face_map.data, *poly_map_data = poly_map.data;
							unsigned char* seg_data = seg.data;
							int pr, pc;
							for (pr = 0; pr < face_map.rows; ++pr)
							{
								for (pc = 0; pc < face_map.cols; ++pc)
								{
									if (*poly_map_data++ > 0)
									{
										if (*face_map_data++ == 255)
											*seg_data = 128;
									}
									else ++face_map_data;
									++seg_data;
								}
							}
						}
						else if (in_ratio > 0.5f)
							cv::drawContours(seg, contours, 0, cv::Scalar(255, 255, 255), CV_FILLED);

						/// Debug ///
						if (in_face_ratio > 0.01f && out_face_ratio > 0.01f && (jaw_ratio > 0.05f && jaw_poly_ratio > 0.02f && jaw_poly_ratio < 0.5f))
						{
							//std::cout << "jaw_ratio = " << jaw_ratio << ", jaw_poly_ratio = " << jaw_poly_ratio << ", in_ratio = " << in_ratio << ", out_ratio = " << out_ratio << std::endl;
							std::cout << "jaw_ratio = " << jaw_ratio << ", jaw_poly_ratio = " << jaw_poly_ratio << ", in_face_ratio = " << in_face_ratio << ", out_face_ratio = " << out_face_ratio << std::endl;
							/*
							const CompoundRegion& cr = GetCompoundRegionFromId(r.id(), seg_hier_->hierarchy(0));
							std::cout << "id = " << r.id() << std::endl;
							std::cout << "parent_id = " << cr.parent_id() << std::endl;
							std::cout << "child_ids = ";
							for (int child_id : cr.child_id())
								std::cout << cr.parent_id() << " ";
							std::cout << std::endl;
							*/
							/*
							std::cout << "cos_a = " << cos_a << ", in_ratio = " << in_ratio << ", out_ratio = " << out_ratio << std::endl;
							cv::imshow("seg", seg);
							cv::waitKey(0);
							*/
						}
						/////////////
					}

					/*
					for (size_t i = 0; i < face_map.total(); ++i)
					{
						if (*poly_map_data++ > 0)
						{
							++total_area;
							face_area += (unsigned int)(*face_map_data++ > 0);
						}
						else ++face_map_data;
					}
					*/
					/*
					// Test against threshold
					if (total_area > 0)
					{
						float ratio = float(face_area) / total_area;
						if(ratio > 0.5f)
							cv::drawContours(seg, contours, 0, cv::Scalar(255, 255, 255), CV_FILLED);
					}
					*/

					// Clear map
					cv::drawContours(poly_map, contours, 0, cv::Scalar(0, 0, 0), CV_FILLED);
				}
				
			}
		}
		/*
		/// Debug ///
		cv::imshow("seg", seg);
		cv::waitKey(1);
		/////////////
		*/
	}

	FaceSegmentationRendererUnit::FaceSegmentationRendererUnit(
		const FaceSegmentationRendererOptions& options,
		const std::map<int, RegionStat>& region_stats)
		: options_(options), region_stats_(region_stats),
		frame_counter_(0)
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

		frame_width_ = vid_stream.frame_width();
		frame_height_ = vid_stream.frame_height();
		frame_width_step_ = vid_stream.width_step();
		fps_ = vid_stream.fps();
		
		// Get face segmentation stream.
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

		// Get landmarks stream
		landmarks_stream_idx_ = FindStreamIdx(options_.landmarks_stream_name, set);
		if (landmarks_stream_idx_ < 0) {
			LOG(ERROR) << "SegmentationRenderUnit::OpenStreams: "
				<< "Could not find landmarks stream!\n";
			return false;
		}

		// Add video output stream.
		VideoStream* rendered_stream = new VideoStream(frame_width_,
			frame_height_,
			frame_width_step_,
			fps_,
			PIXEL_FORMAT_BGR24,
			options_.stream_name);

		set->push_back(shared_ptr<DataStream>(rendered_stream));

		/*
		// Add stream.
		DataStream* landmarks_stream = new DataStream(options_.stream_name);
		set->push_back(shared_ptr<DataStream>(landmarks_stream));
		*/

		return true;
	}

	void FaceSegmentationRendererUnit::ProcessFrame(FrameSetPtr input, std::list<FrameSetPtr>* output)
	{
		// Retrieve video frame
		const VideoFrame* frame = input->at(video_stream_idx_)->AsPtr<VideoFrame>();
		cv::Mat in_frame;
		frame->MatView(&in_frame);
		int64_t pts = frame->pts();

		// Retrieve Segmentation.
		const PointerFrame<SegmentationDesc>& seg_frame =
			input->at(seg_stream_idx_)->As<PointerFrame<SegmentationDesc>>();

		const SegmentationDesc& seg_desc = seg_frame.Ref();

		if (seg_desc.hierarchy_size() > 0)
			m_hierarchy = seg_desc.hierarchy();

		// Retrieve local face segmentation data
		const FaceSegLocalOutput& face_seg_data =
			static_cast<PointerFrame<FaceSegLocalOutput>*>(input->at(face_seg_stream_idx_).get())->Ref();

		// Retrieve landmarks
		const PointerFrame<std::vector<cv::Point>>& landmarks_frame =
			input->at(landmarks_stream_idx_)->As<PointerFrame<std::vector<cv::Point>>>();

		const std::vector<cv::Point>& landmarks = landmarks_frame.Ref();

		// Allocate new output frame.
        VideoFrame* render_frame = nullptr;
        if (options_.debug) render_frame = new VideoFrame(frame_width_, frame_height_,
                3, 0, pts);
        else render_frame = new VideoFrame(frame_width_, frame_height_,
            1, 0, pts);

		cv::Mat out_frame;
		render_frame->MatView(&out_frame);

		// Render
		if (options_.debug)
		{
			in_frame.copyTo(out_frame);
			//renderRegions(out_frame, seg_desc);
			
			renderSegmentation(out_frame, face_seg_data.seg);
			renderBoundaries(out_frame, seg_desc);
            std::vector<std::vector<cv::Point>> full_face(1);
            createFullFace(landmarks, full_face.back());
            cv::drawContours(out_frame, full_face, 0, cv::Scalar(47, 255, 173), 1);
			sfl::render(out_frame, landmarks);
			/*
			// Show overlay
			std::string msg = "Frame count: " + std::to_string(frame_counter_);
			cv::putText(out_frame, msg, cv::Point(15, 15),
				cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 102, 255), 1, CV_AA);
                */
		}
		else
		{
			// Render segmentation by class id (15)
            renderSegmentation(out_frame, face_seg_data.seg, 15);
			//renderSegmentation(out_frame, face_seg_data.seg, cv::Scalar(128, 128, 192));
		}

		// renderSelectedRegions(image, seg_desc, face_seg_data.region_ids);

		// Forward input
		input->push_back(std::shared_ptr<DataFrame>(render_frame));
		output->push_back(input);
		++frame_counter_;
	}

	bool FaceSegmentationRendererUnit::PostProcess(list<FrameSetPtr>* append)
	{
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

	void FaceSegmentationRendererUnit::renderSegmentation(cv::Mat& frame, const cv::Mat& seg)
	{
		int r, c;
		const float a = 0.5f;
		cv::Point3_<uchar>* frame_data = (cv::Point3_<uchar>*)frame.data;
		unsigned char* seg_data = seg.data;
		unsigned char s;
		for (r = 0; r < frame.rows; ++r)
		{
			for (c = 0; c < frame.cols; ++c)
			{
				//a = *seg_data++ / 255.0f;
				s = *seg_data++;
				if (s == 255)
				{
					frame_data->x = (unsigned char)std::round(0 * a + frame_data->x*(1 - a));
					frame_data->y = (unsigned char)std::round(0 * a + frame_data->y*(1 - a));
					frame_data->z = (unsigned char)std::round(255 * a + frame_data->z*(1 - a));
				}
				else if (s == 128)
				{
					frame_data->x = (unsigned char)std::round(255 * a + frame_data->x*(1 - a));
					frame_data->y = (unsigned char)std::round(0 * a + frame_data->y*(1 - a));
					frame_data->z = (unsigned char)std::round(0 * a + frame_data->z*(1 - a));
				}
				++frame_data;
			}
		}
	}

	void FaceSegmentationRendererUnit::renderSegmentation(cv::Mat& frame, const cv::Mat& seg,
		const cv::Scalar& color)
	{
		int r, c;
		cv::Point3_<uchar>* frame_data = (cv::Point3_<uchar>*)frame.data;
		const unsigned char* seg_data = seg.data;
		cv::Point3_<uchar> bgr((uchar)color[0], (uchar)color[1], (uchar)color[2]);
		for (r = 0; r < frame.rows; ++r)
		{
			seg_data = seg.ptr<uchar>(r);
			frame_data = frame.ptr<cv::Point3_<uchar>>(r);
			for (c = 0; c < frame.cols; ++c)
			{
				//a = *seg_data++ / 255.0f;
				if (*seg_data++ > 0) *frame_data = bgr;
				++frame_data;
			}
		}
	}

    void FaceSegmentationRendererUnit::renderSegmentation(cv::Mat& frame, const cv::Mat& seg,
        uchar color)
    {
        int r, c;
        unsigned char* frame_data = frame.data;
        const unsigned char* seg_data = seg.data;
        for (r = 0; r < frame.rows; ++r)
        {
            seg_data = seg.ptr<uchar>(r);
            frame_data = frame.ptr<uchar>(r);
            for (c = 0; c < frame.cols; ++c)
            {
                if (*seg_data++ > 0) *frame_data = color;
                ++frame_data;
            }
        }
    }

	void FaceSegmentationRendererUnit::renderBoundaries(cv::Mat& frame,
		const SegmentationDesc& seg_desc)
	{
		cv::Mat tmp(frame.size(), frame.type());
		RenderRegions(false, false, seg_desc,
			HierarchyColorGenerator(0, 3, &seg_desc.hierarchy()),
			&tmp);

		// Edge highlight post-process.
		const int height = frame.rows;
		const int width = frame.cols;
		const int width_step = frame.step[0];
		const int channels = frame.channels();
		for (int i = 0; i < height - 1; ++i) {
			uint8_t* row_ptr = tmp.ptr<uint8_t>(i);
			uint8_t* out_row_ptr = frame.ptr<uint8_t>(i);
			for (int j = 0; j < width - 1; ++j, row_ptr += channels, out_row_ptr += channels)
			{
				if (ColorDiff_L1(row_ptr, row_ptr + channels) != 0 ||
					ColorDiff_L1(row_ptr, row_ptr + width_step) != 0)
					out_row_ptr[0] = out_row_ptr[1] = out_row_ptr[2] = 255;
			}

			// Last column.
			if (ColorDiff_L1(row_ptr, row_ptr + width_step) != 0)
				out_row_ptr[0] = out_row_ptr[1] = out_row_ptr[2] = 255;
		}

		// Last row.
		uint8_t* row_ptr = tmp.ptr<uint8_t>(height - 1);
		uint8_t* out_row_ptr = frame.ptr<uint8_t>(height - 1);
		for (int j = 0; j < width - 1; ++j, row_ptr += channels) {
			if (ColorDiff_L1(row_ptr, row_ptr + channels) != 0)
				out_row_ptr[0] = out_row_ptr[1] = out_row_ptr[2] = 255;
		}
	}

	void FaceSegmentationRendererUnit::renderRegions(cv::Mat& frame, const SegmentationDesc& seg_desc)
	{
		HierarchyColorGenerator generator(0, frame.channels(), &seg_desc.hierarchy());
		std::vector<uint8_t> color(frame.channels());

		// For each region
		const VectorMesh& mesh = seg_desc.vector_mesh();
		for (const auto& r : seg_desc.region())
		{
			if (r.vectorization().polygon().empty()) continue;	

			// For each polygon
			for (const auto& poly : r.vectorization().polygon())
			{
				std::vector<std::vector<cv::Point>> contours;

				// Get color.
				RegionID mapped_id;
				if (!generator(r.id(), &mapped_id, &color[0])) {
					continue;
				}

				if (poly.coord_idx_size() == 0) continue;
				if (poly.hole())
				{
					// Create hole
					//std::cout << "Found hole!" << std::endl;
                    createContours(mesh, poly, contours);
					color[0] = color[1] = color[2];
				}
				else
				{
                    createContours(mesh, poly, contours);
				}

				// Render polygon
				if(!contours.empty())
					cv::drawContours(frame, contours, 0, cv::Scalar(color[0], color[1], color[2]), CV_FILLED);

				/// Debug ///
				cv::imshow("renderRegions", frame);
				cv::waitKey(0);
				/////////////
			}
		}
	}

	

	void FaceSegmentationRendererUnit::calcRegionCenters(const VectorMesh& mesh,
		const SegmentationDesc_Region2D& r, std::vector<cv::Point2f>& centers)
	{
		const SegmentationDesc_ShapeMoments& shape_moment = r.shape_moments();
		centers.push_back(cv::Point2f(shape_moment.mean_x(), shape_moment.mean_y()));
		/*
		if (r.vectorization().polygon().empty()) return;

		// For each polygon
		for (const auto& poly : r.vectorization().polygon())
		{
			if (poly.coord_idx_size() == 0) continue;
			if (poly.hole()) continue;
			
			cv::Point2f center(0.0f, 0.0f);

			// For each coordinate
			int coord_num = poly.coord_idx_size() - 2;
			for (int c = 0; c < coord_num; ++c)
			{
				int idx = poly.coord_idx(c);
				center.x += mesh.coord(idx);
				center.y += mesh.coord(idx + 1);
			}

			center /= coord_num;
			centers.push_back(center);
		}
		*/
	}

	void FaceSegmentationRendererUnit::renderRegionIds(cv::Mat& img, const SegmentationDesc& seg_desc,
		const cv::Scalar& color)
	{
		const VectorMesh& mesh = seg_desc.vector_mesh();
		cv::Scalar textColor(0, 255, 0);
		std::map<int, RegionStat>::const_iterator it;
		for (const auto& r : seg_desc.region())
		{
			it = region_stats_.find(r.id());
			if (it == region_stats_.end()) continue;
			if (it->second.max_ratio < 0.01f) continue;

			std::vector<cv::Point2f> centers;
			calcRegionCenters(mesh, r, centers);
			for (cv::Point2f c : centers)
				cv::putText(img, std::to_string(r.id()), c, cv::FONT_HERSHEY_PLAIN, 0.5, textColor, 1.0);
		}
	}

}  // namespace fvs
