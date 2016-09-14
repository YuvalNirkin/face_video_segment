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

#include "landmarks_unit.h"
#include "face_segmentation_unit.h"
#include <sfl/sequence_face_landmarks.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>

#include "base/base_impl.h"
#include <segmentation/segmentation.h>
/*
// dlib
#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/shape_predictor.h>
*/

using namespace video_framework;

namespace segmentation
{
	LandmarksUnit::LandmarksUnit(const LandmarksOptions& options)
		: options_(options), main_face_id_(0)
	{
		sfl_ = sfl::SequenceFaceLandmarks::create(options.landmarks_path,
			options.frame_scale, options.tracking);
		if (sfl_->size() > 0)
		{
			const std::list<std::unique_ptr<sfl::Frame>>& sequence = sfl_->getSequence();
			sequence_it = sequence.begin();
			main_face_id_ = sfl::getMainFaceID(sequence);
		}
	}

	LandmarksUnit::~LandmarksUnit() {
	}

	bool LandmarksUnit::OpenStreams(StreamSet* set) {
		// Find video stream idx.
		video_stream_idx_ = FindStreamIdx(options_.video_stream_name, set);

		if (video_stream_idx_ < 0) {
			LOG(ERROR) << "LandmarksUnit::OpenStreams: Could not find Video stream!\n";
			return false;
		}

		const VideoStream& vid_stream = set->at(video_stream_idx_)->As<VideoStream>();
		frame_width_ = vid_stream.frame_width();
		frame_height_ = vid_stream.frame_height();

		// Add stream.
		DataStream* landmarks_stream = new DataStream(options_.stream_name);
		set->push_back(shared_ptr<DataStream>(landmarks_stream));

		return true;
	}

	void LandmarksUnit::ProcessFrame(FrameSetPtr input, std::list<FrameSetPtr>* output)
	{
		// Retrieve video frame
		const VideoFrame* frame = input->at(video_stream_idx_)->AsPtr<VideoFrame>();
		cv::Mat image;
		frame->MatView(&image);

		const sfl::Frame* sfl_frame = nullptr;
		if (sequence_it != sfl_->getSequence().end())
		{
			// Calculate landmarks
			if (sfl_->getModel().empty())
                sfl_frame = (*sequence_it++).get();
			else sfl_frame = &sfl_->addFrame(image);
		}
		
		// Forward input
        input->push_back(std::shared_ptr<ValueFrame<const sfl::Frame*>>(
            new ValueFrame<const sfl::Frame*>(sfl_frame)));
//		input->push_back(std::shared_ptr<PointerFrame<std::vector<cv::Point>>>(
//			new PointerFrame<std::vector<cv::Point>>(std::move(landmarks_out))));

		output->push_back(input);
	}

	bool LandmarksUnit::PostProcess(list<FrameSetPtr>* append)
	{
		return false;
	}

	LandmarksRendererUnit::LandmarksRendererUnit(const LandmarksRendererOptions& options)
		: options_(options)
	{
	}

	LandmarksRendererUnit::~LandmarksRendererUnit() {
	}

	bool LandmarksRendererUnit::OpenStreams(StreamSet* set)
	{
		// Find video stream idx.
		video_stream_idx_ = FindStreamIdx(options_.video_stream_name, set);

		if (video_stream_idx_ < 0) {
			LOG(ERROR) << "LandmarksRendererUnit::OpenStreams: "
				<< "Could not find Video stream!\n";
			return false;
		}

		const VideoStream& vid_stream = set->at(video_stream_idx_)->As<VideoStream>();

		frame_width = vid_stream.frame_width();
		frame_height = vid_stream.frame_height();

		// Get landmarks stream
		landmarks_stream_idx_ = FindStreamIdx(options_.landmarks_stream_name, set);
		if (landmarks_stream_idx_ < 0) {
			LOG(ERROR) << "LandmarksRendererUnit::OpenStreams: "
				<< "Could not find landmarks stream!\n";
			return false;
		}

		// Add stream.
		//DataStream* landmarks_stream = new DataStream(options_.stream_name);
		//set->push_back(shared_ptr<DataStream>(landmarks_stream));

		return true;
	}

	void LandmarksRendererUnit::ProcessFrame(FrameSetPtr input, std::list<FrameSetPtr>* output)
	{
		// Retrieve video frame
		++frame_num_;
		const VideoFrame* frame = input->at(video_stream_idx_)->AsPtr<VideoFrame>();
		cv::Mat image;
		frame->MatView(&image);

		// Retrieve landmarks
		const PointerFrame<std::vector<cv::Point>>& landmarks_frame =
			input->at(landmarks_stream_idx_)->As<PointerFrame<std::vector<cv::Point>>>();

		const std::vector<cv::Point>& landmarks = landmarks_frame.Ref();

		// Render
		sfl::render(image, landmarks);

		// Show overlay
		std::string msg = "Frame count: " + std::to_string(frame_num_);
		cv::putText(image, msg, cv::Point(15, 15),
			cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 102, 255), 1, CV_AA);

		// Forward input
		/*input->push_back(std::shared_ptr<PointerFrame<ring_t>>(
		new PointerFrame<ring_t>(std::move(landmarks_ring))));*/

		output->push_back(input);
	}

	bool LandmarksRendererUnit::PostProcess(list<FrameSetPtr>* append)
	{
		return false;
	}

}  // namespace video_framework.
