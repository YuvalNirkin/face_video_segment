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

#include "video_writer_unit2.h"

#include "base/base_impl.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace video_framework;

namespace segmentation
{
	VideoWriterUnit2::VideoWriterUnit2(const VideoWriter2Options& options,
		const std::string& video_file)
		: options_(options), video_file_(video_file)
	{
	}

	VideoWriterUnit2::~VideoWriterUnit2() {
	}

	bool VideoWriterUnit2::OpenStreams(StreamSet* set) {
		// Find video stream idx.
		video_stream_idx_ = FindStreamIdx(options_.video_stream_name, set);

		if (video_stream_idx_ < 0) {
			LOG(ERROR) << "Could not find Video stream!\n";
			return false;
		}

		const VideoStream& vid_stream = set->at(video_stream_idx_)->As<VideoStream>();

		const int frame_width = vid_stream.frame_width();
		const int frame_height = vid_stream.frame_height();
		const float fps = vid_stream.fps();

		writer_.open(video_file_, CV_FOURCC('F', 'M', 'P', '4'), fps, cv::Size(frame_width, frame_height));

		if(!writer_.isOpened())
			LOG(ERROR) << "Could not open video writer!\n";

		// Add stream.
		//DataStream* landmarks_stream = new DataStream(options_.stream_name);
		//set->push_back(shared_ptr<DataStream>(landmarks_stream));

		return true;
	}

	void VideoWriterUnit2::ProcessFrame(FrameSetPtr input, std::list<FrameSetPtr>* output)
	{
		// Retrieve video frame
		const VideoFrame* frame = input->at(video_stream_idx_)->AsPtr<VideoFrame>();
		cv::Mat image;
		frame->MatView(&image);

		writer_.write(image);

		// Forward input
		output->push_back(input);
	}

	bool VideoWriterUnit2::PostProcess(list<FrameSetPtr>* append)
	{
		return false;
	}

}  // namespace video_framework.
