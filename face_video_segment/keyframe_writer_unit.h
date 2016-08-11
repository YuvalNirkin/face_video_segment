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

#ifndef FACE_VIDEO_SEGMENT_KEYFRAME_WRITER_UNIT_H__
#define FACE_VIDEO_SEGMENT_KEYFRAME_WRITER_UNIT_H__

#include "base/base.h"
#include "video_framework/video_unit.h"

#include <opencv2/core.hpp>

namespace segmentation {

	struct KeyframeWriterOptions {
		std::string video_stream_name = "VideoStream";
		std::string segment_stream_name = "SegmentationStream";
		std::string landmarks_stream_name = "LandmarksStream";
        std::string face_segment_stream_name = "FaceSegmentationStream";
		std::string face_segment_renderer_stream_name = "FaceSegmentationRendererStream";
		int start_frame = 10;
        int stability_range = 5;
        bool debug = false;
	};

	struct Keyframe
	{
		int id;
		cv::Point3f euler_angles;
	};

	class KeyframeWriter : public video_framework::VideoUnit {
	public:
	public:
		KeyframeWriter(const KeyframeWriterOptions& options,
			const std::string& output_dir, const std::string& src_name);
		~KeyframeWriter();

		KeyframeWriter(const KeyframeWriter&) = delete;
		KeyframeWriter& operator=(const KeyframeWriter&) = delete;

		virtual bool OpenStreams(video_framework::StreamSet* set);
		virtual void ProcessFrame(video_framework::FrameSetPtr input, std::list<video_framework::FrameSetPtr>* output);
		virtual bool PostProcess(std::list<video_framework::FrameSetPtr>* append);

    private:
        void renderSegmentation(cv::Mat& frame, const cv::Mat& seg);

	private:
		KeyframeWriterOptions options_;
		std::string output_dir_;
		std::string src_name_;

		int video_stream_idx_;
		int landmarks_stream_idx_;
        int face_seg_stream_idx_;
		int face_segment_renderer_stream_idx_;

		int frame_number_ = 0;

		std::list<Keyframe> keyframes_;
        std::list<std::vector<cv::Point>> history_;
	};

}  // namespace segmentation

#endif  // FACE_VIDEO_SEGMENT_KEYFRAME_WRITER_UNIT_H__
