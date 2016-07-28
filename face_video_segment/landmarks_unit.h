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

#ifndef FACE_VIDEO_SEGMENT_VIDEO_LANDMARKS_UNIT_H__
#define FACE_VIDEO_SEGMENT_VIDEO_LANDMARKS_UNIT_H__

#include "base/base.h"
#include "video_framework/video_unit.h"
#include <sfl/sequence_face_landmarks.h>
#include <sfl/utilities.h>
#include <opencv2/core.hpp>

namespace segmentation {

	struct LandmarksOptions {
		std::string stream_name = "LandmarksStream";
		std::string video_stream_name = "VideoStream";
		std::string landmarks_path = "";
		float frame_scale = 1.0f;
		bool track_faces = true;
	};

	class LandmarksUnit : public video_framework::VideoUnit {
	public:
		LandmarksUnit(const LandmarksOptions& options);
		~LandmarksUnit();

		LandmarksUnit(const LandmarksUnit&) = delete;
		LandmarksUnit& operator=(const LandmarksUnit&) = delete;

		virtual bool OpenStreams(video_framework::StreamSet* set);
		virtual void ProcessFrame(video_framework::FrameSetPtr input, std::list<video_framework::FrameSetPtr>* output);
		virtual bool PostProcess(std::list<video_framework::FrameSetPtr>* append);

	private:
		LandmarksOptions options_;
		std::shared_ptr<sfl::SequenceFaceLandmarks> sfl_;
		std::list<std::unique_ptr<sfl::Frame>>::const_iterator sequence_it;

		int video_stream_idx_;
		int frame_width_;
		int frame_height_;
		int main_face_id_;
	};

	/*
	class LandmarksUnit : public video_framework::VideoUnit {
	public:
		static std::shared_ptr<LandmarksUnit> create(const LandmarksOptions& options);
	};
	*/

	struct LandmarksRendererOptions {
		//std::string stream_name = "LandmarksRendererStream";
		std::string video_stream_name = "VideoStream";
		std::string landmarks_stream_name = "LandmarksStream";
	};

	class LandmarksRendererUnit : public video_framework::VideoUnit
	{
	public:
		LandmarksRendererUnit(const LandmarksRendererOptions& options);
		~LandmarksRendererUnit();

		LandmarksRendererUnit(const LandmarksRendererUnit&) = delete;
		LandmarksRendererUnit& operator=(const LandmarksRendererUnit&) = delete;

		virtual bool OpenStreams(video_framework::StreamSet* set);
		virtual void ProcessFrame(video_framework::FrameSetPtr input, std::list<video_framework::FrameSetPtr>* output);
		virtual bool PostProcess(std::list<video_framework::FrameSetPtr>* append);

	private:
		LandmarksRendererOptions options_;
		int video_stream_idx_;
		int landmarks_stream_idx_;

		int frame_width;
		int frame_height;
		int frame_num_ = 0;

	};

}  // namespace segmentation

#endif  // FACE_VIDEO_SEGMENT_VIDEO_LANDMARKS_UNIT_H__
