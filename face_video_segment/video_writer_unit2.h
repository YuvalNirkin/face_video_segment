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

#ifndef FACE_VIDEO_SEGMENT_VIDEO_WRITER_UNIT_2_H__
#define FACE_VIDEO_SEGMENT_VIDEO_WRITER_UNIT_2_H__

#include "base/base.h"
#include "video_framework/video_unit.h"

#include <opencv2/videoio.hpp>

namespace fvs 
{

	struct VideoWriter2Options {
		// Codec settings.
		// Bit-rate default is 2000 kbps
		int bit_rate = 2000000;

		// Use video streams fps as default.
		// TODO: will be ignored, fix!
		float fps = 0;

		// Scale factor of output. Overrides all other scale settings.
		float scale = 1.0f;

		// width and height will be rounded to multiple of fraction.
		int fraction = 4;

		// If > 0, maximum dimension is scaled to fit that value. Can not both be set.
		int scale_max_dim = 0;
		int scale_min_dim = 0;

		// Guess format from format std::string as default.
		// Call "ffmpeg -formats" to get a std::list of valid formats.
		std::string output_format;

		// Output stream name.
		std::string stream_name = "VideoStream";
	};

    /** @brief Writes video frames from stream to file.
    */
	class VideoWriterUnit2 : public video_framework::VideoUnit {
	public:
	public:
		VideoWriterUnit2(const VideoWriter2Options& options,
			const std::string& video_file);
		~VideoWriterUnit2();

		VideoWriterUnit2(const VideoWriterUnit2&) = delete;
		VideoWriterUnit2& operator=(const VideoWriterUnit2&) = delete;

		virtual bool OpenStreams(video_framework::StreamSet* set);
		virtual void ProcessFrame(video_framework::FrameSetPtr input, std::list<video_framework::FrameSetPtr>* output);
		virtual bool PostProcess(std::list<video_framework::FrameSetPtr>* append);

	private:
		VideoWriter2Options options_;
		std::string video_file_;

		int video_stream_idx_;

		cv::VideoWriter writer_;
	};

}  // namespace segmentation

#endif  // FACE_VIDEO_SEGMENT_VIDEO_WRITER_UNIT_2_H__
