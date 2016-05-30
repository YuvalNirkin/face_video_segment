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

#include "FaceViewer.h"
#include "landmarks_unit.h"
#include <video_framework/video_reader_unit.h>
#include <video_framework/video_display_unit.h>
#include <segmentation/segmentation_unit.h>

using namespace video_framework;

namespace segmentation
{
	FaceView::FaceView(const std::string& video_file, const std::string& seg_file,
		const std::string& landmarks_model_file) :
		m_video_file(video_file), m_seg_file(seg_file),
		m_landmarks_model_file(landmarks_model_file)
	{
	}

	void FaceView::run()
	{
		VideoReaderUnit reader(VideoReaderOptions(), m_video_file);

		SegmentationReaderUnitOptions segOptions;
		segOptions.filename = m_seg_file;
		SegmentationReaderUnit segReader(segOptions);
		segReader.AttachTo(&reader);

		//VideoDisplayUnit display((VideoDisplayOptions()));
		//display.AttachTo(&reader);

		LandmarksOptions landmarksOptions;
		landmarksOptions.landmarks_model_file = m_landmarks_model_file;
		std::shared_ptr<LandmarksUnit> display = LandmarksUnit::create(landmarksOptions);
		display->AttachTo(&segReader);

		if (!reader.PrepareProcessing())
			throw std::runtime_error("Video framework setup failed.");

		// Run with rate limitation.
		RatePolicy rate_policy;
		// Speed up playback for fun :)
		rate_policy.max_rate = 45;

		// This call will block and return when the whole has been displayed.
		if (!reader.RunRateLimited(rate_policy))
			throw std::runtime_error("Could not process video file.");
	}
}