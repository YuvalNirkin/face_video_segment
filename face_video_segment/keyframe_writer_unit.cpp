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

#include "keyframe_writer_unit.h"
#include "face_segmentation_unit.h"
#include "utilities.h"

#include "base/base_impl.h"

#include <sfl/utilities.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/format.hpp>

using namespace video_framework;

namespace fvs
{
	KeyframeWriter::KeyframeWriter(const KeyframeWriterOptions& options,
		const std::string& output_dir, const std::string& src_name)
		: options_(options), output_dir_(output_dir), src_name_(src_name)
	{
        keyframer_ = std::make_unique<Keyframer>(
            options.start_frame, options.stability_range);
	}

	KeyframeWriter::~KeyframeWriter() {
	}

	bool KeyframeWriter::OpenStreams(StreamSet* set) {
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

		// Get landmarks stream
		landmarks_stream_idx_ = FindStreamIdx(options_.landmarks_stream_name, set);
		if (landmarks_stream_idx_ < 0) {
			LOG(ERROR) << "SegmentationRenderUnit::OpenStreams: "
				<< "Could not find landmarks stream!\n";
			return false;
		}

        // Get face segmentation stream.
        face_seg_stream_idx_ = FindStreamIdx(options_.face_segment_stream_name, set);
        if (face_seg_stream_idx_ < 0) {
            LOG(ERROR) << "SegmentationRenderUnit::OpenStreams: "
                << "Could not find face segmentation stream!\n";
            return false;
        }

		// Find face segmentation renderer stream idx.
		face_segment_renderer_stream_idx_ = FindStreamIdx(options_.face_segment_renderer_stream_name, set);

		if (face_segment_renderer_stream_idx_ < 0) {
			LOG(ERROR) << "Could not find face segmentation renderer stream!\n";
			return false;
		}

		return true;
	}

	void KeyframeWriter::ProcessFrame(FrameSetPtr input, std::list<FrameSetPtr>* output)
	{
		// Retrieve video frame
		const VideoFrame* vid_frame = input->at(video_stream_idx_)->AsPtr<VideoFrame>();
		cv::Mat frame;
		vid_frame->MatView(&frame);

		// Retrieve landmarks
		const PointerFrame<std::vector<cv::Point>>& landmarks_frame =
			input->at(landmarks_stream_idx_)->As<PointerFrame<std::vector<cv::Point>>>();

		const std::vector<cv::Point>& landmarks = landmarks_frame.Ref();

        // Retrieve local face segmentation data
        const FaceSegLocalOutput& face_seg_data =
            static_cast<PointerFrame<FaceSegLocalOutput>*>(input->at(face_seg_stream_idx_).get())->Ref();

		// Retrieve face segmentation renderer frame
		const VideoFrame* seg_frame = input->at(face_segment_renderer_stream_idx_)->AsPtr<VideoFrame>();
		cv::Mat seg_render;
		seg_frame->MatView(&seg_render);

		// Process frame
        if (keyframer_->addFrame(landmarks))
        {
            // Crop frame and segmentation
            cv::Rect bbox = sfl::getFaceBBoxFromLandmarks(landmarks, frame.size(), true);
            cv::Mat frame_cropped = frame(bbox);
            cv::Mat seg_render_cropped = seg_render(bbox);
            cv::Mat seg, seg_cropped;
            if (options_.debug)
            {
                seg = frame.clone();
                cv::Scalar red(0, 0, 255);
                renderSegmentationBlend(seg, face_seg_data.seg, 0.5f, red, red);
                seg_cropped = seg(bbox);
            }

            // Limit resolution
            cv::MatSize size = frame_cropped.size;
            int max_index = std::distance(size.p, std::max_element(size.p, size.p + 2));
            if (size[max_index] > 500)
            {
                float scale = 500.0f / (float)size[max_index];
                int w = (int)std::round(frame_cropped.cols * scale);
                int h = (int)std::round(frame_cropped.rows * scale);
                cv::resize(frame_cropped, frame_cropped, cv::Size(w, h));
                cv::resize(seg_render_cropped, seg_render_cropped, cv::Size(w, h));
                if (options_.debug) cv::resize(seg_cropped, seg_cropped, cv::Size(w, h));
            }

            // Output frame and segmentation
            cv::imwrite(str(boost::format("%s\\%s_frame_%04d.jpg") %
                output_dir_ % src_name_ % frame_number_), frame_cropped);
            cv::imwrite(str(boost::format("%s\\%s_seg_%04d.png") %
                output_dir_ % src_name_ % frame_number_), seg_render_cropped);
            if (options_.debug)
                cv::imwrite(str(boost::format("%s\\%s_debug_%04d.jpg") %
                    output_dir_ % src_name_ % frame_number_), seg_cropped);
        }

		// Forward input
		output->push_back(input);
		++frame_number_;
	}

	bool KeyframeWriter::PostProcess(list<FrameSetPtr>* append)
	{
		return false;
	}

}  // namespace fvs.
