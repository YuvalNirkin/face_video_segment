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

#include "keyframe_unit.h"
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
    KeyframeDetectionUnit::KeyframeDetectionUnit(const KeyframeDetectionOptions& options)
        : options_(options)
    {
        keyframer_ = std::make_unique<Keyframer>(
            options.start_frame, options.stability_range);
    }

    KeyframeDetectionUnit::~KeyframeDetectionUnit() {
    }

    bool KeyframeDetectionUnit::OpenStreams(StreamSet* set) {
        // Find video stream idx.
        video_stream_idx_ = FindStreamIdx(options_.video_stream_name, set);

        if (video_stream_idx_ < 0) {
            LOG(ERROR) << "KeyframeDetectionUnit::OpenStreams: "
                << "Could not find video stream!\n";
            return false;
        }

        const VideoStream& vid_stream = set->at(video_stream_idx_)->As<VideoStream>();

        const int frame_width = vid_stream.frame_width();
        const int frame_height = vid_stream.frame_height();
        const float fps = vid_stream.fps();

        // Get landmarks stream
        landmarks_stream_idx_ = FindStreamIdx(options_.landmarks_stream_name, set);
        if (landmarks_stream_idx_ < 0) {
            LOG(ERROR) << "KeyframeDetectionUnit::OpenStreams: "
                << "Could not find landmarks stream!\n";
            return false;
        }

        // Get face regions stream.
        face_regions_stream_idx_ = FindStreamIdx(options_.face_regions_stream_name, set);
        if (face_regions_stream_idx_ < 0) {
            LOG(ERROR) << "KeyframeDetectionUnit::OpenStreams: "
                << "Could not find face regions stream!\n";
            return false;
        }

        return true;
    }

    void KeyframeDetectionUnit::ProcessFrame(FrameSetPtr input, std::list<FrameSetPtr>* output)
    {
        // Retrieve video frame
        const VideoFrame* vid_frame = input->at(video_stream_idx_)->AsPtr<VideoFrame>();
        cv::Mat frame;
        vid_frame->MatView(&frame);

        // Retrieve landmarks
        const ValueFrame<const sfl::Frame*>& landmarks_value_frame =
            input->at(landmarks_stream_idx_)->As<ValueFrame<const sfl::Frame*>>();
        const sfl::Frame* sfl_frame = landmarks_value_frame.Value();

        // Retrieve face regions data
        const ValueFrame<Frame*>& fvs_value_frame =
            input->at(face_regions_stream_idx_)->As<ValueFrame<Frame*>>();
        Frame* fvs_frame = fvs_value_frame.Value();

        // Process frame
        if (sfl_frame != nullptr && fvs_frame != nullptr)
            keyframer_->addFrame(*sfl_frame, *fvs_frame);

        // Forward input
        output->push_back(input);
        ++frame_number_;
    }

    bool KeyframeDetectionUnit::PostProcess(list<FrameSetPtr>* append)
    {
        return false;
    }

	KeyframeWriterUnit::KeyframeWriterUnit(const KeyframeWriterOptions& options,
		const std::string& output_dir, const std::string& src_name)
		: options_(options), output_dir_(output_dir), src_name_(src_name)
	{
	}

	KeyframeWriterUnit::~KeyframeWriterUnit() {
	}

	bool KeyframeWriterUnit::OpenStreams(StreamSet* set)
    {
        // Find video stream idx.
        video_stream_idx_ = FindStreamIdx(options_.video_stream_name, set);

        if (video_stream_idx_ < 0) {
            LOG(ERROR) << "KeyframeWriterUnit::OpenStreams: "
                << "Could not find video stream!\n";
            return false;
        }

		const VideoStream& vid_stream = set->at(video_stream_idx_)->As<VideoStream>();

		const int frame_width = vid_stream.frame_width();
		const int frame_height = vid_stream.frame_height();
		const float fps = vid_stream.fps();

        // Get segmentation stream.
        seg_stream_idx_ = FindStreamIdx(options_.segment_stream_name, set);

        if (seg_stream_idx_ < 0) {
            LOG(ERROR) << "KeyframeWriterUnit::OpenStreams: "
                << "Could not find Segmentation stream!\n";
            return false;
        }

		// Get landmarks stream
		landmarks_stream_idx_ = FindStreamIdx(options_.landmarks_stream_name, set);
		if (landmarks_stream_idx_ < 0) {
			LOG(ERROR) << "KeyframeWriterUnit::OpenStreams: "
				<< "Could not find landmarks stream!\n";
			return false;
		}

        // Get face regions stream.
        face_regions_stream_idx_ = FindStreamIdx(options_.face_regions_stream_name, set);
        if (face_regions_stream_idx_ < 0) {
            LOG(ERROR) << "KeyframeWriterUnit::OpenStreams: "
                << "Could not find face regions stream!\n";
            return false;
        }

		return true;
	}

	void KeyframeWriterUnit::ProcessFrame(FrameSetPtr input, std::list<FrameSetPtr>* output)
	{
		// Retrieve video frame
		const VideoFrame* vid_frame = input->at(video_stream_idx_)->AsPtr<VideoFrame>();
		cv::Mat frame;
		vid_frame->MatView(&frame);

        // Retrieve Segmentation.
        const PointerFrame<SegmentationDesc>& seg_frame =
            input->at(seg_stream_idx_)->As<PointerFrame<SegmentationDesc>>();
        const SegmentationDesc& seg_desc = seg_frame.Ref();

        // Retrieve landmarks
        const ValueFrame<const sfl::Frame*>& landmarks_value_frame =
            input->at(landmarks_stream_idx_)->As<ValueFrame<const sfl::Frame*>>();
        const sfl::Frame* sfl_frame = landmarks_value_frame.Value();

        // Retrieve face regions data
        const ValueFrame<Frame*>& fvs_value_frame =
            input->at(face_regions_stream_idx_)->As<ValueFrame<Frame*>>();
        Frame* fvs_frame = fvs_value_frame.Value();

		// Process frame
        if (sfl_frame != nullptr && fvs_frame != nullptr)
        {
            // For each face
            for (auto& fvs_face : fvs_frame->faces())
            {
                if (!fvs_face.second.keyframe()) continue;
                const sfl::Face* sfl_face = sfl_frame->getFace(fvs_face.second.id());

                // Create segmentation
                cv::Mat face_map, seg;
                if (sfl_face != nullptr)
                {
                    face_map = cv::Mat::zeros(frame.size(), CV_8U);
                    std::vector<std::vector<cv::Point>> face_boundary(1);
                    sfl::createFullFace(sfl_face->landmarks, face_boundary.back());
                    cv::drawContours(face_map, face_boundary, 0, cv::Scalar(255, 255, 255), CV_FILLED);
                    seg = calcSegmentation(face_map, fvs_face.second.regions(), seg_desc);
                }
                else seg = calcSegmentation(frame.size(), fvs_face.second.regions(), seg_desc);
                postprocessSegmentation(seg);

                // Render segmentation
                cv::Mat seg_render, seg_debug;
                renderSegmentation(seg_render, seg, 15);
                if (options_.debug)
                {
                    seg_debug = frame.clone();
                    cv::Scalar red(0, 0, 255);
                    renderSegmentationBlend(seg_debug, seg, 0.5f, red, red);
                }

                // Crop frame and segmentation
                // TODO: Add to utilities: getFaceBBoxFromSegmentation
                if (sfl_face == nullptr) continue;
                cv::Rect bbox = sfl::getFaceBBoxFromLandmarks(sfl_face->landmarks, frame.size(), true);
                cv::Mat frame_cropped = frame(bbox);
                cv::Mat seg_render_cropped = seg_render(bbox);
                cv::Mat seg_debug_cropped;
                if (options_.debug) seg_debug_cropped = seg_debug(bbox);

                // Limit resolution
                cv::MatSize size = frame_cropped.size;
                int max_index = std::distance(size.p, std::max_element(size.p, size.p + 2));
                if (options_.upscale || size[max_index] > (int)options_.max_scale)
                {
                    float scale = (float)options_.max_scale / (float)size[max_index];
                    int w = (int)std::round(frame_cropped.cols * scale);
                    int h = (int)std::round(frame_cropped.rows * scale);
                    cv::resize(frame_cropped, frame_cropped, cv::Size(w, h),
                        0.0, 0.0, cv::INTER_CUBIC);
                    cv::resize(seg_render_cropped, seg_render_cropped, cv::Size(w, h),
                        0.0, 0.0, cv::INTER_NEAREST);
                    if (options_.debug) cv::resize(seg_debug_cropped, seg_debug_cropped,
                        cv::Size(w, h), 0.0, 0.0, cv::INTER_NEAREST);
                }

                // Output frame and segmentation
                cv::imwrite(str(boost::format("%s\\%s_frame_%04d_face_%04d.jpg") %
                    output_dir_ % src_name_ % frame_number_ % fvs_face.second.id()), frame_cropped);
                cv::imwrite(str(boost::format("%s\\%s_seg_%04d_face_%04d.png") %
                    output_dir_ % src_name_ % frame_number_ % fvs_face.second.id()), seg_render_cropped);
                if (options_.debug)
                    cv::imwrite(str(boost::format("%s\\%s_debug_%04d_face_%04d.jpg") %
                        output_dir_ % src_name_ % frame_number_ % fvs_face.second.id()), seg_debug_cropped);

                /*/// Debug ///
                if (options_.debug)
                {
                    std::ofstream output(str(boost::format("%s\\%s_debug_%04d_face_%04d.txt") %
                        output_dir_ % src_name_ % frame_number_ % fvs_face.second.id()));
                    output <<  fvs_face.second.DebugString();
                }
                /////////////*/
            }
        }

		// Forward input
		output->push_back(input);
		++frame_number_;
	}

	bool KeyframeWriterUnit::PostProcess(list<FrameSetPtr>* append)
	{
		return false;
	}

}  // namespace fvs.
