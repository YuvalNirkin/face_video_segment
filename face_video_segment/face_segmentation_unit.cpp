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
    FaceRegionsReaderUnit::FaceRegionsReaderUnit(const FaceRegionsReaderOptions& options,
        const std::string& fvs_path)
        : options_(options)
    {
        fvs_sequence_.reset(new Sequence());
        std::ifstream input(fvs_path, std::ifstream::binary);
        fvs_sequence_->ParseFromIstream(&input);
    }

    FaceRegionsReaderUnit::~FaceRegionsReaderUnit() {
    }

    bool FaceRegionsReaderUnit::OpenStreams(StreamSet* set)
    {
        // Add stream.
        set->push_back(shared_ptr<DataStream>(new DataStream(options_.stream_name)));

        return true;
    }

    void FaceRegionsReaderUnit::ProcessFrame(FrameSetPtr input, std::list<FrameSetPtr>* output)
    {
        // Get fvs frame from sequence
        Frame* fvs_frame = fvs_sequence_->mutable_frames(frame_number_);

        // Forward input
        input->push_back(std::shared_ptr<ValueFrame<Frame*>>(
            new ValueFrame<Frame*>(fvs_frame)));

        output->push_back(input);
        ++frame_number_;
    }

    bool FaceRegionsReaderUnit::PostProcess(list<FrameSetPtr>* append)
    {
        return false;
    }

    FaceRegionsUnit::FaceRegionsUnit(const FaceRegionsOptions& options)
        : options_(options)
    {
        face_regions_.reset(new FaceRegions());
        m_fvs_sequence.reset(new Sequence());
        m_fvs_sequence->set_video_path(options.video_path);
        m_fvs_sequence->set_seg_path(options.seg_path);
        m_fvs_sequence->set_landmarks_path(options.landmarks_path);
    }

    FaceRegionsUnit::~FaceRegionsUnit() {
    }

    bool FaceRegionsUnit::OpenStreams(StreamSet* set)
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

    void FaceRegionsUnit::ProcessFrame(FrameSetPtr input, std::list<FrameSetPtr>* output)
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
        const ValueFrame<const sfl::Frame*>& landmarks_frame = 
        input->at(landmarks_stream_idx_)->As<ValueFrame<const sfl::Frame*>>();
        const sfl::Frame* sfl_frame = landmarks_frame.Value();
//        const PointerFrame<std::vector<cv::Point>>& landmarks_frame =
//            input->at(landmarks_stream_idx_)->As<PointerFrame<std::vector<cv::Point>>>();
//        const std::vector<cv::Point>& landmarks = landmarks_frame.Ref();

        // Add a new frame to the fvs sequence
        Frame* fvs_frame = nullptr;
        if (sfl_frame != nullptr)
        {
            fvs_frame = m_fvs_sequence->add_frames();
            fvs_frame->set_id(frame_number_);
            fvs_frame->set_width(frame_width_);
            fvs_frame->set_height(frame_height_);

            // Calculate face regions
            face_regions_->addFrame(seg_desc, *sfl_frame, *fvs_frame);
        }
        
        // Forward input
        input->push_back(std::shared_ptr<ValueFrame<Frame*>>(
            new ValueFrame<Frame*>(fvs_frame)));

        output->push_back(input);
        ++frame_number_;
    }

    bool FaceRegionsUnit::PostProcess(list<FrameSetPtr>* append)
    {
        return false;
    }

    void FaceRegionsUnit::save(const std::string & filePath) const
    {
        if (m_fvs_sequence->frames_size() == 0) return;
        std::ofstream output(filePath, std::fstream::trunc | std::fstream::binary);
        m_fvs_sequence->SerializeToOstream(&output);
    }
   
}  // namespace fvs
