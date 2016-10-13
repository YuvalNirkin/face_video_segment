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

#include "video_reader_unit2.h"

#include <gflags/gflags.h>

#include "base/base_impl.h"

using namespace video_framework;

namespace fvs
{

VideoReaderUnit2::VideoReaderUnit2(const VideoReader2Options& options,
                                 const std::string& video_file)
    : options_(options),
      video_file_(video_file)
{
}

VideoReaderUnit2::~VideoReaderUnit2()
{
}

bool VideoReaderUnit2::OpenStreams(StreamSet* set)
{
    // Initialize video capture
    m_cap.reset(new cv::VideoCapture());
    if (!m_cap->open(video_file_))
        LOG(ERROR) << "Could not open file: " << video_file_;

    frame_width_ = (int)m_cap->get(cv::CAP_PROP_FRAME_WIDTH);
    frame_height_ = (int)m_cap->get(cv::CAP_PROP_FRAME_HEIGHT);
    fps_ = m_cap->get(cv::CAP_PROP_FPS);
    frame_width_step_ = frame_width_ * 3;

    // Add video output stream.
    VideoStream* rendered_stream = new VideoStream(frame_width_,
        frame_height_,
        frame_width_step_,
        fps_,
        PIXEL_FORMAT_BGR24,
        options_.stream_name);

    set->push_back(shared_ptr<DataStream>(rendered_stream));
    frame_num_ = 0;
    return true;
}

void VideoReaderUnit2::ProcessFrame(FrameSetPtr input, list<FrameSetPtr>* output) {
  if (!used_as_root_) {
    input->push_back(shared_ptr<VideoFrame>(ReadNextFrame()));
    output->push_back(input);
    ++frame_num_;
  }
}

bool VideoReaderUnit2::PostProcess(list<FrameSetPtr>* append) {
  if (used_as_root_) {
    VideoFrame* next_frame = ReadNextFrame();
    if (next_frame != nullptr) {
      // Make new frameset and push VideoFrame.
      FrameSetPtr frame_set (new FrameSet());
      frame_set->push_back(shared_ptr<VideoFrame>(next_frame));
      append->push_back(frame_set);
      ++frame_num_;
      return true;
    } else {
      return false;
    }
  }
  return false;
}

VideoFrame* VideoReaderUnit2::ReadNextFrame()
{
    if (!m_cap->read(m_frame)) return nullptr;

    VideoFrame* curr_frame = new VideoFrame(frame_width_,
        frame_height_,
        3);

    memcpy(curr_frame->mutable_data(), m_frame.data, 
        frame_width_ * frame_height_ * 3);

    return curr_frame;
}

}  // namespace fvs
