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

#ifndef VFS_VIDEO_READER_UNIT2_H__
#define VFS_VIDEO_READER_UNIT2_H__

#include "base/base.h"
#include "video_framework/video_unit.h"

#include <opencv2/videoio.hpp>

struct AVCodec;
struct AVCodecContext;
struct AVFormatContext;
struct AVFrame;
struct AVPacket;
struct SwsContext;

namespace fvs {

struct VideoReader2Options {
  int trim_frames = 0;
  std::string stream_name = "VideoStream";
  video_framework::VideoPixelFormat pixel_format = 
      video_framework::PIXEL_FORMAT_BGR24;

  // For settings below only downscale will be performed, ie. never upscaling.
  enum DOWNSCALE {
    DOWNSCALE_NONE,
    DOWNSCALE_BY_FACTOR,     // Resizes each dimension by downscale_factor.

    DOWNSCALE_TO_MIN_SIZE,   // Resizes minimum dimension to downscale_size.
    DOWNSCALE_TO_MAX_SIZE,   // Resizes maximum dimension to downscale_size.
  };

  DOWNSCALE downscale = DOWNSCALE_NONE;

  float downscale_factor = 1.0f;
  int downscale_size = 0;
};

/** @brief Reads video frames from file into stream.
*/
class VideoReaderUnit2 : public video_framework::VideoUnit {
 public:
     VideoReaderUnit2(const VideoReader2Options& options,
                  const std::string& video_file);
  ~VideoReaderUnit2();

  virtual bool OpenStreams(video_framework::StreamSet* set);
  virtual void ProcessFrame(video_framework::FrameSetPtr input, 
      std::list<video_framework::FrameSetPtr>* output);
  virtual bool PostProcess(std::list<video_framework::FrameSetPtr>* append);
  /*
  // Experimental (might not seek to correct locations).
  virtual bool SeekImpl(int64_t pts);
  bool PrevFrame();

  // Can be positive or negative.
  bool SkipFrames(int frame_offset);
  */

 private:
  // Returns allocated VideoFrame (ownership passed to caller).
  // Returns NULL if end of file is reached.
     video_framework::VideoFrame* ReadNextFrame();

 private:
  VideoReader2Options options_;
  std::string video_file_;

  int video_stream_idx_;
  int frame_num_ = 0;

  int frame_width_ = 0;
  int frame_height_ = 0;
  int frame_width_step_ = 0;
  double fps_;

  bool used_as_root_ = true;

  std::unique_ptr<cv::VideoCapture> m_cap;
  cv::Mat m_frame;
};

}  // namespace fvs.

#endif  // VFS_VIDEO_READER_UNIT2_H__
