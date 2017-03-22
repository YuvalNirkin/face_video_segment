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

#ifndef FACE_VIDEO_SEGMENT_FACE_SEGMENTATION_UNIT_H__
#define FACE_VIDEO_SEGMENT_FACE_SEGMENTATION_UNIT_H__

#include "base/base.h"
#include "video_framework/video_unit.h"
#include "segment_util/segmentation_util.h"
#include "face_regions.h"

namespace fvs
{
    using namespace segmentation;

    struct FaceRegionsReaderOptions {
        std::string stream_name = "FaceRegionsStream";
    };

    /** @brief Reads face regions from file into stream.
    */
    class FaceRegionsReaderUnit : public video_framework::VideoUnit
    {
    public:
        FaceRegionsReaderUnit(const FaceRegionsReaderOptions& options,
            const std::string& fvs_path);
        ~FaceRegionsReaderUnit();

        FaceRegionsReaderUnit(const FaceRegionsReaderUnit&) = delete;
        FaceRegionsReaderUnit& operator=(const FaceRegionsReaderUnit&) = delete;

        virtual bool OpenStreams(video_framework::StreamSet* set);
        virtual void ProcessFrame(video_framework::FrameSetPtr input, std::list<video_framework::FrameSetPtr>* output);
        virtual bool PostProcess(std::list<video_framework::FrameSetPtr>* append);

    private:
        FaceRegionsReaderOptions options_;
        std::unique_ptr<Sequence> fvs_sequence_;
        int frame_number_ = 0;
    };

    struct FaceRegionsOptions {
        std::string stream_name = "FaceRegionsStream";
        std::string video_stream_name = "VideoStream";
        std::string segment_stream_name = "SegmentationStream";
        std::string landmarks_stream_name = "LandmarksStream";
        std::string video_path = "";
        std::string seg_path = "";
        std::string landmarks_path = "";
    };

    /** @brief Classifies face regions from stream.
    */
    class FaceRegionsUnit : public video_framework::VideoUnit
    {
    public:
        FaceRegionsUnit(const FaceRegionsOptions& options);
        ~FaceRegionsUnit();

        FaceRegionsUnit(const FaceRegionsUnit&) = delete;
        FaceRegionsUnit& operator=(const FaceRegionsUnit&) = delete;

        virtual bool OpenStreams(video_framework::StreamSet* set);
        virtual void ProcessFrame(video_framework::FrameSetPtr input, std::list<video_framework::FrameSetPtr>* output);
        virtual bool PostProcess(std::list<video_framework::FrameSetPtr>* append);

        /** @brief Save current sequence of face segmentations to file.
        */
        virtual void save(const std::string& filePath) const;

    private:
        FaceRegionsOptions options_;
        int video_stream_idx_;
        int landmarks_stream_idx_;
        int seg_stream_idx_;

        int frame_width_;
        int frame_height_;
        int frame_number_ = 0;

        std::unique_ptr<FaceRegions> face_regions_;
        std::unique_ptr<Sequence> m_fvs_sequence;
    };

}  // namespace fvs

#endif  // FACE_VIDEO_SEGMENT_FACE_SEGMENTATION_UNIT_H__
