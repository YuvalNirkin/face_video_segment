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

#ifndef FVS_KEYFRAMER_H__
#define FVS_KEYFRAMER_H__

#include "face_video_segment.pb.h"

// std
#include <list>

// sfl
#include <sfl/sequence_face_landmarks.h>

// OpenCV
#include <opencv2/core.hpp>

namespace fvs {

    class Keyframer
    {
    public:

        struct Keyframe
        {
            int id;
            cv::Point3f euler_angles;
        };

        struct FaceData
        {
            std::list<Keyframe> keyframes;
            std::list<std::vector<cv::Point>> history;
            int frame_updated_ind;
        };

        Keyframer(int start_frame = 10, int stability_range = 5);

        void addFrame(const sfl::Frame& sfl_frame, Frame& fvs_frame);

    private:

        bool addFace(const sfl::Face& sfl_face, Face& fvs_face);

    private:
        //std::list<Keyframe> m_keyframes;
        //std::list<std::vector<cv::Point>> m_history;
        std::map<int, FaceData> m_face_data_map;
        int m_start_frame;
        int m_stability_range;
        int m_frame_counter;
    };

}  // namespace fvs

#endif  // FVS_KEYFRAMER_H__
