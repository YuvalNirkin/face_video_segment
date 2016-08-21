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

#include "keyframer.h"
#include <sfl/utilities.h>

const float MAX_DIST = 0.5f*sqrt(3.0f)*CV_PI;
const float MIN_DIST = MAX_DIST / 20.0f;

namespace fvs 
{
    Keyframer::Keyframer(int start_frame, int stability_range) :
        m_start_frame(start_frame), 
        m_stability_range(stability_range),
        m_frame_counter(0)
    {
    }

    bool Keyframer::addFrame(const std::vector<cv::Point>& landmarks)
    {
        ++m_frame_counter;

        // Check if a face was found this frame
        if (landmarks.empty())
        {
            m_history.clear();
            return false;
        }

        // Update history
        m_history.push_back(landmarks);
        if (m_history.size() > m_stability_range)
            m_history.pop_front();
        else return false;

        // Check start frame
        if (m_frame_counter < m_start_frame)
            return false;

        // Compare against previous keyframes
        cv::Point3f face_euler = sfl::getFaceApproxEulerAngles(landmarks);
        for (Keyframe& kf : m_keyframes)
        {
            float d = cv::norm(face_euler - kf.euler_angles);
            if (d < MIN_DIST) return false;
        }

        // Add new keyframe
        m_keyframes.push_front({ m_frame_counter - 1 , face_euler });

        return true;
    }

}  // namespace fvs
