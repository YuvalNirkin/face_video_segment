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

    void Keyframer::addFrame(const sfl::Frame& sfl_frame, Frame& fvs_frame)
    {
        auto& fvs_face_map = *fvs_frame.mutable_faces();

        // For each sfl face
        for (auto& sfl_face : sfl_frame.faces)
        {
            // Check to find a corresponding fvs face
            auto fvs_face = fvs_face_map.find(sfl_face->id);
            if (fvs_face == fvs_face_map.end()) continue;

            addFace(*sfl_face, fvs_face->second);
        }

        // Clear history for faces that were not found
        for (auto& face_data : m_face_data_map)
        {
            if (face_data.second.frame_updated_ind < m_frame_counter)
                face_data.second.history.clear();
        }

        ++m_frame_counter;
    }

    bool Keyframer::addFace(const sfl::Face& sfl_face, Face& fvs_face)
    {
        FaceData& face_data = m_face_data_map[sfl_face.id];
        face_data.frame_updated_ind = m_frame_counter;

        // Check if we the landmarks are not empty
        if (sfl_face.landmarks.empty())
        {
            face_data.history.clear();
            return false;
        }

        // Update history
        face_data.history.push_back(sfl_face.landmarks);
        if (face_data.history.size() > m_stability_range)
            face_data.history.pop_front();
        else return false;

        // Check start frame
        if (m_frame_counter < m_start_frame)
            return false;

        // Compare against previous keyframes
        cv::Point3f face_euler = sfl::getFaceApproxEulerAngles(sfl_face.landmarks);
        for (Keyframe& kf : face_data.keyframes)
        {
            float d = cv::norm(face_euler - kf.euler_angles);
            if (d < MIN_DIST) return false;
        }

        // Add new keyframe
        face_data.keyframes.push_front({ m_frame_counter , face_euler });
        fvs_face.set_keyframe(true);
        return true;
    }

}  // namespace fvs
