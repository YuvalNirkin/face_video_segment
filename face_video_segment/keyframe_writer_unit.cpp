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

#include "base/base_impl.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/format.hpp>

using namespace video_framework;

const float MAX_DIST = 0.5f*sqrt(3.0f)*CV_PI;
const float MIN_DIST = MAX_DIST / 10.0f;
const float MAX_FACE_ANGLE = 75.0f;

cv::Point2f getLeftEye(const std::vector<cv::Point>& landmarks)
{
	if (landmarks.size() != 68) return cv::Point2f();

	cv::Point2f left_eye(0, 0);
	for (size_t i = 42; i <= 47; ++i)
		left_eye += cv::Point2f(landmarks[i]);

	return (left_eye / 6);
}

cv::Point2f getRightEye(const std::vector<cv::Point>& landmarks)
{
	if (landmarks.size() != 68) return cv::Point2f();

	cv::Point2f right_eye(0, 0);
	for (size_t i = 36; i <= 41; ++i)
		right_eye += cv::Point2f(landmarks[i]);

	return (right_eye / 6);
}

float getFaceApproxVertAngle(const std::vector<cv::Point>& landmarks)
{
	cv::Point2f left_eye = getLeftEye(landmarks);
	cv::Point2f right_eye = getRightEye(landmarks);
	cv::Point2f x1 = landmarks[0], x2 = landmarks[16];
	cv::Point2f v = x2 - x1;
	cv::Point2f right_eye_dir = x1 - right_eye;
	cv::Point2f left_eye_dir = x1 - left_eye;
	float x12_dist = cv::norm(v);
	float d1 = v.cross(right_eye_dir) / x12_dist;
	float d2 = v.cross(left_eye_dir) / x12_dist;
	float d = (d1 + d2)*0.5f / cv::norm(left_eye - right_eye);
	return d * (2 * MAX_FACE_ANGLE) * (CV_PI / 180.0f);
}

float getFaceApproxHorAngle(const std::vector<cv::Point>& landmarks)
{
	if (landmarks.size() != 68) return 0;
	const float max_angle = 75.0f;

	const cv::Point& center = landmarks[27];
	const cv::Point& left_eye = landmarks[42];
	const cv::Point& right_eye = landmarks[39];
	float left_dist = cv::norm(center - left_eye);
	float right_dist = cv::norm(center - right_eye);
	float d = (left_dist / (left_dist + right_dist) - 0.5f);

	return d * (2 * MAX_FACE_ANGLE) * (CV_PI / 180.0f);
}

float getFaceApproxTiltAngle(const std::vector<cv::Point>& landmarks)
{
	if (landmarks.size() != 68) return 0;

	cv::Point2f left_eye = getLeftEye(landmarks);
	cv::Point2f right_eye = getRightEye(landmarks);
	cv::Point2f v = left_eye - right_eye;
	return atan2(v.y, v.x);
}

cv::Point3f getFaceApproxEulerAngles(const std::vector<cv::Point>& landmarks)
{
	float x = getFaceApproxVertAngle(landmarks);
	float y = getFaceApproxHorAngle(landmarks);
	float z = getFaceApproxTiltAngle(landmarks);

	return cv::Point3f(x, y, z);
}

cv::Rect createBBoxFromLandmarks(const std::vector<cv::Point>& landmarks,
	const cv::Size& frameSize, bool square)
{
	int xmin(std::numeric_limits<int>::max()), ymin(std::numeric_limits<int>::max()),
		xmax(-1), ymax(-1), sumx(0), sumy(0);
	for (const cv::Point& p : landmarks)
	{
		xmin = std::min(xmin, p.x);
		ymin = std::min(ymin, p.y);
		xmax = std::max(xmax, p.x);
		ymax = std::max(ymax, p.y);
		sumx += p.x;
		sumy += p.y;
	}

	int width = xmax - xmin + 1;
	int height = ymax - ymin + 1;
	int centerx = (xmin + xmax) / 2;
	int centery = (ymin + ymax) / 2;
	int avgx = (int)std::round(sumx / landmarks.size());
	int avgy = (int)std::round(sumy / landmarks.size());
	int devx = centerx - avgx;
	int devy = centery - avgy;
	int dleft = (int)std::round(0.1*width) + abs(devx < 0 ? devx : 0);
	int dtop = (int)std::round(height*(std::max(float(width) / height, 1.0f) * 2 - 1)) + abs(devy < 0 ? devy : 0);
	int dright = (int)std::round(0.1*width) + abs(devx > 0 ? devx : 0);
	int dbottom = (int)std::round(0.1*height) + abs(devy > 0 ? devy : 0);

	// Limit to frame boundaries
	xmin = std::max(0, xmin - dleft);
	ymin = std::max(0, ymin - dtop);
	xmax = std::min((int)frameSize.width - 1, xmax + dright);
	ymax = std::min((int)frameSize.height - 1, ymax + dbottom);

	// Make square
	if (square)
	{
		int sq_width = std::max(xmax - xmin + 1, ymax - ymin + 1);
		centerx = (xmin + xmax) / 2;
		centery = (ymin + ymax) / 2;
		xmin = centerx - ((sq_width - 1) / 2);
		ymin = centery - ((sq_width - 1) / 2);
		xmax = xmin + sq_width - 1;
		ymax = ymin + sq_width - 1;

		// Limit to frame boundaries
		xmin = std::max(0, xmin);
		ymin = std::max(0, ymin);
		xmax = std::min((int)frameSize.width - 1, xmax);
		ymax = std::min((int)frameSize.height - 1, ymax);
	}

	return cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax));
}

namespace segmentation
{
	KeyframeWriter::KeyframeWriter(const KeyframeWriterOptions& options,
		const std::string& output_dir, const std::string& src_name)
		: options_(options), output_dir_(output_dir), src_name_(src_name)
	{
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
		if (frame_number_ < options_.start_frame)
		{
			++frame_number_;
			return;
		}

		// Retrieve video frame
		const VideoFrame* vid_frame = input->at(video_stream_idx_)->AsPtr<VideoFrame>();
		cv::Mat frame;
		vid_frame->MatView(&frame);

		// Retrieve landmarks
		const PointerFrame<std::vector<cv::Point>>& landmarks_frame =
			input->at(landmarks_stream_idx_)->As<PointerFrame<std::vector<cv::Point>>>();

		const std::vector<cv::Point>& landmarks = landmarks_frame.Ref();

		// Retrieve face segmentation renderer frame
		const VideoFrame* seg_frame = input->at(face_segment_renderer_stream_idx_)->AsPtr<VideoFrame>();
		cv::Mat seg;
		seg_frame->MatView(&seg);

		// Process frame
		bool add_keyframe = true;
		if (!landmarks.empty())
		{
			cv::Point3f face_euler = getFaceApproxEulerAngles(landmarks);

			// Compare against previous keyframes
			for (Keyframe& kf : keyframes_)
			{
				float d = cv::norm(face_euler - kf.euler_angles);
				std::cout << "dist to kf " << kf.id << " = " << d << std::endl;
				if (d < MIN_DIST)
				{
					add_keyframe = false;
					break;
				}
			}
		
			if (add_keyframe)
			{
				// Add keyframe
				std::cout << "adding keyframe " << frame_number_ << std::endl;
				keyframes_.push_front({ frame_number_ , face_euler });

				// Crop frame and segmentation
				cv::Rect bbox = createBBoxFromLandmarks(landmarks, frame.size(), true);
				cv::Mat frame_cropped = frame(bbox);
				cv::Mat seg_cropped = seg(bbox);
				
				// Limit resolution
				cv::MatSize size = frame_cropped.size;
				int max_index = std::distance(size.p, std::max_element(size.p, size.p + 2));
				if (size[max_index] > 500)
				{
					float scale = 500.0f / (float)size[max_index];
					int w = (int)std::round(frame_cropped.cols * scale);
					int h = (int)std::round(frame_cropped.rows * scale);
					cv::resize(frame_cropped, frame_cropped, cv::Size(w, h));
					cv::resize(seg_cropped, seg_cropped, cv::Size(w, h));
				}

				// Output frame and segmentation
				cv::imwrite(str(boost::format("%s\\%s_frame_%04d.png") %
					output_dir_ % src_name_ % frame_number_), frame_cropped);
				cv::imwrite(str(boost::format("%s\\%s_seg_%04d.png") %
					output_dir_ % src_name_ % frame_number_), seg_cropped);
			}
		}

		// Forward input
		output->push_back(input);
		++frame_number_;
	}

	bool KeyframeWriter::PostProcess(list<FrameSetPtr>* append)
	{
		return false;
	}

}  // namespace video_framework.
