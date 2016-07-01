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

#include "landmarks_unit.h"
#include "face_segmentation_unit.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>

#include "base/base_impl.h"
#include <segmentation/segmentation.h>

// dlib
#include <dlib/opencv.h>
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing/shape_predictor.h>

using namespace video_framework;

namespace segmentation
{
	class LandmarksUnitImpl : public LandmarksUnit
	{
	public:
		LandmarksUnitImpl(const LandmarksOptions& options);
		~LandmarksUnitImpl();

		LandmarksUnitImpl(const LandmarksUnitImpl&) = delete;
		LandmarksUnitImpl& operator=(const LandmarksUnitImpl&) = delete;

		virtual bool OpenStreams(StreamSet* set);
		virtual void ProcessFrame(FrameSetPtr input, std::list<FrameSetPtr>* output);
		virtual bool PostProcess(std::list<FrameSetPtr>* append);

	private:
		dlib::rectangle& selectMainFace(const cv::Mat& img, std::vector<dlib::rectangle>& faces);

	private:
		LandmarksOptions options_;
		int video_stream_idx_;
		int seg_stream_idx_;
		int display_unit_id_;

		std::string window_name_;
		std::unique_ptr<cv::Mat> frame_buffer_;

		dlib::frontal_face_detector detector_;
		dlib::shape_predictor pose_model_;

		cv::RNG rng;

		static int display_unit_count;
	};


	int LandmarksUnitImpl::display_unit_count = 0;

	LandmarksUnitImpl::LandmarksUnitImpl(const LandmarksOptions& options)
		: options_(options), rng(0xFFFFFFFF)
	{
		display_unit_id_ = display_unit_count++;

		// Face detector for finding bounding boxes for each face in an image
		detector_ = dlib::get_frontal_face_detector();

		// Shape predictor for finding landmark positions given an image and face bounding box.
		dlib::deserialize(options_.landmarks_model_file) >> pose_model_;
	}

	LandmarksUnitImpl::~LandmarksUnitImpl() {
	}

	std::shared_ptr<LandmarksUnit> LandmarksUnit::create(const LandmarksOptions& options)
	{
		return std::make_shared<LandmarksUnitImpl>(options);
	}

	bool LandmarksUnitImpl::OpenStreams(StreamSet* set) {
		// Find video stream idx.
		video_stream_idx_ = FindStreamIdx(options_.video_stream_name, set);

		if (video_stream_idx_ < 0) {
			LOG(ERROR) << "Could not find Video stream!\n";
			return false;
		}

		const VideoStream& vid_stream = set->at(video_stream_idx_)->As<VideoStream>();

		const int frame_width = vid_stream.frame_width();
		const int frame_height = vid_stream.frame_height();

		if (options_.output_scale != 1.0f) {
			frame_buffer_.reset(new cv::Mat(frame_height * options_.output_scale,
				frame_width * options_.output_scale,
				CV_8UC3));
		}
		/*
		// Get segmentation stream.
		seg_stream_idx_ = FindStreamIdx(options_.segment_stream_name, set);

		if (seg_stream_idx_ < 0) {
			LOG(ERROR) << "SegmentationRenderUnit::OpenStreams: "
				<< "Could not find Segmentation stream!\n";
			return false;
		}
		*/

		// Add stream.
		DataStream* landmarks_stream = new DataStream(options_.stream_name);
		set->push_back(shared_ptr<DataStream>(landmarks_stream));

		// Open display window.
		std::ostringstream os;
		os << "LandmarksUnit_" << display_unit_id_;
		window_name_ = os.str();

		//cv::namedWindow(window_name_);
		//cv::waitKey(10);
		return true;
	}

	void LandmarksUnitImpl::ProcessFrame(FrameSetPtr input, std::list<FrameSetPtr>* output)
	{
		// Retrieve video frame
		const VideoFrame* frame = input->at(video_stream_idx_)->AsPtr<VideoFrame>();
		cv::Mat image;
		frame->MatView(&image);
		/*
		// Retrieve Segmentation.
		const PointerFrame<SegmentationDesc>& seg_frame =
			input->at(seg_stream_idx_)->As<PointerFrame<SegmentationDesc>>();

		const SegmentationDesc& desc = seg_frame.Ref();
		*/
		// Convert OpenCV's mat to dlib format
		dlib::cv_image<dlib::bgr_pixel> dlib_frame(image);

		// Detect bounding boxes around all the faces in the image.
		std::vector<dlib::rectangle> faces = detector_(dlib_frame);

		// If faces were found
		std::unique_ptr<std::vector<cv::Point>> landmarks_out(new std::vector<cv::Point>());
		if (!faces.empty())
		{
			// Determine main face
			dlib::rectangle& main_face = selectMainFace(image, faces);

			// Detect landmarks
			dlib::full_object_detection landmarks = pose_model_(dlib_frame, main_face);

			// Output landmarks
			unsigned long num_parts = landmarks.num_parts();
			landmarks_out->reserve(num_parts);
			for (unsigned long i = 0; i < num_parts; ++i)
			{
				landmarks_out->push_back(
					cv::Point(landmarks.part(i).x(), landmarks.part(i).y()));
			}
		}

		// Forward input
		input->push_back(std::shared_ptr<PointerFrame<std::vector<cv::Point>>>(
			new PointerFrame<std::vector<cv::Point>>(std::move(landmarks_out))));

		output->push_back(input);
	}

	bool LandmarksUnitImpl::PostProcess(list<FrameSetPtr>* append)
	{
		return false;
	}

	dlib::rectangle& LandmarksUnitImpl::selectMainFace(const cv::Mat& img,
		std::vector<dlib::rectangle>& faces)
	{
		std::vector<double> scores(faces.size());
		dlib::point bl, tr;
		cv::Point face_center, img_center(img.cols / 2, img.rows / 2);
		double dist, size;
		for (size_t i = 0; i < faces.size(); ++i)
		{
			dlib::rectangle& face = faces[i];
			bl = face.bl_corner(); tr = face.tr_corner();
			face_center = cv::Point((bl.x() + tr.x()) / 2, (bl.y() + tr.y()) / 2);
			dist = cv::norm(cv::Mat(img_center - face_center), cv::NORM_L2SQR);
			size = (double)face.area();
			scores[i] = size - dist;
		}

		// Return the face with the largest score
		int max_index = std::distance(scores.begin(),
			std::max_element(scores.begin(), scores.end()));
		return faces[max_index];
	}

	LandmarksRendererUnit::LandmarksRendererUnit(const LandmarksRendererOptions& options)
		: options_(options)
	{
	}

	LandmarksRendererUnit::~LandmarksRendererUnit() {
	}

	bool LandmarksRendererUnit::OpenStreams(StreamSet* set)
	{
		// Find video stream idx.
		video_stream_idx_ = FindStreamIdx(options_.video_stream_name, set);

		if (video_stream_idx_ < 0) {
			LOG(ERROR) << "Could not find Video stream!\n";
			return false;
		}

		const VideoStream& vid_stream = set->at(video_stream_idx_)->As<VideoStream>();

		frame_width = vid_stream.frame_width();
		frame_height = vid_stream.frame_height();

		// Get landmarks stream
		landmarks_stream_idx_ = FindStreamIdx(options_.landmarks_stream_name, set);
		if (landmarks_stream_idx_ < 0) {
			LOG(ERROR) << "SegmentationRenderUnit::OpenStreams: "
				<< "Could not find landmarks stream!\n";
			return false;
		}

		// Add stream.
		//DataStream* landmarks_stream = new DataStream(options_.stream_name);
		//set->push_back(shared_ptr<DataStream>(landmarks_stream));

		return true;
	}

	void LandmarksRendererUnit::ProcessFrame(FrameSetPtr input, std::list<FrameSetPtr>* output)
	{
		// Retrieve video frame
		const VideoFrame* frame = input->at(video_stream_idx_)->AsPtr<VideoFrame>();
		cv::Mat image;
		frame->MatView(&image);

		// Retrieve landmarks
		const PointerFrame<std::vector<cv::Point>>& landmarks_frame =
			input->at(landmarks_stream_idx_)->As<PointerFrame<std::vector<cv::Point>>>();

		const std::vector<cv::Point>& landmarks = landmarks_frame.Ref();

		// Render
		renderLandmarks(image, landmarks);

		// Forward input
		/*input->push_back(std::shared_ptr<PointerFrame<ring_t>>(
		new PointerFrame<ring_t>(std::move(landmarks_ring))));*/

		output->push_back(input);
	}

	bool LandmarksRendererUnit::PostProcess(list<FrameSetPtr>* append)
	{
		return false;
	}

	void LandmarksRendererUnit::renderLandmarks(cv::Mat& img,
		const std::vector<cv::Point>& landmarks,
		const cv::Scalar& color)
	{
		if (landmarks.size() != 68) return;

		//const dlib::full_object_detection& d = landmarks;
		for (unsigned long i = 1; i <= 16; ++i)
			cv::line(img, landmarks[i], landmarks[i - 1], color);

		for (unsigned long i = 28; i <= 30; ++i)
			cv::line(img, landmarks[i], landmarks[i - 1], color);

		for (unsigned long i = 18; i <= 21; ++i)
			cv::line(img, landmarks[i], landmarks[i - 1], color);

		for (unsigned long i = 23; i <= 26; ++i)
			cv::line(img, landmarks[i], landmarks[i - 1], color);

		for (unsigned long i = 31; i <= 35; ++i)
			cv::line(img, landmarks[i], landmarks[i - 1], color);
		cv::line(img, landmarks[30], landmarks[35], color);

		for (unsigned long i = 37; i <= 41; ++i)
			cv::line(img, landmarks[i], landmarks[i - 1], color);
		cv::line(img, landmarks[36], landmarks[41], color);

		for (unsigned long i = 43; i <= 47; ++i)
			cv::line(img, landmarks[i], landmarks[i - 1], color);
		cv::line(img, landmarks[42], landmarks[47], color);

		for (unsigned long i = 49; i <= 59; ++i)
			cv::line(img, landmarks[i], landmarks[i - 1], color);
		cv::line(img, landmarks[48], landmarks[59], color);

		for (unsigned long i = 61; i <= 67; ++i)
			cv::line(img, landmarks[i], landmarks[i - 1], color);
		cv::line(img, landmarks[60], landmarks[67], color);

		
		// Add labels
		for (unsigned long i = 0; i < 68; ++i)
			cv::putText(img, std::to_string(i), landmarks[i],
				cv::FONT_HERSHEY_PLAIN, 0.5, color, 1.0);
				
	}

}  // namespace video_framework.
