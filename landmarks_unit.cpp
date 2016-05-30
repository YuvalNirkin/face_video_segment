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

// boost geometry
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/io/io.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/strategies/strategies.hpp>	// important

using namespace video_framework;
namespace bg = boost::geometry;

typedef bg::model::d2::point_xy<float> point_t;
typedef bg::model::ring<point_t> ring_t;
typedef bg::model::polygon<point_t> poly_t;
typedef bg::model::multi_polygon<poly_t> mpoly_t;

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
		void renderLandmarks(cv::Mat& img, const dlib::full_object_detection& landmarks,
			const cv::Scalar& color = cv::Scalar(0, 255, 0));
		void createRing(const VectorMesh& mesh, const SegmentationDesc_Polygon& poly, ring_t& ring);
		void renderMultiPolygon(cv::Mat& img, const mpoly_t& mpoly);
		void renderRing(cv::Mat& img, const ring_t& ring, 
			const cv::Scalar& color = cv::Scalar(0, 255, 0));

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

		// Get segmentation stream.
		seg_stream_idx_ = FindStreamIdx(options_.segment_stream_name, set);

		if (seg_stream_idx_ < 0) {
			LOG(ERROR) << "SegmentationRenderUnit::OpenStreams: "
				<< "Could not find Segmentation stream!\n";
			return false;
		}

		// Add stream.
		DataStream* landmarks_stream = new DataStream(options_.stream_name);
		set->push_back(shared_ptr<DataStream>(landmarks_stream));

		// Open display window.
		std::ostringstream os;
		os << "LandmarksUnit_" << display_unit_id_;
		window_name_ = os.str();

		cv::namedWindow(window_name_);
		cv::waitKey(10);
		return true;
	}

	void LandmarksUnitImpl::ProcessFrame(FrameSetPtr input, std::list<FrameSetPtr>* output)
	{
		// Retrieve video frame
		const VideoFrame* frame = input->at(video_stream_idx_)->AsPtr<VideoFrame>();
		cv::Mat image;
		frame->MatView(&image);

		// Retrieve Segmentation.
		const PointerFrame<SegmentationDesc>& seg_frame =
			input->at(seg_stream_idx_)->As<PointerFrame<SegmentationDesc>>();

		const SegmentationDesc& desc = seg_frame.Ref();

		// Convert OpenCV's mat to dlib format
		dlib::cv_image<dlib::bgr_pixel> dlib_frame(image);

		// Detect bounding boxes around all the faces in the image.
		std::vector<dlib::rectangle> faces = detector_(dlib_frame);

		// If faces were found
		std::unique_ptr<ring_t> landmarks_ring(new ring_t());
		if (!faces.empty())
		{
			// Determine main face
			dlib::rectangle& main_face = selectMainFace(image, faces);

			// Detect landmarks
			dlib::full_object_detection landmarks = pose_model_(dlib_frame, main_face);

			// Output landmarks
			unsigned long num_parts = landmarks.num_parts();
			landmarks_ring->reserve(num_parts);
			for (unsigned long i = 0; i < num_parts; ++i)
			{
				landmarks_ring->push_back(
				{ (float)landmarks.part(i).x(), (float)landmarks.part(i).y() });
			}
			
			

			// Create face ring
			cv::Point2f p19((float)landmarks.part(19).x(), (float)landmarks.part(19).y());
			cv::Point2f p21((float)landmarks.part(21).x(), (float)landmarks.part(21).y());
			cv::Point2f p22((float)landmarks.part(22).x(), (float)landmarks.part(22).y());
			cv::Point2f p24((float)landmarks.part(24).x(), (float)landmarks.part(24).y());
			cv::Point2f p27((float)landmarks.part(27).x(), (float)landmarks.part(27).y());
			cv::Point2f p34((float)landmarks.part(34).x(), (float)landmarks.part(34).y());
			cv::Point2f pmid = (p21 + p22) * 0.5f;
			cv::Point2f dir = (p27 - p34);
			//cv::Point2f ptop = pmid + (p27 - p34);
			cv::Point2f ptop_l = p19 + dir;
			cv::Point2f ptop_r = p24 + dir;
			ring_t face{ 
				{ (float)landmarks.part(0).x(), (float)landmarks.part(0).y() },
				{ (float)landmarks.part(1).x(), (float)landmarks.part(1).y() },
				{ (float)landmarks.part(2).x(), (float)landmarks.part(2).y() },
				{ (float)landmarks.part(3).x(), (float)landmarks.part(3).y() },
				{ (float)landmarks.part(4).x(), (float)landmarks.part(4).y() },
				{ (float)landmarks.part(5).x(), (float)landmarks.part(5).y() },
				{ (float)landmarks.part(6).x(), (float)landmarks.part(6).y() },
				{ (float)landmarks.part(7).x(), (float)landmarks.part(7).y() },
				{ (float)landmarks.part(8).x(), (float)landmarks.part(8).y() },
				{ (float)landmarks.part(9).x(), (float)landmarks.part(9).y() },
				{ (float)landmarks.part(10).x(), (float)landmarks.part(10).y() },
				{ (float)landmarks.part(11).x(), (float)landmarks.part(11).y() },
				{ (float)landmarks.part(12).x(), (float)landmarks.part(12).y() },
				{ (float)landmarks.part(13).x(), (float)landmarks.part(13).y() },
				{ (float)landmarks.part(14).x(), (float)landmarks.part(14).y() },
				{ (float)landmarks.part(15).x(), (float)landmarks.part(15).y() },
				{ (float)landmarks.part(16).x(), (float)landmarks.part(16).y() },
				{ (float)landmarks.part(26).x(), (float)landmarks.part(26).y() },
				{ ptop_r.x, ptop_r.y },
				{ ptop_l.x, ptop_l.y },
				{ (float)landmarks.part(17).x(), (float)landmarks.part(17).y() },
				{ (float)landmarks.part(0).x(), (float)landmarks.part(0).y() }
			};
			bg::correct(face);

			renderLandmarks(image, landmarks);
			renderRing(image, face, cv::Scalar(0, 0, 255));

			// For each region
			const VectorMesh& mesh = desc.vector_mesh();
			for (const auto& r : desc.region())
			{
				if (r.vectorization().polygon().empty()) continue;
				mpoly_t mpoly;

				// For each polygon
				for (const auto& poly : r.vectorization().polygon())
				{
					if (poly.coord_idx_size() == 0) continue;
					if (poly.hole())
					{
						ring_t ring;
						createRing(mesh, poly, ring);

						// For each polygon already added to the multipolygon 
						// (assumes that all holes come after the outer polygons)
						for (poly_t& bpoly : mpoly)
						{
							if (bg::covered_by(ring, bpoly))
							{
								bpoly.inners().push_back(ring);
								break;
							}
						}
					}
					else
					{
						mpoly.resize(mpoly.size() + 1);
						createRing(mesh, poly, mpoly.back().outer());
					}
				}

				// Calculate intersection area ratio
				mpoly_t out;
				bg::intersection(face, mpoly, out);
				double ratio = (bg::area(out) / bg::area(mpoly));
				std::cout << "ratio = " << ratio << std::endl;
				const double t = 0.5;


				//if(ratio > t) renderMultiPolygon(image, mpoly);
				//cv::imshow(window_name_.c_str(), image);
				//cv::waitKey(1);
			}
		}

		if (frame_buffer_) {
			cv::resize(image, *frame_buffer_, frame_buffer_->size());
			cv::imshow(window_name_.c_str(), *frame_buffer_);
		}
		else {
			cv::imshow(window_name_.c_str(), image);
		}

		// Forward input
		input->push_back(std::shared_ptr<PointerFrame<ring_t>>(
			new PointerFrame<ring_t>(std::move(landmarks_ring))));

		output->push_back(input);
		cv::waitKey(1);
	}

	bool LandmarksUnitImpl::PostProcess(list<FrameSetPtr>* append)
	{
		cv::destroyWindow(window_name_);
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

	void LandmarksUnitImpl::renderLandmarks(cv::Mat& img,
		const dlib::full_object_detection& landmarks,
		const cv::Scalar& color)
	{
		if (landmarks.num_parts() != 68)
			throw std::runtime_error("Each shape size must be exactly 68!");

		const dlib::full_object_detection& d = landmarks;
		for (unsigned long i = 1; i <= 16; ++i)
			cv::line(img, cv::Point(d.part(i).x(), d.part(i).y()),
				cv::Point(d.part(i - 1).x(), d.part(i - 1).y()), color);

		for (unsigned long i = 28; i <= 30; ++i)
			cv::line(img, cv::Point(d.part(i).x(), d.part(i).y()),
				cv::Point(d.part(i - 1).x(), d.part(i - 1).y()), color);

		for (unsigned long i = 18; i <= 21; ++i)
			cv::line(img, cv::Point(d.part(i).x(), d.part(i).y()),
				cv::Point(d.part(i - 1).x(), d.part(i - 1).y()), color);
		for (unsigned long i = 23; i <= 26; ++i)
			cv::line(img, cv::Point(d.part(i).x(), d.part(i).y()),
				cv::Point(d.part(i - 1).x(), d.part(i - 1).y()), color);
		for (unsigned long i = 31; i <= 35; ++i)
			cv::line(img, cv::Point(d.part(i).x(), d.part(i).y()),
				cv::Point(d.part(i - 1).x(), d.part(i - 1).y()), color);
		cv::line(img, cv::Point(d.part(30).x(), d.part(30).y()),
			cv::Point(d.part(35).x(), d.part(35).y()), color);

		for (unsigned long i = 37; i <= 41; ++i)
			cv::line(img, cv::Point(d.part(i).x(), d.part(i).y()),
				cv::Point(d.part(i - 1).x(), d.part(i - 1).y()), color);
		cv::line(img, cv::Point(d.part(36).x(), d.part(36).y()),
			cv::Point(d.part(41).x(), d.part(41).y()), color);

		for (unsigned long i = 43; i <= 47; ++i)
			cv::line(img, cv::Point(d.part(i).x(), d.part(i).y()),
				cv::Point(d.part(i - 1).x(), d.part(i - 1).y()), color);
		cv::line(img, cv::Point(d.part(42).x(), d.part(42).y()),
			cv::Point(d.part(47).x(), d.part(47).y()), color);

		for (unsigned long i = 49; i <= 59; ++i)
			cv::line(img, cv::Point(d.part(i).x(), d.part(i).y()),
				cv::Point(d.part(i - 1).x(), d.part(i - 1).y()), color);
		cv::line(img, cv::Point(d.part(48).x(), d.part(48).y()),
			cv::Point(d.part(59).x(), d.part(59).y()), color);

		for (unsigned long i = 61; i <= 67; ++i)
			cv::line(img, cv::Point(d.part(i).x(), d.part(i).y()),
				cv::Point(d.part(i - 1).x(), d.part(i - 1).y()), color);
		cv::line(img, cv::Point(d.part(60).x(), d.part(60).y()),
			cv::Point(d.part(67).x(), d.part(67).y()), color);

		// Add labels
		for (unsigned long i = 0; i < 68; ++i)
			cv::putText(img, std::to_string(i), cv::Point(d.part(i).x(), d.part(i).y()),
				cv::FONT_HERSHEY_PLAIN, 0.5, color, 1.0);
	}

	void LandmarksUnitImpl::createRing(const VectorMesh& mesh,
		const SegmentationDesc_Polygon& poly, ring_t& ring)
	{
		// For each coordinate
		ring.resize(poly.coord_idx_size() - 1);
		for (int c = 0; c < ring.size(); ++c)
		{
			int idx = poly.coord_idx(c);
			ring[c] = point_t(mesh.coord(idx), mesh.coord(idx + 1));
		}

		bg::correct(ring);
	}

	void LandmarksUnitImpl::renderMultiPolygon(cv::Mat& img, const mpoly_t& mpoly)
	{
		cv::Scalar color = cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		//cv::Scalar color(0, 255, 0);
		for (const poly_t& poly : mpoly)
		{
			renderRing(img, poly.outer(), color);
			for(const ring_t ring : poly.inners())
				renderRing(img, poly.outer(), color);
		}
	}

	void LandmarksUnitImpl::renderRing(cv::Mat& img, const ring_t& ring, 
		const cv::Scalar& color)
	{
		if (ring.empty()) return;

		cv::putText(img, std::to_string(0), cv::Point(ring[0].x(), ring[0].y()),
			cv::FONT_HERSHEY_PLAIN, 0.5, color, 1.0);
		for (int c = 1; c < ring.size(); ++c)
		{
			cv::line(img, cv::Point(ring[c].x(), ring[c].y()),
				cv::Point(ring[c - 1].x(), ring[c - 1].y()), color);
			cv::putText(img, std::to_string(c), cv::Point(ring[c].x(), ring[c].y()),
				cv::FONT_HERSHEY_PLAIN, 0.5, color, 1.0);
		}
	}

}  // namespace video_framework.
