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

// boost geometry
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>
#include <boost/geometry/io/io.hpp>
#include <boost/geometry/algorithms/covered_by.hpp>
#include <boost/geometry/algorithms/correct.hpp>
#include <boost/geometry/algorithms/intersection.hpp>
#include <boost/geometry/strategies/strategies.hpp>	// important

namespace bg = boost::geometry;

namespace segmentation 
{
	typedef bg::model::d2::point_xy<float> point_t;
	typedef bg::model::ring<point_t> ring_t;
	typedef bg::model::polygon<point_t> poly_t;
	typedef bg::model::multi_polygon<poly_t> mpoly_t;

	struct FaceSegmentationOptions {
		std::string stream_name = "FaceSegmentationStream";
		std::string landmarks_stream_name = "LandmarksStream";
		std::string segment_stream_name = "SegmentationStream";
	};

	class FaceSegmentationUnit : public video_framework::VideoUnit 
	{
	public:
		FaceSegmentationUnit(const FaceSegmentationOptions& options);
		~FaceSegmentationUnit();

		FaceSegmentationUnit(const FaceSegmentationUnit&) = delete;
		FaceSegmentationUnit& operator=(const FaceSegmentationUnit&) = delete;

		virtual bool OpenStreams(video_framework::StreamSet* set);
		virtual void ProcessFrame(video_framework::FrameSetPtr input, std::list<video_framework::FrameSetPtr>* output);
		virtual bool PostProcess(std::list<video_framework::FrameSetPtr>* append);

	private:
		void createRing(const VectorMesh& mesh, const SegmentationDesc_Polygon& poly, ring_t& ring);
		/*
		void renderMultiPolygon(cv::Mat& img, const mpoly_t& mpoly);
		void renderRing(cv::Mat& img, const ring_t& ring,
			const cv::Scalar& color = cv::Scalar(0, 255, 0));
			*/

	private:
		FaceSegmentationOptions options_;
		int landmarks_stream_idx_;
		int seg_stream_idx_;
	};

}  // namespace segmentation

#endif  // FACE_VIDEO_SEGMENT_FACE_SEGMENTATION_UNIT_H__
