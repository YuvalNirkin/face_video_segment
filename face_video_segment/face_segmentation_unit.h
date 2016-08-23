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

	struct FaceSegmentationOptions {
		std::string stream_name = "FaceSegmentationStream";
		std::string video_stream_name = "VideoStream";
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
		void rasterFaceSeg(const std::vector<cv::Point>& landmarks, const SegmentationDesc& seg_desc,
			std::vector<int>& output_ids);


	private:
		FaceSegmentationOptions options_;
		int video_stream_idx_;
		int landmarks_stream_idx_;
		int seg_stream_idx_;

		int frame_width_;
		int frame_height_;
	};

	struct FaceSegGlobalOptions {
		std::string stream_name = "FaceSegmentationStream";
		std::string video_stream_name = "VideoStream";
		std::string landmarks_stream_name = "LandmarksStream";
		std::string segment_stream_name = "SegmentationStream";
	};

	struct RegionStat
	{
		std::vector<float> ratios;
		std::vector<int> frame_ids;
		float avg = 0.0f;
		float max_ratio = 0.0f;
	};

	class FaceSegGlobalUnit : public video_framework::VideoUnit
	{
	public:
		FaceSegGlobalUnit(const FaceSegGlobalOptions& options);
		~FaceSegGlobalUnit();

		FaceSegGlobalUnit(const FaceSegGlobalUnit&) = delete;
		FaceSegGlobalUnit& operator=(const FaceSegGlobalUnit&) = delete;

		virtual bool OpenStreams(video_framework::StreamSet* set);
		virtual void ProcessFrame(video_framework::FrameSetPtr input, std::list<video_framework::FrameSetPtr>* output);
		virtual bool PostProcess(std::list<video_framework::FrameSetPtr>* append);

		std::map<int, RegionStat>& getRegionStats();

	private:
		void rasterFaceSeg(const std::vector<cv::Point>& landmarks, const SegmentationDesc& seg_desc);


	private:
		FaceSegGlobalOptions options_;
		int video_stream_idx_;
		int landmarks_stream_idx_;
		int seg_stream_idx_;

		int frame_width_;
		int frame_height_;

		std::map<int, RegionStat> region_stats_;

		int frame_number_ = 0;
		cv::Mat frame_;	// Debug
	};

	struct FaceSegLocalOptions {
		std::string stream_name = "FaceSegLocalStream";
		std::string video_stream_name = "VideoStream";
		std::string segment_stream_name = "SegmentationStream";
		std::string landmarks_stream_name = "LandmarksStream";
	};

	struct FaceSegLocalOutput
	{
		cv::Mat seg;
		std::vector<int> region_ids;
	};

	class FaceSegLocalUnit : public video_framework::VideoUnit
	{
	public:
		FaceSegLocalUnit(const FaceSegLocalOptions& options,
			const std::map<int, RegionStat>& region_stats);
		~FaceSegLocalUnit();

		FaceSegLocalUnit(const FaceSegLocalUnit&) = delete;
		FaceSegLocalUnit& operator=(const FaceSegLocalUnit&) = delete;

		virtual bool OpenStreams(video_framework::StreamSet* set);
		virtual void ProcessFrame(video_framework::FrameSetPtr input, std::list<video_framework::FrameSetPtr>* output);
		virtual bool PostProcess(std::list<video_framework::FrameSetPtr>* append);

	private:
		void findFaceRegions(const SegmentationDesc& seg_desc, std::vector<int>& output_ids);
		void calcSegmentation1(const cv::Mat& frame, const SegmentationDesc& seg_desc,
			const std::vector<int>& region_ids, cv::Mat& seg);
		void calcSegmentation2(const cv::Mat& frame, const SegmentationDesc& seg_desc,
			const std::vector<cv::Point>& landmarks, const std::vector<int>& region_ids,
			cv::Mat& seg);

	private:
		FaceSegLocalOptions options_;
		int video_stream_idx_;
		int landmarks_stream_idx_;
		int seg_stream_idx_;

		int frame_width_;
		int frame_height_;

		const std::map<int, RegionStat>& region_stats_;
		int frame_number_ = 0;

		// Holds the segmentation for the current chunk.
		std::unique_ptr<SegmentationDesc> seg_hier_;
	};

    struct FaceRegionsOptions {
        std::string stream_name = "FaceRegionsStream";
        std::string video_stream_name = "VideoStream";
        std::string segment_stream_name = "SegmentationStream";
        std::string landmarks_stream_name = "LandmarksStream";
    };

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

	struct FaceSegmentationRendererOptions {
		std::string stream_name = "FaceSegmentationRendererStream";
		std::string video_stream_name = "VideoStream";
		std::string face_segment_stream_name = "FaceSegmentationStream";
		std::string segment_stream_name = "SegmentationStream";
		std::string landmarks_stream_name = "LandmarksStream";
		bool debug = false;
	};

	class FaceSegmentationRendererUnit : public video_framework::VideoUnit
	{
	public:
		FaceSegmentationRendererUnit(const FaceSegmentationRendererOptions& options,
			const std::map<int, RegionStat>& region_stats = std::map<int, RegionStat>());
		~FaceSegmentationRendererUnit();

		FaceSegmentationRendererUnit(const FaceSegmentationRendererUnit&) = delete;
		FaceSegmentationRendererUnit& operator=(const FaceSegmentationRendererUnit&) = delete;

		virtual bool OpenStreams(video_framework::StreamSet* set);
		virtual void ProcessFrame(video_framework::FrameSetPtr input, std::list<video_framework::FrameSetPtr>* output);
		virtual bool PostProcess(std::list<video_framework::FrameSetPtr>* append);

	private:
		void renderSelectedRegions(cv::Mat& img, const SegmentationDesc& seg_desc, 
			const std::vector<int>& region_ids, const cv::Scalar& color = cv::Scalar(0, 0, 255));

		void calcRegionCenters(const VectorMesh& mesh, const SegmentationDesc_Region2D& r,
			std::vector<cv::Point2f>& centers);

		void renderRegionIds(cv::Mat& img, const SegmentationDesc& seg_desc,
			const cv::Scalar& color = cv::Scalar(0, 0, 255));

		void renderSegmentation(cv::Mat& frame, const cv::Mat& seg);

		void renderSegmentation(cv::Mat& frame, const cv::Mat& seg, const cv::Scalar& color);

        void renderSegmentation(cv::Mat& frame, const cv::Mat& seg, uchar color);

		void renderBoundaries(cv::Mat& frame, const SegmentationDesc& seg_desc);

		void renderRegions(cv::Mat& frame, const SegmentationDesc& seg_desc);

	private:
		FaceSegmentationRendererOptions options_;
		int video_stream_idx_;
		int face_seg_stream_idx_;
		int seg_stream_idx_;
		int landmarks_stream_idx_;

		int frame_width_;
		int frame_height_;
		int frame_width_step_;
		int frame_counter_;
		float fps_;

		std::map<int, RegionStat> region_stats_;

		// Debug
		Hierarchy m_hierarchy;
	};

}  // namespace fvs

#endif  // FACE_VIDEO_SEGMENT_FACE_SEGMENTATION_UNIT_H__
