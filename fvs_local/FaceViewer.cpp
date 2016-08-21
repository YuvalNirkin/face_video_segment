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

#include "FaceViewer.h"
#include "landmarks_unit.h"
#include "face_segmentation_unit.h"
#include "video_writer_unit2.h"
#include "keyframe_writer_unit.h"
#include <video_framework/video_reader_unit.h>
#include <video_framework/video_display_unit.h>
#include <video_framework/video_writer_unit.h>
#include <video_framework/video_pipeline.h>
#include <segmentation/segmentation_unit.h>
#include <boost/filesystem.hpp>

using namespace video_framework;

namespace fvs
{
	FaceView::FaceView(const std::string& video_file, const std::string& seg_file,
		const std::string& landmarks_path, const std::string& output_dir,
		unsigned int verbose) :
		m_video_file(video_file), m_seg_file(seg_file),
		m_landmarks_path(landmarks_path), m_output_dir(output_dir),
		m_verbose(verbose)
	{
	}

	void FaceView::run()
	{
		//run_serial();
		//run_parallel();
		test3();
	}

	/*
	void FaceView::run_serial()
	{
		// Video Reader Unit
		VideoReaderUnit reader(VideoReaderOptions(), m_video_file);

		// Segmentation Reader Unit
		SegmentationReaderUnitOptions segOptions;
		segOptions.filename = m_seg_file;
		SegmentationReaderUnit segReader(segOptions);
		segReader.AttachTo(&reader);

		//VideoDisplayUnit display((VideoDisplayOptions()));
		//display.AttachTo(&reader);

		// Landmarks Unit
		LandmarksOptions landmarksOptions;
		landmarksOptions.landmarks_model_file = m_landmarks_model_file;
		std::shared_ptr<LandmarksUnit> landmarksUnit = LandmarksUnit::create(landmarksOptions);
		landmarksUnit->AttachTo(&segReader);

		// Face Segmentation Unit
		FaceSegmentationOptions faceSegOptions;
		FaceSegmentationUnit faceSegUnit(faceSegOptions);
		faceSegUnit.AttachTo(landmarksUnit.get());

		// Face Segmentation Renderer Unit
		FaceSegmentationRendererOptions faceSegRendererOptions;
		FaceSegmentationRendererUnit faceSegRendererUnit(faceSegRendererOptions);
		faceSegRendererUnit.AttachTo(&faceSegUnit);

		// Video Writer Unit
		VideoWriterOptions writer_options;
		VideoWriterUnit writer(writer_options, m_output_path);
		if (!m_output_path.empty())
		{
			writer.AttachTo(&faceSegRendererUnit);
		}

		if (!reader.PrepareProcessing())
			throw std::runtime_error("Video framework setup failed.");

		// Run with rate limitation.
		RatePolicy rate_policy;
		// Speed up playback for fun :)
		rate_policy.max_rate = 45;

		// This call will block and return when the whole has been displayed.
		if (!reader.RunRateLimited(rate_policy))
			throw std::runtime_error("Could not process video file.");
	}

	void FaceView::run_parallel()
	{
		std::vector<std::unique_ptr<VideoPipelineSource>> sources;
		std::vector<std::unique_ptr<VideoPipelineSink>> sinks;

		// Video Reader Unit
		VideoReaderOptions reader_options;
		std::unique_ptr<VideoReaderUnit> reader_unit(
			new VideoReaderUnit(reader_options, m_video_file));

		sinks.emplace_back(new VideoPipelineSink());
		sinks.back()->AttachTo(reader_unit.get());
		SourceRatePolicy srp;
		sources.emplace_back(new VideoPipelineSource(sinks.back().get(),
			nullptr, srp));

		// Segmentation Reader Unit
		SegmentationReaderUnitOptions seg_options;
		seg_options.filename = m_seg_file;
		std::unique_ptr<SegmentationReaderUnit> seg_reader(
			new SegmentationReaderUnit(seg_options));
		seg_reader->AttachTo(sources.back().get());

		// Landmarks Unit
		LandmarksOptions landmarks_options;
		landmarks_options.landmarks_model_file = m_landmarks_model_file;
		std::shared_ptr<LandmarksUnit> landmarks_unit = LandmarksUnit::create(landmarks_options);
		landmarks_unit->AttachTo(seg_reader.get());

		sinks.emplace_back(new VideoPipelineSink());
		sinks.back()->AttachTo(landmarks_unit.get());
		sources.emplace_back(new VideoPipelineSource(sinks.back().get()));

		// Face Segmentation Unit
		FaceSegmentationOptions face_seg_options;
		std::unique_ptr<FaceSegmentationUnit> face_seg_unit(
			new FaceSegmentationUnit(face_seg_options));
		face_seg_unit->AttachTo(sources.back().get());

		sinks.emplace_back(new VideoPipelineSink());
		sinks.back()->AttachTo(face_seg_unit.get());
		sources.emplace_back(new VideoPipelineSource(sinks.back().get()));

		// Face Segmentation Renderer Unit
		FaceSegmentationRendererOptions face_seg_renderer_options;
		std::unique_ptr<FaceSegmentationRendererUnit> face_seg_renderer_unit(
			new FaceSegmentationRendererUnit(face_seg_renderer_options));
		face_seg_renderer_unit->AttachTo(sources.back().get());

		// Video Writer Unit 2
		std::unique_ptr<VideoWriterUnit2> writer_unit;
		if (!m_output_path.empty())
		{
			VideoWriter2Options writer_options;
			writer_unit.reset(new VideoWriterUnit2(writer_options, m_output_path));
			writer_unit->AttachTo(face_seg_renderer_unit.get());
		}

		// Prepare processing
		if (!reader_unit->PrepareProcessing())
			LOG(ERROR) << "Setup failed.";

		// Start the threads
		RatePolicy pipeline_policy;
		pipeline_policy.max_rate = 20;
		pipeline_policy.dynamic_rate = true;
		//pipeline_policy.startup_frames = 10;
		pipeline_policy.update_interval = 1;
		//pipeline_policy.queue_throttle_threshold = 10;
		// Guarantee that buffers never go empty in non-camera mode.
		pipeline_policy.dynamic_rate_scale = 1.1;
		VideoPipelineInvoker invoker;

		// First source is run rate limited.
		invoker.RunRootRateLimited(pipeline_policy, reader_unit.get());

		// Run last source in main thread.
		for (int k = 0; k < sources.size() - 1; ++k) {
			invoker.RunPipelineSource(sources[k].get());
		}

		sources.back()->Run();

		invoker.WaitUntilPipelineFinished();

		LOG(INFO) << "__FACE_SEGMENTATION_FINISHED__";
	}

	void FaceView::test()
	{
		// Video Reader Unit
		VideoReaderUnit reader(VideoReaderOptions(), m_video_file);

		// Segmentation Reader Unit
		SegmentationReaderUnitOptions segOptions;
		segOptions.filename = m_seg_file;
		SegmentationReaderUnit seg_reader(segOptions);
		seg_reader.AttachTo(&reader);

		// Landmarks Unit
		LandmarksOptions landmarks_options;
		landmarks_options.landmarks_model_file = m_landmarks_model_file;
		std::shared_ptr<LandmarksUnit> landmarks_unit = LandmarksUnit::create(landmarks_options);
		landmarks_unit->AttachTo(&seg_reader);

		// Segmentation Renderer Unit
		SegmentationRenderUnitOptions seg_render_options;
		seg_render_options.blend_alpha = 0.4f;
		seg_render_options.hierarchy_level = 0.1f;
		SegmentationRenderUnit seg_render(seg_render_options);
		seg_render.AttachTo(landmarks_unit.get());

		// Landmarks Renderer Unit
		LandmarksRendererOptions landmarks_renderer_options;
		landmarks_renderer_options.video_stream_name = "RenderedRegionStream";
		LandmarksRendererUnit landmarks_renderer(landmarks_renderer_options);
		landmarks_renderer.AttachTo(&seg_render);
		
		// Video Display Unit
		VideoDisplayOptions video_display_options;
		video_display_options.stream_name = "RenderedRegionStream";
		VideoDisplayUnit display(video_display_options);
		display.AttachTo(&seg_render);

		// Prepare processing
		if (!reader.PrepareProcessing())
			throw std::runtime_error("Video framework setup failed.");

		// Run with rate limitation.
		RatePolicy rate_policy;
		// Speed up playback for fun :)
		rate_policy.max_rate = 45;

		// This call will block and return when the whole has been displayed.
		if (!reader.RunRateLimited(rate_policy))
			throw std::runtime_error("Could not process video file.");
	}

	void FaceView::test2()
	{
		std::vector<std::unique_ptr<VideoPipelineSource>> sources;
		std::vector<std::unique_ptr<VideoPipelineSink>> sinks;

		// Video Reader Unit
		VideoReaderUnit reader(VideoReaderOptions(), m_video_file);

		// Segmentation Reader Unit
		SegmentationReaderUnitOptions segOptions;
		segOptions.filename = m_seg_file;
		SegmentationReaderUnit seg_reader(segOptions);
		seg_reader.AttachTo(&reader);

		sinks.emplace_back(new VideoPipelineSink());
		sinks.back()->AttachTo(&seg_reader);
		sources.emplace_back(new VideoPipelineSource(sinks.back().get()));

		// Landmarks Unit
		LandmarksOptions landmarks_options;
		landmarks_options.landmarks_model_file = m_landmarks_model_file;
		std::shared_ptr<LandmarksUnit> landmarks_unit = LandmarksUnit::create(landmarks_options);
		landmarks_unit->AttachTo(sources.back().get());

		sinks.emplace_back(new VideoPipelineSink());
		sinks.back()->AttachTo(landmarks_unit.get());
		sources.emplace_back(new VideoPipelineSource(sinks.back().get()));

		// Segmentation Renderer Unit
		SegmentationRenderUnitOptions seg_render_options;
		seg_render_options.blend_alpha = 0.4f;
		seg_render_options.hierarchy_level = 0.1f;
		SegmentationRenderUnit seg_render(seg_render_options);
		seg_render.AttachTo(sources.back().get());

		// Landmarks Renderer Unit
		LandmarksRendererOptions landmarks_renderer_options;
		landmarks_renderer_options.video_stream_name = "RenderedRegionStream";
		LandmarksRendererUnit landmarks_renderer(landmarks_renderer_options);
		landmarks_renderer.AttachTo(&seg_render);

		// Video Display Unit
		VideoDisplayOptions video_display_options;
		video_display_options.stream_name = "RenderedRegionStream";
		VideoDisplayUnit display(video_display_options);
		display.AttachTo(&seg_render);

		// Prepare processing
		if (!reader.PrepareProcessing())
			throw std::runtime_error("Video framework setup failed.");

		VideoPipelineInvoker invoker;
		RatePolicy pipeline_policy;

		// Start threads
		invoker.RunRoot(&reader);

		// Run last source in main thread.
		for (int k = 0; k < sources.size() - 1; ++k) {
			invoker.RunPipelineSource(sources[k].get());
		}

		sources.back()->Run();

		invoker.WaitUntilPipelineFinished();
	}
	*/

	void FaceView::test3()
	{
		VideoUnit* input = nullptr;  // Updated throughout graph construction.

		boost::filesystem::path orig = m_video_file;
//		std::string seg_out_path = (boost::filesystem::path(m_output_dir) /
//			(orig.stem() += "_seg.mp4")).string();
		std::string debug1_out_path = (boost::filesystem::path(m_output_dir) /
			(orig.stem() += "_debug1.mp4")).string();
		std::string debug2_out_path = (boost::filesystem::path(m_output_dir) /
			(orig.stem() += "_debug2.mp4")).string();
		std::string stats_out_path = (boost::filesystem::path(m_output_dir) / 
			(orig.stem() += "_stats.txt")).string();

		// Face Segmentation Global
		FaceSegGlobalOptions face_seg_global_options;
		FaceSegGlobalUnit face_seg_global(face_seg_global_options);
		std::map<int, RegionStat>& region_stats = face_seg_global.getRegionStats();
		/*
		// First pass
		{
			// Video Reader Unit
			VideoReaderUnit reader(VideoReaderOptions(), m_video_file);

			// Segmentation Reader Unit
			SegmentationReaderUnitOptions segOptions;
			segOptions.filename = m_seg_file;
			SegmentationReaderUnit seg_reader(segOptions);
			seg_reader.AttachTo(&reader);

			// Landmarks Unit
			LandmarksOptions landmarks_options;
			landmarks_options.landmarks_path = m_landmarks_path;
			LandmarksUnit landmarks_unit(landmarks_options);
			landmarks_unit.AttachTo(&seg_reader);

			// Face Segmentation Global
			face_seg_global.AttachTo(&landmarks_unit);

			// Debug segmentation
			std::unique_ptr<SegmentationRenderUnit> seg_render;
			std::unique_ptr<LandmarksRendererUnit> landmarks_renderer;
			std::unique_ptr<VideoDisplayUnit> display;
			std::unique_ptr<VideoWriterUnit2> writer;
			if (m_verbose > 0)
			{
				// Face Segmentation Renderer Unit
				SegmentationRenderUnitOptions seg_render_options;
				seg_render_options.blend_alpha = 0.35f;
				seg_render_options.highlight_edges = false;
				seg_render_options.draw_shape_descriptors = true;
				seg_render_options.hierarchy_level = 0.1f;
				seg_render.reset(
					new SegmentationRenderUnit(seg_render_options));
				seg_render->AttachTo(&face_seg_global);

				// Landmarks Renderer Unit
				LandmarksRendererOptions landmarks_renderer_options;
				landmarks_renderer_options.video_stream_name = seg_render_options.out_stream_name;
				landmarks_renderer.reset(
					new LandmarksRendererUnit(landmarks_renderer_options));
				landmarks_renderer->AttachTo(seg_render.get());

				// Video Display Unit
				VideoDisplayOptions video_display_options;
				video_display_options.stream_name = seg_render_options.out_stream_name;
				display.reset(
					new VideoDisplayUnit(video_display_options));
				display->AttachTo(landmarks_renderer.get());

				// Video Writer Unit
				VideoWriter2Options writer_options;
				writer_options.video_stream_name = seg_render_options.out_stream_name;
				writer.reset(
					new VideoWriterUnit2(writer_options, debug1_out_path));
				if (!m_output_dir.empty())
				{
					writer->AttachTo(display.get());
				}
			}

			// Prepare processing
			if (!reader.PrepareProcessing())
				throw std::runtime_error("Video framework setup failed.");

			// Run with rate limitation.
			RatePolicy rate_policy;
			// Speed up playback for fun :)
			rate_policy.max_rate = 100;	// 45

			// This call will block and return when the whole has been displayed.
			if (!reader.RunRateLimited(rate_policy))
				throw std::runtime_error("Could not process video file.");
		}
		std::cout << "End of first pass" << std::endl;
		*/
		/*
		/// Debug ///
		for each (auto region_stat in region_stats)
		{
			// Calculate average
			float avg = 0;
			for (size_t i = 0; i < region_stat.second.ratios.size(); ++i)
				avg += region_stat.second.ratios[i];
			avg /= region_stat.second.ratios.size();

			if (avg > 0.0f)
			{
				std::cout << "region " << region_stat.first << ":" << std::endl;
				for (size_t i = 0; i < region_stat.second.ratios.size(); ++i)
					std::cout << region_stat.second.ratios[i] << " ";
				std::cout << std::endl;
			}
			
		}
		/////////////
		*/

		// Calculate statistics
		/*
		for (auto& region_stat : region_stats)
		{
			// Calculate average and max
			float avg = 0, r;
			for (size_t i = 0; i < region_stat.second.ratios.size(); ++i)
			{
				r = region_stat.second.ratios[i];
				avg += r;
				region_stat.second.max_ratio = std::max(region_stat.second.max_ratio, r);
			}
			if(region_stat.second.ratios.size() > 0)
				avg /= region_stat.second.ratios.size();
			region_stat.second.avg = avg;
		}
		*/


		// Write stats to text file
		std::ofstream ofs(stats_out_path);	
		for (auto& region_stat : region_stats)
		{
			ofs << "region " << region_stat.first << ":" << std::endl;
			for (size_t i = 0; i < region_stat.second.ratios.size(); ++i)
			{
				ofs << "(" << region_stat.second.frame_ids[i] << ", " << region_stat.second.ratios[i] << ") ";
			}
			ofs << std::endl;
		}
		ofs.close();

		// Second pass
		{
			// Video Reader Unit
			VideoReaderUnit reader(VideoReaderOptions(), m_video_file);

			// Segmentation Reader Unit
			SegmentationReaderUnitOptions segOptions;
			segOptions.filename = m_seg_file;
			SegmentationReaderUnit seg_reader(segOptions);
			seg_reader.AttachTo(&reader);

			// Landmarks Unit
			LandmarksOptions landmarks_options;
			landmarks_options.landmarks_path = m_landmarks_path;
			LandmarksUnit landmarks_unit(landmarks_options);
			landmarks_unit.AttachTo(&seg_reader);

			// Face Segmentation Local Unit
			FaceSegLocalOptions face_seg_local_options;
			FaceSegLocalUnit face_seg_local(face_seg_local_options, region_stats);
			face_seg_local.AttachTo(&landmarks_unit);

			// Face Segmentation Renderer Unit
			FaceSegmentationRendererOptions face_seg_renderer_options;
			face_seg_renderer_options.face_segment_stream_name = face_seg_local_options.stream_name;
			FaceSegmentationRendererUnit face_seg_renderer(face_seg_renderer_options, region_stats);
			face_seg_renderer.AttachTo(&face_seg_local);
            input = &face_seg_local;
            /*
			// Video Display Unit
			VideoDisplayOptions video_display_options;
			video_display_options.stream_name = face_seg_renderer_options.stream_name;
			VideoDisplayUnit display(video_display_options);
			display.AttachTo(&face_seg_renderer);

			// Video Writer Unit
			VideoWriter2Options writer_options;
			writer_options.video_stream_name = face_seg_renderer_options.stream_name;
			VideoWriterUnit2 writer(writer_options, seg_out_path);
			if (!m_output_dir.empty())
			{
				writer.AttachTo(&display);
			}
			input = &writer;
            */

			// Debug segmentation
			std::unique_ptr<FaceSegmentationRendererUnit> face_seg_renderer_debug;
			std::unique_ptr<VideoDisplayUnit> face_seg_display_debug;
			std::unique_ptr<VideoWriterUnit2> face_seg_writer_debug;
			if (m_verbose > 1)
			{
				// Face Segmentation Renderer Unit
				FaceSegmentationRendererOptions face_seg_renderer_options;
				face_seg_renderer_options.stream_name = "FaceSegmentationRendererDebug";
				face_seg_renderer_options.face_segment_stream_name = face_seg_local_options.stream_name;
				face_seg_renderer_options.debug = true;
				face_seg_renderer_debug.reset(
					new FaceSegmentationRendererUnit(face_seg_renderer_options));
				face_seg_renderer_debug->AttachTo(input);

				// Video Display Unit
				VideoDisplayOptions video_display_options;
				video_display_options.stream_name = face_seg_renderer_options.stream_name;
				face_seg_display_debug.reset(
					new VideoDisplayUnit(video_display_options));
				face_seg_display_debug->AttachTo(face_seg_renderer_debug.get());

				// Video Writer Unit
				VideoWriter2Options writer_options;
				writer_options.video_stream_name = face_seg_renderer_options.stream_name;
				face_seg_writer_debug.reset(
					new VideoWriterUnit2(writer_options, debug2_out_path));
				if (!m_output_dir.empty())
				{
					face_seg_writer_debug->AttachTo(face_seg_display_debug.get());
				}
				input = face_seg_writer_debug.get();
			}

			// Keyframe writer
			KeyframeWriterOptions keyframe_writer_options;
            keyframe_writer_options.face_segment_stream_name = face_seg_local_options.stream_name;
            keyframe_writer_options.debug = m_verbose > 1;
			KeyframeWriter keyframe_writer(keyframe_writer_options, m_output_dir,
				orig.stem().string());
			if (!m_output_dir.empty())
				keyframe_writer.AttachTo(input);

			// Prepare processing
			if (!reader.PrepareProcessing())
				throw std::runtime_error("Video framework setup failed.");

			// Run with rate limitation.
			RatePolicy rate_policy;
			// Speed up playback for fun :)
			rate_policy.max_rate = 45;

			// This call will block and return when the whole has been displayed.
			if (!reader.RunRateLimited(rate_policy))
				throw std::runtime_error("Could not process video file.");
		}
		std::cout << "End of second pass" << std::endl;
	}
}   // namespace fvs