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
#include "video_reader_unit2.h"
#include "video_writer_unit2.h"
#include "keyframe_unit.h"
#include <video_framework/video_reader_unit.h>
#include <video_framework/video_display_unit.h>
#include <video_framework/video_writer_unit.h>
#include <video_framework/video_pipeline.h>
#include <segmentation/segmentation_unit.h>

#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

using std::string;
using std::cout;
using std::endl;
using std::cerr;
using namespace boost::program_options;
using namespace boost::filesystem;
using namespace fvs;

int main(int argc, char* argv[])
{
	// Parse command line arguments
	string fvsPath, outputDir, segPath, landmarksPath, videoPath;
	unsigned int max_scale, debug;
    bool upscale;
	try {
		options_description desc("Allowed options");
        desc.add_options()
            ("help", "display the help message")
            ("input,i", value<string>(&fvsPath)->required(), "path to face video segmentation (.fvs)")
            ("output,o", value<string>(&outputDir)->required(), "output directory")
            ("segmentation,s", value<string>(&segPath), "input segmentation protobuffer (.pb)")
            ("landmarks,l", value<string>(&landmarksPath), "path to landmarks cache (.lms)")
            ("video,v", value<string>(&videoPath), "path to video file")
            ("max_scale,m", value<unsigned int>(&max_scale)->default_value(500),
                "max keyframe scale [pixels]")
            ("upscale,u", value<bool>(&upscale)->default_value(false)->implicit_value(true),
                "upscale to max scale")
            ("debug,d", value<unsigned int>(&debug)->default_value(0), "output debug information")
            ;
		variables_map vm;
		store(command_line_parser(argc, argv).options(desc).
			positional(positional_options_description().add("input", -1)).run(), vm);
		if (vm.count("help")) {
			cout << "Usage: fvs_write_keyframes [options]" << endl;
			cout << desc << endl;
			exit(0);
		}
		notify(vm);

        // Read fvs file to extract the stored paths
        fvs::Sequence sequence;
        std::ifstream input(fvsPath, std::ifstream::binary);
        sequence.ParseFromIstream(&input);

        path inputPath = path(fvsPath);
		if (!is_regular_file(fvsPath)) 
            throw error("input must be a path to a face video segmentation file!");
		if (outputDir.empty() || !is_directory(outputDir))
			throw error("output must be a path to a directory!");
        if (videoPath.empty()) videoPath = sequence.video_path();
        if (!is_regular_file(videoPath))
            throw error("Couldn't find video file!");
        if (segPath.empty()) segPath = sequence.seg_path();
        if (!is_regular_file(segPath))
        {
            segPath = (inputPath.parent_path() / (inputPath.filename() += ".pb")).string();
            if (!is_regular_file(segPath))
                throw error("Couldn't find segmentation file!");
        }
        if (landmarksPath.empty()) landmarksPath = sequence.landmarks_path();
        if (!is_regular_file(landmarksPath))
        {
            landmarksPath =
                (inputPath.parent_path() / (inputPath.stem() += ".lms")).string();
            if (!is_regular_file(landmarksPath))
                throw error("Couldn't find landmarks cache file!");
        }
	}
	catch (const error& e) {
		cout << "Error while parsing command-line arguments: " << e.what() << endl;
		cout << "Use --help to display a list of options." << endl;
		exit(1);
	}

	try
	{
        // Video Reader Unit
        VideoReaderUnit reader(VideoReaderOptions(), videoPath);
        //VideoReaderUnit2 reader(VideoReader2Options(), videoPath);

        // Segmentation Reader Unit
        SegmentationReaderUnitOptions segOptions;
        segOptions.filename = segPath;
        SegmentationReaderUnit seg_reader(segOptions);
        seg_reader.AttachTo(&reader);

        // Landmarks Unit
        LandmarksOptions landmarks_options;
        landmarks_options.landmarks_path = landmarksPath;
        LandmarksUnit landmarks_unit(landmarks_options);
        landmarks_unit.AttachTo(&seg_reader);

        // Face Regions Unit
        FaceRegionsReaderOptions face_regions_options;
        FaceRegionsReaderUnit face_regions_unit(face_regions_options, fvsPath);
        face_regions_unit.AttachTo(&landmarks_unit);
        /*
        FaceRegionsOptions face_regions_options;
        FaceRegionsUnit face_regions_unit(face_regions_options);
        face_regions_unit.AttachTo(&landmarks_unit);

        // Keyframe Detection Unit
        KeyframeDetectionOptions keyframe_detection_options;
        KeyframeDetectionUnit keyframe_detection_unit(keyframe_detection_options);
        keyframe_detection_unit.AttachTo(&face_regions_unit);
        */

        // Keyframe Writer Unit
        KeyframeWriterOptions keyframe_writer_options;
        keyframe_writer_options.max_scale = max_scale;
        keyframe_writer_options.upscale = upscale;
        keyframe_writer_options.debug = debug > 0;
        KeyframeWriterUnit keyframe_writer_unit(keyframe_writer_options, outputDir,
            path(videoPath).stem().string());
        keyframe_writer_unit.AttachTo(&face_regions_unit);

        // Prepare processing
        if (!reader.PrepareProcessing())
            throw std::runtime_error("Video framework setup failed.");

        // This call will block and return when the whole has been displayed.
        if (!reader.Run())
            throw std::runtime_error("Could not process video file.");
	}
	catch (std::exception& e)
	{
		cerr << e.what() << endl;
		return 1;
	}

	return 0;
}