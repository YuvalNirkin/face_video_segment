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
    string videoPath, outputDir, segPath, landmarksPath;
    unsigned int verbose;
	try {
        options_description desc("Allowed options");
        desc.add_options()
            ("help", "display the help message")
            ("input,i", value<string>(&videoPath)->required(), "path to video file")
            ("output,o", value<string>(&outputDir)->required(), "output directory")
            ("segmentation,s", value<string>(&segPath), "input segmentation protobuffer (.pb)")
            ("landmarks,l", value<string>(&landmarksPath), "path to landmarks cache (.lms)")
            ("verbose,v", value<unsigned int>(&verbose)->default_value(0), "output debug information")
            ;
        variables_map vm;
        store(command_line_parser(argc, argv).options(desc).
            positional(positional_options_description().add("input", -1)).run(), vm);
        if (vm.count("help")) {
            cout << "Usage: fvs_find_regions [options]" << endl;
            cout << desc << endl;
            exit(0);
        }
        notify(vm);

        if (!is_regular_file(videoPath)) throw error("input must be a path to a video file!");
        if (vm.count("output") && !is_directory(outputDir))
            throw error("output must be a path to a directory!");
        if (!is_regular_file(segPath)) throw error("segmentation must be a path to a file!");
        if (!is_regular_file(landmarksPath))
        {
            path input = path(videoPath);
            landmarksPath =
                (input.parent_path() / (input.stem() += ".lms")).string();
            if (!is_regular_file(landmarksPath))
                throw error("Couldn't find landmarks model or cache file!");
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
        //VideoReaderUnit reader(VideoReaderOptions(), videoPath);
        VideoReaderUnit2 reader(VideoReader2Options(), videoPath);

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
        FaceRegionsOptions face_regions_options;
        face_regions_options.video_path = videoPath;
        face_regions_options.seg_path = segPath;
        face_regions_options.landmarks_path = landmarksPath;
        FaceRegionsUnit face_regions_unit(face_regions_options);
        face_regions_unit.AttachTo(&landmarks_unit);

        // Keyframe Detection Unit
        KeyframeDetectionOptions keyframe_detection_options;
        KeyframeDetectionUnit keyframe_detection_unit(keyframe_detection_options);
        keyframe_detection_unit.AttachTo(&face_regions_unit);

        // Prepare processing
        if (!reader.PrepareProcessing())
            throw std::runtime_error("Video framework setup failed.");

        // This call will block and return when the whole has been displayed.
        if (!reader.Run())
            throw std::runtime_error("Could not process video file.");

        // Write output to file
        boost::filesystem::path orig = videoPath;
        std::string fvs_out_path = (boost::filesystem::path(outputDir) /
            (orig.stem() += ".fvs")).string();
        std::cout << "Writing face video segmentation to \"" <<
            fvs_out_path << "\"." << std::endl;
        face_regions_unit.save(fvs_out_path);
	}
	catch (std::exception& e)
	{
		cerr << e.what() << endl;
		return 1;
	}

	return 0;
}