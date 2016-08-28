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

#include "editor.h"
#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

#include <video_framework/video_reader_unit.h>
#include <video_framework/video_display_unit.h>
#include <video_framework/video_writer_unit.h>
#include <video_framework/video_pipeline.h>
#include <segmentation/segmentation_unit.h>
#include <video_display_qt/video_display_qt_unit.h>

#include <QApplication>

using std::string;
using std::cout;
using std::endl;
using std::cerr;
using namespace boost::program_options;
using namespace boost::filesystem;
using namespace video_framework;
using namespace segmentation;

int main(int argc, char* argv[])
{
	// Parse command line arguments
	string fvsPath, outputDir, segPath, landmarksPath, videoPath;
	int device;
	unsigned int width, height, verbose;
	double fps, frame_scale;
	bool preview;
	try {
		options_description desc("Allowed options");
		desc.add_options()
			("help", "display the help message")
			("input,i", value<string>(&fvsPath), "path to face video segmentation (.fvs)")
			("output,o", value<string>(&outputDir), "output directory")
			("segmentation,s", value<string>(&segPath), "input segmentation protobuffer (.pb)")
            ("landmarks,l", value<string>(&landmarksPath), "path to landmarks cache (.pb)")
			("video,v", value<string>(&videoPath)->default_value(""), "path to video file")
			;
		variables_map vm;
		store(command_line_parser(argc, argv).options(desc).
			positional(positional_options_description().add("input", -1)).run(), vm);
		if (vm.count("help")) {
			cout << "Usage: face_video_segment [options]" << endl;
			cout << desc << endl;
			exit(0);
		}
		notify(vm);
		if (!is_regular_file(fvsPath)) throw error("input must be a path to a file!");
		if (vm.count("output") && !is_directory(outputDir))
			throw error("output must be a path to a directory!");
        /*
		if (!is_regular_file(segPath)) throw error("segmentation must be a path to a file!");
        if (!is_regular_file(landmarksPath))
        {
            path input = path(inputPath);
            landmarksPath =
                (input.parent_path() / (input.stem() += "_landmarks.pb")).string();
            if (!is_regular_file(landmarksPath))
                throw error("Couldn't find landmarks model or cache file!");
        }
        */
	}
	catch (const error& e) {
		cout << "Error while parsing command-line arguments: " << e.what() << endl;
		cout << "Use --help to display a list of options." << endl;
		exit(1);
	}

	//try
	{
        Q_INIT_RESOURCE(fvs_editor);

        QApplication app(argc, argv);
        fvs::Editor editor(fvsPath, outputDir, videoPath, segPath, landmarksPath);
        editor.show();
        return app.exec();
        /*
        // Video Reader Unit
        VideoReaderUnit reader(VideoReaderOptions(), inputPath);

        // Segmentation Reader Unit
        SegmentationReaderUnitOptions segOptions;
        segOptions.filename = segPath;
        SegmentationReaderUnit seg_reader(segOptions);
        seg_reader.AttachTo(&reader);

        // Video display
        SegmentationDisplayOptions seg_display_options;
        seg_display_options.hierarchy_level = 0.1f;
        std::unique_ptr<SegmentationDisplayUnit> display;
        display.reset(new SegmentationDisplayUnit(seg_display_options));
        display->AttachTo(&reader);

        //std::unique_ptr<VideoDisplayQtUnit> display;
        //display.reset(new VideoDisplayQtUnit(VideoDisplayQtOptions()));
        //display->AttachTo(&reader);

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
            */
	}
    /*
	catch (std::exception& e)
	{
		cerr << e.what() << endl;
		return 1;
	}*/

	return 0;
}