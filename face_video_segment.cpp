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
#include <iostream>
#include <string>
#include <boost/program_options.hpp>

using std::string;
using std::cout;
using std::endl;
using std::cerr;
using namespace boost::program_options;

int main(int argc, char* argv[])
{
	// Parse command line arguments
	string inputPath, outputPath, segPath, landmarksModelPath;
	int device;
	unsigned int width, height, verbose;
	double fps, frame_scale;
	bool preview;
	try {
		options_description desc("Allowed options");
		desc.add_options()
			("help", "display the help message")
			("input,i", value<string>(&inputPath), "path to video file")
			("output,o", value<string>(&outputPath), "output path")
			("segmentation,s", value<string>(&segPath), "input segmentation protobuffer (.pb)")
			("landmarks,l", value<string>(&landmarksModelPath), "path to landmarks model file")
			("verbose,v", value<unsigned int>(&verbose)->default_value(0), "output debug information")
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
	}
	catch (const error& e) {
		cout << "Error while parsing command-line arguments: " << e.what() << endl;
		cout << "Use --help to display a list of options." << endl;
		exit(1);
	}

	try
	{
		segmentation::FaceView viewer(inputPath, segPath, landmarksModelPath, outputPath);
		viewer.run();
	}
	catch (std::exception& e)
	{
		cerr << e.what() << endl;
		return 1;
	}

	return 0;
}
/*
DEFINE_string(input, "", "The input segmentation protobuffer (.pb). REQUIRED");
DEFINE_string(window_name, "", "Use different window names to support "
                               "multiple viewer sessions.");

int main(int argc, char** argv) {
  // Initialize Google's logging library.
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);

  if (FLAGS_input.empty()) {
    std::cout << "Usage: segment_viewer -input=FILE_NAME\n";
    return 1;
  }

  std::string window_name;
  if (FLAGS_window_name.empty()) {
    window_name = argv[2];
  } else {
    // if not set as the filename
    size_t temp = FLAGS_input.find_last_of("/\\");
    window_name = FLAGS_input.substr(temp+1);
  }

  Viewer viewer(FLAGS_input, window_name);
  if (!viewer.ReadFile()) {
    LOG(ERROR) << "Could not read file!";
    return 1;
  }

  viewer.CreateUI();
  viewer.Run();

   return 0;
}
*/