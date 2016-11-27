#include "fvs_editor.h"

// std
#include <iostream>
#include <exception>

// Boost
#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

// Qt
#include <QApplication>

using std::cout;
using std::endl;
using std::cerr;
using std::string;
using std::runtime_error;
using namespace boost::program_options;
using namespace boost::filesystem;


int main(int argc, char* argv[])
{
    /// Parse command line arguments
    string fvsPath, outputDir, segPath, landmarksPath, videoPath;
    unsigned int debug;
    try {
        options_description desc("Allowed options");
        desc.add_options()
            ("help", "display the help message")
            ("input,i", value<string>(&fvsPath)->required(),
                "path to face video segmentation (.fvs)")
                ("output,o", value<string>(&outputDir), "output directory")
            ("segmentation,s", value<string>(&segPath), "input segmentation protobuffer (.pb)")
            ("landmarks,l", value<string>(&landmarksPath), "path to landmarks cache (.pb)")
            ("video,v", value<string>(&videoPath), "path to video file")
            ("debug,d", value<unsigned int>(&debug)->default_value(0), "output debug information")
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
    }
    catch (const error& e) {
        cout << "Error while parsing command-line arguments: " << e.what() << endl;
        cout << "Use --help to display a list of options." << endl;
        exit(1);
    }

    try
    {
        QApplication a(argc, argv);
        fvs::Editor editor;
        editor.show();

        return a.exec();
    }
    catch (std::exception& e)
    {
        cerr << e.what() << endl;
        return 1;
    }

    return 0;
}