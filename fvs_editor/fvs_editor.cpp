#include "fvs_editor.h"
//#include <vsal/VideoStreamFactory.h>
#include <sfl/utilities.h>

#include <exception>
#include <iostream>//

// Boost
#include <boost/filesystem.hpp>

// OpenCV
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// Qt
#include <QEventTransition>
#include <QKeyEventTransition>
#include <QSignalTransition>
#include <QFileDialog>
#include <QResizeEvent>
#include <QMessageBox>//

using namespace boost::filesystem;

namespace fvs
{
    Editor::Editor() : 
        sm(this)
    {
        setupUi(this);
        setupBl();
    }

    void Editor::setupBl()
    {
        // Initialize state machine
        sm.initiate();

        // Connect actions
        connect(action_Open, &QAction::triggered, this, &Editor::open);
        connect(action_Close, &QAction::triggered, this, &Editor::close);
        connect(actionPlay, &QAction::triggered, this, &Editor::playPause);
        connect(actionBackward, &QAction::triggered, this, &Editor::backward);
        connect(actionForward, &QAction::triggered, this, &Editor::forward);
        connect(frame_slider, SIGNAL(valueChanged(int)), this, SLOT(frameSliderChanged(int)));
        connect(actionShowLandmarks, SIGNAL(toggled(bool)), this, SLOT(toggleRenderParams(bool)));
        connect(actionShowBBox, SIGNAL(toggled(bool)), this, SLOT(toggleRenderParams(bool)));
        connect(actionShowIDs, SIGNAL(toggled(bool)), this, SLOT(toggleRenderParams(bool)));
        connect(actionShowLabels, SIGNAL(toggled(bool)), this, SLOT(toggleRenderParams(bool)));

        play_pause_btn->setDefaultAction(actionPlay);
        backward_btn->setDefaultAction(actionBackward);
        forward_btn->setDefaultAction(actionForward);
        keyframe_btn->setDefaultAction(actionKeyframe);

        // Adjust window size
        adjustSize();
    }

    void Editor::setInputPath(const std::string & input_path)
    {
        if (path(input_path).extension() == ".lms")
            initLandmarks(input_path);
        else initVideoSource(input_path);
    }

    void Editor::initLandmarks(const std::string & _landmarks_path)
    {
    }

    void Editor::initVideoSource(const std::string & _sequence_path)
    {
    }

    void Editor::resizeEvent(QResizeEvent* event)
    {
        QMainWindow::resizeEvent(event);

        QSize displaySize = display->size();
        render_frame = cv::Mat::zeros(displaySize.height(), displaySize.width(), CV_8UC3);

        // Make Qt image.
        render_image.reset(new QImage((const uint8_t*)render_frame.data,
            render_frame.cols,
            render_frame.rows,
            render_frame.step[0],
            QImage::Format_RGB888));

        sm.process_event(EvUpdate());
    }

    void Editor::timerEvent(QTimerEvent *event)
    {
        sm.process_event(EvTimerTick());
    }

    void Editor::open()
    {
        QString file = QFileDialog::getOpenFileName(
            this,
            "Select one or more files to open",
            QString(),
            "Landmarks (*.lms);;Videos (*.mp4 *.mkv *.avi *wmv);;All files (*.*)",
            nullptr);
        setInputPath(file.toStdString());
    }

    void Editor::close()
    {
        QMainWindow::close();
    }

    void Editor::playPause()
    {
        sm.process_event(EvPlayPause());
    }

    void Editor::backward()
    {
        sm.process_event(EvSeek(curr_frame_pos - 1));
    }

    void Editor::forward()
    {
        sm.process_event(EvSeek(curr_frame_pos + 1));
    }

    void Editor::frameSliderChanged(int i)
    {
        sm.process_event(EvSeek(i));
    }

    void Editor::toggleRenderParams(bool toggled)
    {
        sm.process_event(EvUpdate());
    }

    void Editor::render()
    {
    }

}   // namespace fvs

