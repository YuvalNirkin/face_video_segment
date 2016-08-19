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

// std
#include <exception>
#include <iostream> // debug

// sfl
#include <sfl/sequence_face_landmarks.h>
#include <sfl/utilities.h>

// segmentation
#include <segment_util/segmentation_io.h>
#include <segment_util/segmentation_util.h>
#include <segment_util/segmentation_render.h>

// OpenCV
#include <opencv2/highgui.hpp>

// Qt
#include <QLabel>
#include <QSlider>
#include <QGridLayout>
#include <QMouseEvent>
#include <QCoreApplication>

namespace fvs
{
    Editor::Editor(const std::string & video_file, const std::string & seg_file,
        const std::string& landmarks_file, const std::string & output_dir) :
        m_loop(true),
        m_refresh(true),
        m_slider_pause(false),
        m_update_pending(false),
        m_curr_frame_ind(0),
        m_frame_width(0), m_frame_height(0),
        m_fps(0),
        m_total_frames(0),
        m_main_face_id(0),
        m_hierarchy_pos(0)
    {
        // Initialize video capture
        m_cap.reset(new cv::VideoCapture());
        if (!m_cap->open(video_file))
            throw std::runtime_error("Failed to open video file!");

        m_frame_width = (int)m_cap->get(cv::CAP_PROP_FRAME_WIDTH);
        m_frame_height = (int)m_cap->get(cv::CAP_PROP_FRAME_HEIGHT);
        m_fps = m_cap->get(cv::CAP_PROP_FPS);
        m_total_frames = (size_t)m_cap->get(cv::CAP_PROP_FRAME_COUNT);
        m_scaled_frame.reset(new cv::Mat(m_frame_height, m_frame_width, CV_8UC3));

        // Make Qt image.
        m_render_image.reset(new QImage((const uint8_t*)m_scaled_frame->data,
            m_scaled_frame->cols,
            m_scaled_frame->rows,
            m_scaled_frame->step[0],
            QImage::Format_RGB888));

        // Initialize sequence face landmarks
        m_sfl = sfl::SequenceFaceLandmarks::create(landmarks_file);
        if(m_sfl->size() < m_total_frames)
            throw std::runtime_error(
                "The number of landmark frames does not match the number of video frames!");
        const std::list<std::unique_ptr<sfl::Frame>>& sfl_frames_list =  m_sfl->getSequence();
        m_main_face_id = sfl::getMainFaceID(sfl_frames_list);
        m_sfl_frames.reserve(sfl_frames_list.size());
        for (auto& frame : sfl_frames_list)
            m_sfl_frames.push_back(frame.get());  

        // Initialize segmentation reader
        m_seg_reader.reset(new segmentation::SegmentationReader(seg_file));
        if (!m_seg_reader->OpenFileAndReadHeaders())
            throw std::runtime_error("Failed to read segmentation file!");
        m_seg_desc.reset(new segmentation::SegmentationDesc);
        m_seg_hierarchy.reset(new segmentation::SegmentationDesc);
        m_seg_reader->ReadNextFrame(m_seg_hierarchy.get());

        // Create main widget
        m_main_widget = new QLabel(this);
        setCentralWidget(m_main_widget);
        setWindowTitle("Face Video Segmentation Editor");

        // Create simple widget used for display.
        m_display_widget = new QLabel(this);
        m_display_widget->installEventFilter(this);
        

        // Frame slider
        m_frame_slider = new QSlider(Qt::Horizontal);
        m_frame_slider->setMinimum(0);
        m_frame_slider->setMaximum(m_total_frames - 1);
        m_frame_slider->setTickPosition(QSlider::NoTicks);
        m_frame_slider->setValue(m_curr_frame_ind);
        connect(m_frame_slider, SIGNAL(valueChanged(int)), this, SLOT(frameIndexChanged(int)));
        connect(m_frame_slider, SIGNAL(sliderPressed()), this, SLOT(frameSliderPress()));
        connect(m_frame_slider, SIGNAL(sliderReleased()), this, SLOT(frameSliderRelease()));

        // Hierarchy level slider.
        m_max_hierarchy_level = 20;
        m_curr_hierarchy_level = 0;
        m_hierarchy_slider = new QSlider(Qt::Horizontal);
        m_hierarchy_slider->setMinimum(0);
        m_hierarchy_slider->setMaximum(m_max_hierarchy_level);
        m_hierarchy_slider->setTickPosition(QSlider::TicksBelow);
        m_hierarchy_slider->setValue(m_curr_hierarchy_level);
        //connect(m_hierarchy_slider, SIGNAL(sliderMoved(int)), this, SLOT(ChangeLevel(int)));
        connect(m_hierarchy_slider, SIGNAL(valueChanged(int)), this, SLOT(hierarchyLevelChanged(int)));

        // GUI layout.
        QGridLayout* centralLayout = new QGridLayout;
        centralLayout->addWidget(m_display_widget);
        centralLayout->addWidget(m_frame_slider);
        centralLayout->addWidget(m_hierarchy_slider);
        centralLayout->setAlignment(Qt::Alignment(Qt::AlignTop));
        m_main_widget->setLayout(centralLayout);

        // Resize
        const int border = 11 * 2;
        //m_main_widget->resize(m_frame_width + border, m_frame_height + 4 * border);
        //resize(m_frame_width + border, m_frame_height + 4 * border);
        //m_display_widget->resize(m_frame_width, m_frame_height);

        QSize frame_slider_size = m_frame_slider->minimumSizeHint();
        QSize hierarchy_slider_size = m_frame_slider->minimumSizeHint();
        QSize window_size(m_frame_width + border, m_frame_height + 2*border +
            frame_slider_size.height() + hierarchy_slider_size.height());
        m_display_widget->resize(m_frame_width, m_frame_height);
        m_display_widget->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
        m_main_widget->resize(window_size);
        resize(window_size);

    }

    Editor::~Editor()
    {
    }

    bool Editor::event(QEvent * event)
    {
        switch (event->type())
        {
        case QEvent::UpdateRequest:
            m_update_pending = false;
            update();
            return QMainWindow::event(event);
        default:
            return QMainWindow::event(event);
        };
    }

    bool Editor::eventFilter(QObject * object, QEvent * event)
    {
        //std::cout << "eventFilter" << std::endl;

        if (object == m_display_widget && event->type() == QEvent::MouseButtonPress)
        {
            QMouseEvent *mouseEvent = static_cast<QMouseEvent*>(event);
            QPoint pos = mouseEvent->pos();
            std::cout << "pos = (" << pos.x() << ", " << pos.y() << ")" << std::endl;//
            int id = segmentation::GetOversegmentedRegionIdFromPoint(pos.x(), pos.y(), *m_seg_desc);//
            int parent_id = segmentation::GetParentId(id, 0, m_curr_hierarchy_level, m_seg_hierarchy->hierarchy());
            std::cout << "oversegmented region id = " << id << std::endl;
            std::cout << "parent region id = " << parent_id << std::endl;
            return true;
        }
        return false;

        //return QMainWindow::eventFilter(object, event);
    }

    void Editor::keyPressEvent(QKeyEvent * event)
    {
        switch (event->key())
        {
        case Qt::Key_Escape:
            close(); break;
        case Qt::Key_Space:
            pause(m_loop); break;
        default: break;
        }
    }

    void Editor::update()
    {
        if (!m_refresh) return;
        //std::cout << "update" << std::endl;
        m_curr_frame_ind = (int)m_cap->get(cv::CAP_PROP_POS_FRAMES);
        m_frame_slider->setValue(m_curr_frame_ind);
        //std::cout << "(" << m_curr_frame_ind << ", " << m_total_frames - 1 << ")" << std::endl;
        if (m_curr_frame_ind < m_total_frames && m_cap->read(*m_scaled_frame))
        {
            // Update segmentation
            m_seg_reader->SeekToFrame(m_curr_frame_ind);
            m_seg_reader->ReadNextFrame(m_seg_desc.get());

            // Update hierarchy if necessary.
            if (m_hierarchy_pos != m_seg_desc->hierarchy_frame_idx()) {
                m_hierarchy_pos = m_seg_desc->hierarchy_frame_idx();
                m_seg_reader->SeekToFrame(m_hierarchy_pos);
                m_seg_reader->ReadNextFrame(m_seg_hierarchy.get());
                //std::cout << "hierarchy size = " << m_seg_hierarchy->hierarchy_size() << std::endl;//
            }

            // Render
            render(*m_scaled_frame);
            m_display_widget->setPixmap(QPixmap::fromImage(m_render_image->rgbSwapped()));
            m_display_widget->update();
            if (m_loop) updateLater();
            else m_refresh = false;
        }
        else pause(true);
    }

    void Editor::updateLater()
    {
        if (!m_update_pending) {
            m_update_pending = true;
            QCoreApplication::postEvent(this, new QEvent(QEvent::UpdateRequest));
        }
    }

    void Editor::seek(int index)
    {
        m_cap->set(cv::CAP_PROP_POS_FRAMES, (double)index);
        m_refresh = true;
    }

    void Editor::pause(bool pause)
    {
        m_loop = !pause;
        m_refresh = m_loop;
        if (m_loop) updateLater();
    }

    void Editor::render(cv::Mat& frame)
    {
        // Render segmentation at specified level.
        segmentation::RenderRegionsRandomColor(m_curr_hierarchy_level,
            true,
            false,
            *m_seg_desc,
            &m_seg_hierarchy->hierarchy(),
            &frame);

        // Render landmarks
        const sfl::Face* main_face = m_sfl_frames[m_curr_frame_ind]->getFace(m_main_face_id);
        if(main_face != nullptr)
            sfl::render(frame, main_face->landmarks);
    }

    void Editor::frameIndexChanged(int n)
    {
        //m_curr_frame_ind = n;
        //m_frame_slider->setValue(m_curr_frame_ind);
        seek(n);
    }  

    void Editor::hierarchyLevelChanged(int n) {
        m_curr_hierarchy_level = n;
        m_hierarchy_slider->setValue(m_curr_hierarchy_level);
        m_refresh = true;
    }

    void Editor::frameSliderPress()
    {
        std::cout << "press" << std::endl;
        m_slider_pause = !m_loop;
        pause(true);
    }

    void Editor::frameSliderRelease()
    {
        std::cout << "release" << std::endl;
        pause(m_slider_pause);
        m_slider_pause = false;
    }
}   // namespace fvs