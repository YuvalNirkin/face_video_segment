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

// face segmentation
#include <utilities.h>
#include <face_video_segment.pb.h>
#include <keyframer.h>

// OpenCV
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

// Qt
#include <QLabel>
#include <QSlider>
#include <QGridLayout>
#include <QBoxLayout>
#include <QMouseEvent>
#include <QCoreApplication>
#include <QStyle>
#include <QToolButton>

namespace fvs
{
    Editor::Editor(const std::string & video_file, const std::string & seg_file,
        const std::string& landmarks_file, const std::string & output_dir) :
        m_loop(false),
        m_refresh(true),
        m_slider_pause(false),
        m_update_pending(false),
        m_update_frame(true),
        m_curr_frame_ind(0),
        m_next_frame_ind(-1),
        m_frame_width(0), m_frame_height(0),
        m_fps(0),
        m_total_frames(0),
        m_main_face_id(0),
        m_hierarchy_pos(0),
        m_edit_index(-1)
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
        m_render_frame.reset(new cv::Mat(m_frame_height, m_frame_width, CV_8UC3));

        // Make Qt image.
        m_render_image.reset(new QImage((const uint8_t*)m_render_frame->data,
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

        // Initialize face segmentation
        m_sequence_regions.reset(new Sequence());
        m_edited_regions.reset(new Sequence());
        m_face_boundary.reset(new std::vector<std::vector<cv::Point>>());
        m_face_boundary->resize(1);
        m_face_map.reset(new cv::Mat(m_frame_height, m_frame_width, CV_8U));

        // For each frame in the sequence
        for (unsigned int i = 0; i < m_total_frames; ++i)
        {
            Frame* frame = m_sequence_regions->add_frames();
            frame->set_id(0);
            frame->set_width(m_frame_width);
            frame->set_height(m_frame_height);
            auto& faces = *frame->mutable_faces();
            Face& face = faces[m_main_face_id];
            face.set_id(m_main_face_id);
        }

        // Initialize keyframer
        m_keyframer = std::make_unique<Keyframer>(15, 5);
        for (auto& frame : m_sfl->getSequence())
        {
            const sfl::Face* face = frame->getFace(m_main_face_id);
            if (m_keyframer->addFrame(face->landmarks))
                m_keyframes.push_back(frame->id);
            
        }
        /// Debug keyframes ///
        //std::cout << "keyframes = " << std::endl;
        //for (int id : m_keyframes) std::cout << id << ", ";
        //std::cout << std::endl;
        ///////////////////////
        
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
        
        // Create labels
        m_frame_label = new QLabel(this);
        m_frame_label->setText("Frame: ");
        m_frame_label->setAlignment(Qt::AlignLeft);
        m_curr_frame_label = new QLabel(this);
        m_curr_frame_label->setText(std::to_string(m_curr_frame_ind).c_str());
        m_curr_frame_label->setAlignment(Qt::AlignLeft);
        m_max_frame_label = new QLabel(this);
        m_max_frame_label->setText(std::to_string(m_total_frames - 1).c_str());
        m_max_frame_label->setAlignment(Qt::AlignRight);

        m_hierarchy_label = new QLabel(this);
        m_hierarchy_label->setText("Hierarchy: ");
        m_hierarchy_label->setAlignment(Qt::AlignLeft);
        m_curr_hierarchy_label = new QLabel(this);
        m_curr_hierarchy_label->setText(std::to_string(m_curr_hierarchy_level).c_str());
        m_curr_hierarchy_label->setAlignment(Qt::AlignLeft);
        m_max_hierarchy_label = new QLabel(this);
        m_max_hierarchy_label->setText(std::to_string(m_max_hierarchy_level - 1).c_str());
        m_max_hierarchy_label->setAlignment(Qt::AlignRight);

        // Create buttons
        m_play_button = new QToolButton(this);
        m_play_button->setIcon(style()->standardIcon(QStyle::SP_MediaPause));
        connect(m_play_button, SIGNAL(clicked()), this, SLOT(playButtonClicked()));
        m_previous_keyframe_button = new QToolButton(this);
        m_previous_keyframe_button->setIcon(style()->standardIcon(QStyle::SP_MediaSeekBackward));
        connect(m_previous_keyframe_button, SIGNAL(clicked()), this, SLOT(previousKeyFrameButtonClicked()));
        m_next_keyframe_button = new QToolButton(this);
        m_next_keyframe_button->setIcon(style()->standardIcon(QStyle::SP_MediaSeekForward));
        connect(m_next_keyframe_button, SIGNAL(clicked()), this, SLOT(nextKeyFrameButtonClicked()));

        //;

        // GUI layout
        QGridLayout* centralLayout = new QGridLayout;
        centralLayout->addWidget(m_display_widget, 0 ,0, 1, 4);
        centralLayout->addWidget(m_frame_label, 1, 0);
        centralLayout->addWidget(m_curr_frame_label, 1, 1);
        centralLayout->addWidget(m_frame_slider, 1, 2);
        centralLayout->addWidget(m_max_frame_label, 1, 3);
        centralLayout->addWidget(m_hierarchy_label, 2, 0);
        centralLayout->addWidget(m_curr_hierarchy_label, 2, 1);
        centralLayout->addWidget(m_hierarchy_slider, 2, 2);
        centralLayout->addWidget(m_max_hierarchy_label, 2, 3);
        QHBoxLayout* buttonLayout = new QHBoxLayout;
        buttonLayout->addStretch();
        buttonLayout->addWidget(m_previous_keyframe_button);
        buttonLayout->addSpacing(8);
        buttonLayout->addWidget(m_play_button);
        buttonLayout->addSpacing(8);
        buttonLayout->addWidget(m_next_keyframe_button);
        buttonLayout->addStretch();
        //buttonLayout->setAlignment(Qt::AlignCenter);
        centralLayout->addLayout(buttonLayout, 3, 0, 1, 4);
        centralLayout->setHorizontalSpacing(0);
        centralLayout->setAlignment(Qt::Alignment(Qt::AlignTop));
        m_main_widget->setLayout(centralLayout);
        /*
        QGridLayout* centralLayout = new QGridLayout;
        centralLayout->addWidget(m_display_widget);
        centralLayout->addWidget(m_frame_slider);
        centralLayout->addWidget(m_hierarchy_slider);
        centralLayout->setAlignment(Qt::Alignment(Qt::AlignTop));
        m_main_widget->setLayout(centralLayout);
        */

        // Resize
        const int border = 11 * 2;
        //m_main_widget->resize(m_frame_width + border, m_frame_height + 4 * border);
        //resize(m_frame_width + border, m_frame_height + 4 * border);
        //m_display_widget->resize(m_frame_width, m_frame_height);

        QSize frame_slider_size = m_frame_slider->minimumSizeHint();
        QSize hierarchy_slider_size = m_frame_slider->minimumSizeHint();
        QSize window_size(m_frame_width + border, m_frame_height + 2*border +
            frame_slider_size.height() + hierarchy_slider_size.height() + m_play_button->height());
        m_display_widget->resize(m_frame_width, m_frame_height);
        m_display_widget->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
        m_curr_frame_label->setFixedWidth((border * 3) / 2);
        m_max_frame_label->setFixedWidth((border * 3) / 2);
        m_curr_hierarchy_label->setFixedWidth((border * 3) / 2);
        m_max_hierarchy_label->setFixedWidth((border * 3) / 2);
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
            regionSelected(mouseEvent);
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
        //if (!m_refresh) return;
        //std::cout << "update" << std::endl;
        //std::cout << "(" << m_curr_frame_ind << ", " << m_total_frames - 1 << ")" << std::endl;

        if (m_update_frame)
        {
            m_curr_frame_ind = (int)m_cap->get(cv::CAP_PROP_POS_FRAMES);
            m_frame_slider->setValue(m_curr_frame_ind);
            if (m_curr_frame_ind >= m_total_frames) pause(true);
            else if (m_cap->read(*m_scaled_frame))
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

                // Update face segmentation
                const sfl::Face* face = m_sfl_frames[m_curr_frame_ind]->getFace(m_main_face_id);
                if (face != nullptr)
                {
                    createFullFace(face->landmarks, m_face_boundary->back());
                    *m_face_map = cv::Mat::zeros(m_scaled_frame->size(), CV_8U);
                    cv::drawContours(*m_face_map, *m_face_boundary, 0, cv::Scalar(255, 255, 255), CV_FILLED);
                }  
                else m_face_boundary->back().clear();

                m_refresh = true;
                m_update_frame = m_loop;
            }
        }

        // Render
        if (m_refresh)
        {
            m_scaled_frame->copyTo(*m_render_frame);
            render(*m_render_frame);
            m_display_widget->setPixmap(QPixmap::fromImage(m_render_image->rgbSwapped()));
            m_display_widget->update();
        }
        
        if (m_loop) updateLater();
        //else m_refresh = false;
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
        m_update_frame = true;
    }

    void Editor::pause(bool pause)
    {
        m_loop = !pause;
        m_update_frame = m_refresh = m_loop;
        if (m_loop)
        {
            updateLater();
            m_play_button->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
        }
        else m_play_button->setIcon(style()->standardIcon(QStyle::SP_MediaPause));
    }

    void Editor::render(cv::Mat& frame)
    {
        /*
        // Render segmentation at specified level.
        segmentation::RenderRegionsRandomColor(m_curr_hierarchy_level,
            true,
            false,
            *m_seg_desc,
            &m_seg_hierarchy->hierarchy(),
            &frame);
            */

        // Render segmentation
        //auto& faces = m_sequence_regions->frames(m_curr_frame_ind).faces();
        //auto& face = faces.find(m_main_face_id);
        //if (face != faces.end())
        //{
            cv::Mat seg;
            Face* edit_face = getNearestEditedFace();
            if (edit_face != nullptr)
            {
                if (m_face_boundary->back().empty())
                    seg = calcSegmentation(frame.size(), edit_face->regions(), *m_seg_desc);
                else seg = calcSegmentation(*m_face_map, edit_face->regions(), *m_seg_desc);
                renderSegmentationBlend(frame, seg, 0.25f);
            } 
        //}   

        renderBoundaries(frame, m_curr_hierarchy_level, *m_seg_desc, &m_seg_hierarchy->hierarchy());

        if(!m_face_boundary->back().empty())
            cv::drawContours(frame, *m_face_boundary, 0, cv::Scalar(0, 255, 0), 1);

        /*
        // Render landmarks
        const sfl::Face* main_face = m_sfl_frames[m_curr_frame_ind]->getFace(m_main_face_id);
        if(main_face != nullptr)
            sfl::render(frame, main_face->landmarks);
            */
    }

    void Editor::regionSelected(QMouseEvent * event)
    {
        // Decide what to do with the selected regions
        RegionType type = FULL;
        bool insert = true;
        switch (event->button())
        {
        case Qt::LeftButton: type = FULL; break;
        case Qt::RightButton: type = INTERSECTION; break;
        case Qt::MiddleButton: insert = false; break;
        default: break;
        }

        //std::cout << "pos = (" << event->x() << ", " << event->y() << ")" << std::endl;//
        int id = segmentation::GetOversegmentedRegionIdFromPoint(event->x(), event->y(), *m_seg_desc);//
        int parent_id = segmentation::GetParentId(id, 0, m_curr_hierarchy_level, m_seg_hierarchy->hierarchy());
        //std::cout << "oversegmented region id = " << id << std::endl;
        //std::cout << "parent region id = " << parent_id << std::endl;

        // Get children
        segmentation::ParentMap parentMap;
        segmentation::GetParentMap(m_curr_hierarchy_level, *m_seg_desc, m_seg_hierarchy->hierarchy(), &parentMap);
        /*
        std::cout << "children ids = ";
        for (auto& r : parentMap[parent_id])
            std::cout << r->id() << ", ";
        std::cout << std::endl;
        */
        /*
        Frame* frame = m_sequence_regions->mutable_frames(m_curr_frame_ind);
        auto& faces = *frame->mutable_faces();
        Face& face = faces[(unsigned int)m_main_face_id];
        */
        Face& face = getFaceForEditing();
        auto& face_regions = *face.mutable_regions();

        if (insert)
        {
            for (auto& r : parentMap[parent_id])
            {
                Region& face_region = face_regions[(unsigned int)r->id()];
                face_region.set_id((unsigned int)r->id());
                face_region.set_type(type);
            }
        }
        else    // Remove
        {
            for (auto& r : parentMap[parent_id])
                face_regions.erase((unsigned int)r->id());
        }

        m_refresh = true;
        updateLater();
    }

    Face & Editor::getFaceForEditing()
    {
        // Find edit frame (assume sorted)
        Frame *edit_frame = nullptr, *nearest_edit_frame = nullptr;
        int frame_ind = 0;
        for (Frame& frame : *m_edited_regions->mutable_frames())
        {
            if (frame.id() == m_curr_frame_ind)
            {
                edit_frame = &frame;
                m_edit_index = frame_ind;
                break;
            }
            else if (frame.id() < m_curr_frame_ind)
                nearest_edit_frame = &frame;

            ++frame_ind;
        }

        // Don't inherit if already exists
        if (edit_frame != nullptr) nearest_edit_frame = nullptr;

        // Create edit frame if it doesn't exist for current frame index
        if (edit_frame == nullptr)
        {
            edit_frame = m_edited_regions->add_frames();
            edit_frame->set_id(m_curr_frame_ind);
            edit_frame->set_width(m_frame_width);
            edit_frame->set_height(m_frame_height);

            // Sort frames
            std::sort(m_edited_regions->mutable_frames()->begin(),
                m_edited_regions->mutable_frames()->end(),
                [](const Frame& f1, const Frame& f2) {
                return f1.id() < f1.id();
            });
        }

        // Get edit face
        auto& face_map = *edit_frame->mutable_faces();
        Face& edit_face = face_map[(unsigned int)m_main_face_id];
        edit_face.set_id((unsigned int)m_main_face_id);

        // Inherit regions from nearest edit frame
        if (nearest_edit_frame != nullptr)
        {
            auto& nearest_face_map = *nearest_edit_frame->mutable_faces();
            Face& nearest_edit_face = nearest_face_map[(unsigned int)m_main_face_id];
            for (auto& r : *nearest_edit_face.mutable_regions())
                (*edit_face.mutable_regions())[r.first] = r.second;
        }

        // Return face for editing
        return edit_face;
    }

    Face* Editor::getNearestEditedFace()
    {
        if (m_edited_regions->mutable_frames()->empty()) return nullptr;
        
        // Find closest frame (assume sorted)
        Face* edit_face = nullptr;
        for (Frame& frame : *m_edited_regions->mutable_frames())
        {
            if (frame.id() <= m_curr_frame_ind)
            {
                auto& face_map = *frame.mutable_faces();
                edit_face = &face_map[(unsigned int)m_main_face_id];
                if(frame.id() == m_curr_frame_ind) break;
            }
        }

        return edit_face;
    }

    void Editor::frameIndexChanged(int n)
    {
        //m_curr_frame_ind = n;
        //m_frame_slider->setValue(m_curr_frame_ind);
        m_curr_frame_label->setText(std::to_string(n).c_str());
        seek(n);
    }  

    void Editor::hierarchyLevelChanged(int n) {
        m_curr_hierarchy_level = n;
        m_hierarchy_slider->setValue(m_curr_hierarchy_level);
        m_curr_hierarchy_label->setText(std::to_string(n).c_str());
        m_refresh = true;
    }

    void Editor::playButtonClicked()
    {
        pause(m_loop);
    }

    void Editor::previousKeyFrameButtonClicked()
    {
        if (m_keyframes.empty()) return;
        for (int i = (int)m_keyframes.size() - 1; i >= 0; --i)
        {
            if (m_keyframes[i] < m_curr_frame_ind)
            {
                seek(m_keyframes[i]);
                break;
            }
        }     
    }

    void Editor::nextKeyFrameButtonClicked()
    {
        if (m_keyframes.empty()) return;
        for (size_t i = 0; i < m_keyframes.size(); ++i)
        {
            if (m_keyframes[i] > m_curr_frame_ind)
            {
                seek(m_keyframes[i]);
                break;
            }
        }      
    }

    void Editor::frameSliderPress()
    {
        //std::cout << "press" << std::endl;
        m_slider_pause = !m_loop;
        pause(true);
    }

    void Editor::frameSliderRelease()
    {
        //std::cout << "release" << std::endl;
        pause(m_slider_pause);
        m_slider_pause = false;
    }
}   // namespace fvs