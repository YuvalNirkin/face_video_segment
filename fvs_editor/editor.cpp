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

// boost
#include <boost/filesystem.hpp>
#include <boost/format.hpp> // debug

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
#include <QComboBox>
#include <QToolButton>
#include <QStatusBar>
#include <QCheckBox>

using namespace boost::filesystem;

namespace fvs
{
    Editor::Editor(const std::string& fvs_path, const std::string& output_dir,
        const std::string& video_file, const std::string& seg_file, 
        const std::string& landmarks_file, bool debug) :
        m_loop(false),
        m_refresh(true),
        m_slider_pause(false),
        m_update_pending(false),
        m_update_frame(true),
        m_update_face(false),
        m_debug(debug),
        m_curr_frame_ind(0),
        m_next_frame_ind(-1),
        m_frame_width(0), m_frame_height(0),
        m_fps(0),
        m_total_frames(0),
        m_main_face_id(0),
        m_hierarchy_pos(0),
        m_edit_index(-1),
        m_curr_face_id(0),
        m_render_contours(true),
        m_render_borders(true),
        m_render_seg(true),
        m_postprocess(false),
        m_alpha(0.0f),
        m_curr_file(fvs_path),
        m_output_dir(output_dir),
        m_video_file(video_file),
        m_seg_file(seg_file),
        m_landmarks_file(landmarks_file)
    {
        // Initialize keyframer
        m_keyframer = std::make_unique<Keyframer>(10, 5);

        // Initialize face segmentation
        m_input_regions.reset(new Sequence());
        m_edited_regions.reset(new Sequence());
        m_face_boundary.reset(new std::vector<std::vector<cv::Point>>());
        m_face_boundary->resize(1);

        if (fvs_path.empty())   // Initialize empty faces
        {
            throw std::runtime_error(
                "Path to face video segmentation (.fvs) must be provided!");
            /*
            // For each frame in the sequence
            auto& sfl_it = m_sfl->getSequence().begin();
            for (unsigned int i = 0; i < m_total_frames; ++i)
            {
                auto& sfl_frame = *sfl_it++;
                Frame* fvs_frame = m_input_regions->add_frames();
                fvs_frame->set_id(i);
                fvs_frame->set_width(m_frame_width);
                fvs_frame->set_height(m_frame_height);
                auto& faces = *fvs_frame->mutable_faces();

                // For each face in the sfl frame
                for (auto& sfl_face : sfl_frame->faces)
                {
                    Face& fvs_face = faces[sfl_face->id];
                    fvs_face.set_id((unsigned int)sfl_face->id);
                }

                // Check for keyframes
                m_keyframer->addFrame(*sfl_frame, *fvs_frame);
            }
            */
        }
        else    // Read input regions from file
        {
            std::ifstream input(fvs_path, std::ifstream::binary);
            m_input_regions->ParseFromIstream(&input);

            // Extract paths to the rest of the files
            if (m_video_file.empty()) m_video_file = m_input_regions->video_path();
            if (!is_regular_file(m_video_file))
                throw std::runtime_error("Couldn't find video file!");
            if (m_seg_file.empty()) m_seg_file = m_input_regions->seg_path();
            if (!is_regular_file(m_seg_file))
                throw std::runtime_error("Couldn't find segmentation file!");
            if (m_landmarks_file.empty()) m_landmarks_file = m_input_regions->landmarks_path();
            if (!is_regular_file(m_landmarks_file))
                throw std::runtime_error("Couldn't find landmarks cache file!");
        }

        // Initialize video capture
        m_cap.reset(new cv::VideoCapture());
        if (!m_cap->open(m_video_file))
            throw std::runtime_error("Failed to open video file!");

        m_frame_width = (int)m_cap->get(cv::CAP_PROP_FRAME_WIDTH);
        m_frame_height = (int)m_cap->get(cv::CAP_PROP_FRAME_HEIGHT);
        m_fps = m_cap->get(cv::CAP_PROP_FPS);
        m_total_frames = (size_t)m_cap->get(cv::CAP_PROP_FRAME_COUNT);
        m_scaled_frame.reset(new cv::Mat(m_frame_height, m_frame_width, CV_8UC3));
        m_render_frame.reset(new cv::Mat(m_frame_height, m_frame_width, CV_8UC3));
        m_face_map.reset(new cv::Mat(m_frame_height, m_frame_width, CV_8U));

        // Make Qt image.
        m_render_image.reset(new QImage((const uint8_t*)m_render_frame->data,
            m_scaled_frame->cols,
            m_scaled_frame->rows,
            m_scaled_frame->step[0],
            QImage::Format_RGB888));

        // Initialize sequence face landmarks
        m_sfl = sfl::SequenceFaceLandmarks::create(m_landmarks_file);
        if(m_sfl->size() < m_total_frames)
            throw std::runtime_error(
                "The number of landmark frames does not match the number of video frames!");
        const std::list<std::unique_ptr<sfl::Frame>>& sfl_frames_list =  m_sfl->getSequence();
        m_curr_face_id = m_main_face_id = sfl::getMainFaceID(sfl_frames_list);
        m_sfl_frames.reserve(sfl_frames_list.size());
        for (auto& frame : sfl_frames_list)
            m_sfl_frames.push_back(frame.get());  

        // Initialize segmentation reader
        m_seg_reader.reset(new segmentation::SegmentationReader(m_seg_file));
        if (!m_seg_reader->OpenFileAndReadHeaders())
            throw std::runtime_error("Failed to read segmentation file!");
        m_seg_desc.reset(new segmentation::SegmentationDesc);
        m_seg_hierarchy.reset(new segmentation::SegmentationDesc);
        m_seg_reader->ReadNextFrame(m_seg_hierarchy.get());

        // Get all face ids
        for (const Frame& fvs_frame : m_input_regions->frames())
        {
            for (auto& fvs_face : fvs_frame.faces())
                m_face_ids.insert((int)fvs_face.second.id());
        }

        // Create Menu
        createMenu();
        
        // Create main widget
        m_main_widget = new QLabel(this);
        setCentralWidget(m_main_widget);
        std::string title = (path(fvs_path).stem() += 
            " - Face Video Segmentation Editor").string();
        setWindowTitle(title.c_str());

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

        m_face_id_label = new QLabel(this);
        m_face_id_label->setText("Face id: ");
        m_face_id_label->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);

        // Create combobox
        m_face_id_combobox = new QComboBox(this);
        for (int face_id : m_face_ids)
        {
            m_face_id_combobox->addItem(QString::number(face_id));
        }
        m_face_id_combobox->setCurrentText(QString::number(m_curr_face_id));
        connect(m_face_id_combobox, SIGNAL(currentTextChanged(const QString&)), this, SLOT(currFaceIdChanged(const QString&)));

        // Create buttons
        m_play_button = new QToolButton(this);
        m_play_button->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
        connect(m_play_button, SIGNAL(clicked()), this, SLOT(playButtonClicked()));
        m_previous_keyframe_button = new QToolButton(this);
        m_previous_keyframe_button->setIcon(style()->standardIcon(QStyle::SP_MediaSeekBackward));
        connect(m_previous_keyframe_button, SIGNAL(clicked()), this, SLOT(previousKeyFrameButtonClicked()));
        m_next_keyframe_button = new QToolButton(this);
        m_next_keyframe_button->setIcon(style()->standardIcon(QStyle::SP_MediaSeekForward));
        connect(m_next_keyframe_button, SIGNAL(clicked()), this, SLOT(nextKeyFrameButtonClicked()));

        // Create keyframe widgets
        m_toggle_keyframe_checkbox = new QCheckBox(this);
        m_toggle_keyframe_checkbox->setText("Toggle keyframe");
        m_toggle_keyframe_checkbox->setCheckState(
            isKeyframe(m_curr_frame_ind, m_curr_face_id) ? Qt::Checked : Qt::Unchecked);
        connect(m_toggle_keyframe_checkbox, SIGNAL(clicked(bool)), this, SLOT(toggleKeyframe(bool)));
        m_keyframe_label = new QLabel(this);
        m_keyframe_label->setText("");
        m_keyframe_label->setMinimumWidth(m_toggle_keyframe_checkbox->width());

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
        centralLayout->addWidget(m_face_id_label, 3, 0);
        centralLayout->addWidget(m_face_id_combobox, 3, 1);
        
        QHBoxLayout* buttonLayout = new QHBoxLayout;
        buttonLayout->addStretch();
        //buttonLayout->addLayout(leftLayout);
        buttonLayout->addWidget(m_toggle_keyframe_checkbox);
        buttonLayout->addSpacing(8);
        buttonLayout->addWidget(m_previous_keyframe_button);
        buttonLayout->addSpacing(8);
        buttonLayout->addWidget(m_play_button);
        buttonLayout->addSpacing(8);
        buttonLayout->addWidget(m_next_keyframe_button);
        buttonLayout->addSpacing(8);
        buttonLayout->addWidget(m_keyframe_label);
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
        int verticalSpacing = centralLayout->verticalSpacing();

        QSize frame_slider_size = m_frame_slider->minimumSizeHint();
        QSize hierarchy_slider_size = m_frame_slider->minimumSizeHint();
        QSize window_size(m_frame_width + border, m_frame_height + 2*border +
            frame_slider_size.height() + hierarchy_slider_size.height() + m_play_button->height());
        //m_display_widget->resize(m_frame_width, m_frame_height);
        m_display_widget->setFixedSize(m_frame_width, m_frame_height);
        m_display_widget->setSizePolicy(QSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed));
        m_curr_frame_label->setFixedWidth((border * 3) / 2);
        m_max_frame_label->setFixedWidth((border * 3) / 2);
        m_curr_hierarchy_label->setFixedWidth((border * 3) / 2);
        m_max_hierarchy_label->setFixedWidth((border * 3) / 2);

        //m_main_widget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        //setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        QSize main_widget_size = m_main_widget->size();
        //m_main_widget->resize(window_size);
        m_main_widget->setMinimumSize(window_size);
        m_main_widget->adjustSize();//
        //resize(window_size);
        //adjustSize();//

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

        // Update frame
        if (m_update_frame)
        {
            m_curr_frame_ind = (int)m_cap->get(cv::CAP_PROP_POS_FRAMES);
            m_frame_slider->setValue(m_curr_frame_ind);
            if (m_curr_frame_ind >= m_total_frames)
            {
                m_curr_frame_ind = (int)m_total_frames - 1;
                pause(true);
            }
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

                m_update_face = true;
                m_update_frame = m_loop;
            }
        }

        // Update face
        if (m_update_face)
        {
            // Update face segmentation
            const sfl::Face* face = m_sfl_frames[m_curr_frame_ind]->getFace(m_curr_face_id);
            if (face != nullptr)
            {
                sfl::createFullFace(face->landmarks, m_face_boundary->back());
                *m_face_map = cv::Mat::zeros(m_scaled_frame->size(), CV_8U);
                cv::drawContours(*m_face_map, *m_face_boundary, 0, cv::Scalar(255, 255, 255), CV_FILLED);
            }
            else m_face_boundary->back().clear();
            m_refresh = true;
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
        m_update_frame = m_loop;
        //m_update_frame = m_refresh = m_loop;
        if (m_loop)
        {
            updateLater();
            m_play_button->setIcon(style()->standardIcon(QStyle::SP_MediaPause));
        }
        else m_play_button->setIcon(style()->standardIcon(QStyle::SP_MediaPlay));
    }

    void Editor::render(cv::Mat& frame)
    {
        // Get regions for rendering
        google::protobuf::Map<google::protobuf::uint32, fvs::Region> region_map;
        getCurrMergedRegions(region_map);

        // Calculate segmentation
        cv::Mat seg;
        if (m_face_boundary->back().empty())
            seg = calcSegmentation(frame.size(), region_map, *m_seg_desc);
        else seg = calcSegmentation(*m_face_map, region_map, *m_seg_desc);
        if (m_postprocess)
            postprocessSegmentation(seg);

        // Render segmentation
        if(m_render_seg) renderSegmentationBlend(frame, seg, m_alpha);
        if(m_render_borders) renderBoundaries(frame, m_curr_hierarchy_level,
            *m_seg_desc, &m_seg_hierarchy->hierarchy());

        if(m_render_contours && !m_face_boundary->back().empty())
            cv::drawContours(frame, *m_face_boundary, 0, cv::Scalar(0, 255, 0), 1);
    }

    void Editor::regionSelected(QMouseEvent * event)
    {
        const segmentation::VectorMesh& mesh = m_seg_desc->vector_mesh();

        // Decide what to do with the selected regions
        PolygonType type = FULL;
        //bool insert = true;
        switch (event->button())
        {
        case Qt::LeftButton: type = FULL; break;
        case Qt::RightButton: type = INTERSECTION; break;
        //case Qt::MiddleButton: insert = false; break;
        case Qt::MiddleButton: type = EMPTY; break;
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
        Frame* frame = m_input_regions->mutable_frames(m_curr_frame_ind);
        auto& faces = *frame->mutable_faces();
        Face& face = faces[(unsigned int)m_curr_face_id];
        */
        Face& face = getFaceForEditing();
        auto& face_regions = *face.mutable_regions();

        if (m_curr_hierarchy_level == 0)
        {
            Region& edit_region = face_regions[id];
            edit_region.set_id((unsigned int)id);
            if (edit_region.polygons_size() == 0)    // Not initialized
            {
                // Copy regions from input regions
                const Frame& input_frame = m_input_regions->frames(m_curr_frame_ind);
                auto& input_face_map = input_frame.faces();
                auto& input_face = input_face_map.find(m_curr_face_id);
                if (input_face != input_face_map.end())
                {
                    auto& input_region_map = input_face->second.regions();
                    auto& input_region = input_region_map.find(id);
                    if (input_region != input_region_map.end())
                        edit_region = input_region->second;
                }
            }
         
            if (edit_region.polygons_size() == 0)    // Still not initialized
            {
                // Initialize all polygons to empty
                const segmentation::Region2D* r = parentMap[id][0];

                // For each polygon
                for (const auto& poly : r->vectorization().polygon())
                {
                    if (poly.hole()) continue;
                    if (poly.coord_idx_size() == 0) continue;
                    edit_region.add_polygons(EMPTY);

                    //std::vector<std::vector<cv::Point>> contours;
                    //createContours(mesh, poly, contours);
                }
            }

            // Change selected polygon
            const segmentation::Region2D* r = parentMap[id][0];

            // For each polygon
            int poly_ind = 0;
            for (const auto& poly : r->vectorization().polygon())
            {
                if (poly.hole()) continue;
                if (poly.coord_idx_size() == 0) continue;
                //edit_region.add_polygons(EMPTY);

                std::vector<std::vector<cv::Point>> contours;
                createContours(mesh, poly, contours);

                // Check if mouse click is inside the current polygon
                if (cv::pointPolygonTest(contours.back(),
                    cv::Point2f((float)event->x(), (float)event->y()), false) >= 0)
                {
                    edit_region.set_polygons(poly_ind, type);
                    std::cout << "Selected region " << id << " poly " << poly_ind << 
                        " [0, " << edit_region.polygons_size() - 1 << "]" << std::endl;//
                    break;
                }

                ++poly_ind;
            }
        }
        else    // m_curr_hierarchy_level > 0
        {
            // For each selected region
            for (auto& r : parentMap[parent_id])
            {
                Region& edit_region = face_regions[(unsigned int)r->id()];
                edit_region.set_id((unsigned int)r->id());

                // For each polygon
                if (edit_region.polygons_size() == 0)    // Not initialized
                {
                    for (const auto& poly : r->vectorization().polygon())
                    {
                        if (poly.hole()) continue;
                        if (poly.coord_idx_size() == 0) continue;
                        edit_region.add_polygons(type);
                    }
                }
                else    // Polygons already exist
                {
                    for (int i = 0; i < edit_region.polygons_size(); ++i)
                        edit_region.set_polygons(i, type);
                }
            }
        }

        /// Debug ///
        face.PrintDebugString();
        /////////////

        m_refresh = true;
        updateLater();
    }

    Face& Editor::getFaceForEditing()
    {
        Frame *edit_frame = nullptr, *nearest_edit_frame = nullptr;
        Face *edit_face = nullptr, *nearest_edit_face = nullptr;

        // Find edit frame (assume sorted)
        for (Frame& frame : *m_edited_regions->mutable_frames())
        {
            if (frame.id() == m_curr_frame_ind)
            {
                edit_frame = &frame;
                break;
            }
            else if (frame.id() < m_curr_frame_ind)
                nearest_edit_frame = &frame;
        }

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
        else
        {
            // Find edit face
            auto& edit_face_map = *edit_frame->mutable_faces();
            auto& edit_face_it = edit_face_map.find(m_curr_face_id);
            if (edit_face_it != edit_face_map.end())
                edit_face = &edit_face_it->second;
        }

        // Find nearest edit face
        if (nearest_edit_frame != nullptr)
        {
            auto& nearest_edit_face_map = *nearest_edit_frame->mutable_faces();
            auto& nearest_edit_face_it = nearest_edit_face_map.find(m_curr_face_id);
            if (nearest_edit_face_it != nearest_edit_face_map.end())
                nearest_edit_face = &nearest_edit_face_it->second;
        }

        // Check if edit face already exists
        if (edit_face == nullptr)
        {
            // Create edit face
            auto& edit_face_map = *edit_frame->mutable_faces();
            edit_face = &edit_face_map[(unsigned int)m_curr_face_id];
            edit_face->set_id((unsigned int)m_curr_face_id);

            // Inherit regions from nearest edit frame
            if (nearest_edit_face != nullptr)
            {
                auto& edit_region_map = *edit_face->mutable_regions();
                auto& nearest_edit_region_map = *nearest_edit_face->mutable_regions();
                //const Frame& input_frame = m_input_regions->frames(m_curr_frame_ind);
                //auto& input_face_map = input_frame.faces();
                //auto& input_face = input_face_map.find(m_curr_face_id);

                // For each region
                for (auto& r : m_seg_desc->region())
                {
                    auto& nearest_edit_region = nearest_edit_region_map.find((unsigned int)r.id());
                    if (nearest_edit_region == nearest_edit_region_map.end()) continue;

                    // Count each polygon
                    int poly_count = 0;
                    for (auto& poly : r.vectorization().polygon())
                    {
                        if (poly.hole()) continue;
                        if (poly.coord_idx_size() == 0) continue;
                        ++poly_count;
                    }

                    if (poly_count > 0)
                    {
                        if (nearest_edit_region->second.polygons_size() == poly_count)
                        {
                            edit_region_map[nearest_edit_region->first] = 
                                nearest_edit_region->second;
                        }
                        else    // Different number of polygons
                        {
                            // Check if to inherit
                            PolygonType type = nearest_edit_region->second.polygons(0);
                            bool inherit = true;
                            for (int i = 1; i < nearest_edit_region->second.polygons_size(); ++i)
                            {
                                if (nearest_edit_region->second.polygons(i) != type)
                                {
                                    inherit = false;
                                    break;
                                }
                            }
                            if (!inherit) continue;

                            // Inherit
                            Region& edit_region = edit_region_map[nearest_edit_region->first];
                            edit_region.set_id(nearest_edit_region->first);

                            // For each polygon
                            for (const auto& poly : r.vectorization().polygon())
                            {
                                if (poly.hole()) continue;
                                if (poly.coord_idx_size() == 0) continue;
                                edit_region.add_polygons(type);
                            }
                        }
                    }
                }
            }
        }

        return *edit_face;

        /////
        /////
        /*
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
        Face& edit_face = face_map[(unsigned int)m_curr_face_id];
        edit_face.set_id((unsigned int)m_curr_face_id);

        // Inherit regions from nearest edit frame
        if (nearest_edit_frame != nullptr)
        {
            // Initialize

            //
            auto& nearest_face_map = *nearest_edit_frame->mutable_faces();
            Face& nearest_edit_face = nearest_face_map[(unsigned int)m_curr_face_id];
            for (auto& r : *nearest_edit_face.mutable_regions())
                (*edit_face.mutable_regions())[r.first] = r.second;
        }

        // Return face for editing
        return edit_face;
        */
    }

    Frame * Editor::getNearestEditedFrame(int frame_id)
    {
        if (m_edited_regions->mutable_frames()->empty()) return nullptr;

        // Find closest frame (assume sorted)
        Frame* edit_frame = nullptr;
        for (Frame& frame : *m_edited_regions->mutable_frames())
        {
            if (frame.id() <= frame_id)
            {
                edit_frame = &frame;
                if (frame.id() == frame_id) break;
            }
        }

        return edit_frame;
    }

    Face* Editor::getNearestEditedFace(int frame_id, int face_id)
    {
        // TODO: Rewrite this for each frame
        Frame* edit_frame = getNearestEditedFrame(frame_id);
        if (edit_frame == nullptr) return nullptr;
        auto& face_map = *edit_frame->mutable_faces();
        auto edit_face = face_map.find(face_id);
        if (edit_face == face_map.end()) return nullptr;
        return &edit_face->second;
    }

    void Editor::getMergedRegions(int frame_id, int face_id,
        google::protobuf::Map<unsigned int, Region>& region_map)
    {
        auto& input_face_map = m_input_regions->frames(frame_id).faces();
        auto& input_face = input_face_map.find(face_id);
        if (input_face != input_face_map.end())
            region_map = input_face->second.regions();

        Frame* edit_frame = getNearestEditedFrame(frame_id);
        if (edit_frame == nullptr) return;
        auto& edit_face_map = *edit_frame->mutable_faces();
        auto& edit_face = edit_face_map.find(face_id);
        if (edit_face == edit_face_map.end()) return;
        auto& edit_region_map = edit_face->second.regions();

        if (edit_frame->id() == frame_id)
        {
            // Overide edited regions
            for (auto& edit_region : edit_region_map)
                region_map[edit_region.second.id()] = edit_region.second;
                //region_map[edit_region.second.id()].CopyFrom(edit_region.second);
        }
        else
        {
            // Only inherit totally full or empty edit regions
            // For each region
            for (const auto& r : m_seg_desc->region())
            {
                if (r.vectorization().polygon().empty()) continue;
                auto& edit_region = edit_region_map.find((unsigned int)r.id());
                if (edit_region == edit_region_map.end()) continue;
                if (edit_region->second.polygons_size() == 0) continue;

                // Check if to inherit
                PolygonType type = edit_region->second.polygons(0);
                bool inherit = true;
                for (int i = 1; i < edit_region->second.polygons_size(); ++i)
                {
                    if (edit_region->second.polygons(i) != type)
                    {
                        inherit = false;
                        break;
                    }
                }
                if (!inherit) continue;

                Region& region = region_map[(unsigned int)r.id()];
                region.set_id((unsigned int)r.id());
                if (region.polygons_size() > 0) // Already initialized
                {
                    for (int i = 0; i < region.polygons_size(); ++i)
                        region.set_polygons(i, type);
                }
                else    // Require initialization
                {
                    // For each polygon
                    int poly_ind = 0;
                    for (const auto& poly : r.vectorization().polygon())
                    {
                        if (poly.hole()) continue;
                        if (poly.coord_idx_size() == 0) continue;
                        region.add_polygons(type);
                    }
                }
            }
        }
    }

    void Editor::getCurrMergedRegions(
        google::protobuf::Map<unsigned int, Region>& region_map)
    {
        getMergedRegions(m_curr_frame_ind, m_curr_face_id, region_map);
    }

    bool Editor::isKeyframe(int frame_id, int face_id)
    {
        // Check edit regions
        Frame* edit_frame = getNearestEditedFrame(frame_id);
        if(edit_frame != nullptr && edit_frame->id() == frame_id)
        {
            auto& edit_face_map = edit_frame->faces();
            auto& edit_face = edit_face_map.find(face_id);
            if (edit_face != edit_face_map.end())
                return edit_face->second.keyframe();
        }

        // Check input regions
        const Frame& frame = m_input_regions->frames(frame_id);
        auto& face_map = frame.faces();
        auto& face = face_map.find(face_id);
        if (face == face_map.end()) return false;
        return face->second.keyframe();
    }

    bool Editor::saveFile(const std::string& filename)
    {
        statusBar()->showMessage(tr("Saving..."));
        Sequence sequence(*m_input_regions);

        // For each frame
        for (Frame& frame : *sequence.mutable_frames())
        {
            // Update segmentation
            m_seg_reader->SeekToFrame(frame.id());
            m_seg_reader->ReadNextFrame(m_seg_desc.get());

            // For each face
            for (int face_id : m_face_ids)
            {
                google::protobuf::Map<unsigned int, Region> region_map;
                getMergedRegions(frame.id(), face_id, region_map);
                if (region_map.empty()) continue;

                auto& face_map = *frame.mutable_faces();
                *face_map[face_id].mutable_regions() = region_map;
                face_map[face_id].set_keyframe(isKeyframe(frame.id(), face_id));
            }
        }

        /// Debug ///
        if (m_debug)
        {
            std::string outPath = (boost::filesystem::path(m_output_dir) /
                (path(filename).stem() += ".txt")).string();
            std::ofstream output(outPath);
            output << m_edited_regions->DebugString();
        }
        /*
        if (face_map[face_id].keyframe())
        {
        face_map[face_id].PrintDebugString();
        }
        */
        /////////////

        // Write to file
        std::ofstream output(filename, std::fstream::trunc | std::fstream::binary);
        sequence.SerializeToOstream(&output);
        
        // Restore segmentation to current frame
        m_seg_reader->SeekToFrame(m_curr_frame_ind);
        m_seg_reader->ReadNextFrame(m_seg_desc.get());

        statusBar()->showMessage(tr("Saving... done!"));
        return true;
    }

    void Editor::frameIndexChanged(int n)
    {
        //m_curr_frame_ind = n;
        //m_frame_slider->setValue(m_curr_frame_ind);
        m_curr_frame_label->setText(std::to_string(n).c_str());
        m_toggle_keyframe_checkbox->setCheckState(
            isKeyframe(n, m_curr_face_id) ? Qt::Checked : Qt::Unchecked);
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
        int seek_ind = -1;

        // For each input frame before the current frame
        for (int i = m_curr_frame_ind - 1; i >= 0; --i)
        {
            auto& face_map = m_input_regions->frames(i).faces();
            auto& face = face_map.find(m_curr_face_id);
            if (face == face_map.end()) continue;
            if (face->second.keyframe())
            {
                seek_ind = i;
                break;
            }
        }

        // For each edit frame
        for (int i = m_edited_regions->frames_size() - 1; i >= 0; --i)
        {
            const Frame& edit_frame = m_edited_regions->frames(i);
            if (edit_frame.id() >= m_curr_frame_ind) continue;
            auto& face_map = edit_frame.faces();
            auto& face = face_map.find(m_curr_face_id);
            if (face == face_map.end()) continue;
            if (face->second.keyframe())
            {
                if (seek_ind < 0) seek_ind = edit_frame.id();
                else seek_ind = std::max(seek_ind, (int)edit_frame.id());
                break;
            }
        }

        if(seek_ind >= 0) seek(seek_ind);
    }

    void Editor::nextKeyFrameButtonClicked()
    {
        int seek_ind = -1;

        // For each input frame after the current frame
        for (int i = m_curr_frame_ind + 1; i < m_total_frames; ++i)
        {
            auto& face_map = m_input_regions->frames(i).faces();
            auto& face = face_map.find(m_curr_face_id);
            if (face == face_map.end()) continue;
            if (face->second.keyframe())
            {
                seek_ind = i;
                break;
            }
        }

        // For each edit frame
        for (const Frame& edit_frame : m_edited_regions->frames())
        {
            if (edit_frame.id() <= m_curr_frame_ind) continue;
            auto& face_map = edit_frame.faces();
            auto& face = face_map.find(m_curr_face_id);
            if (face == face_map.end()) continue;
            if (face->second.keyframe())
            {
                if (seek_ind < 0) seek_ind = edit_frame.id();
                else seek_ind = std::min(seek_ind, (int)edit_frame.id());
                break;
            }
        }

        if (seek_ind >= 0) seek(seek_ind);
    }

    void Editor::currFaceIdChanged(const QString& text)
    {
        m_curr_face_id = text.toInt();
        m_toggle_keyframe_checkbox->setCheckState(
            isKeyframe(m_curr_frame_ind, m_curr_face_id) ? Qt::Checked : Qt::Unchecked);
        //std::cout << "curr face id = " << m_curr_face_id << std::endl;//
        m_update_face = true;
        updateLater();
    }

    void Editor::toggleKeyframe(bool checked)
    {
        std::cout << checked << std::endl;//
        Face& face = getFaceForEditing();
//        face.set_keyframe(state == Qt::Checked);
        face.set_keyframe(checked);
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