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

#ifndef __FVS_EDITOR_H__
#define __FVS_EDITOR_H__

// std
#include <memory>
#include <set>

// Qt
#include <QMainWindow>

// OpenCV
#include <opencv2/core/types.hpp>

// Forward declarations
class QLabel;
class QImage;
class QSlider;
class QMouseEvent;
class QKeyEvent;
class QComboBox;
class QToolButton;
class QString;
class QCheckBox;

namespace cv
{
    class Mat;
    //class Point;
    class VideoCapture;
}

namespace sfl
{
    class SequenceFaceLandmarks;
    class Frame;
}

namespace segmentation
{
    class SegmentationReader;
    class SegmentationDesc;
}

namespace google
{
    namespace protobuf
    {
        template <typename Key, typename T>
        class Map;
    }
}

namespace fvs
{
    class Sequence;
    class Frame;
    class Face;
    class Region;
    enum PolygonType;
    class Keyframer;

	class Editor : public QMainWindow
	{
        Q_OBJECT

	public:
        explicit Editor(const std::string& fvs_path, const std::string& output_dir = "",
            const std::string& video_file = "", const std::string& seg_file = "",
            const std::string& landmarks_file = "", bool debug = false);
        ~Editor();

    private:
        // UI
        void createMenu();

    protected:
        bool event(QEvent *event) Q_DECL_OVERRIDE;
        bool eventFilter(QObject * object, QEvent * event) Q_DECL_OVERRIDE;
        void keyPressEvent(QKeyEvent* event) Q_DECL_OVERRIDE;

    private:
        void update();
        void updateLater();
        void seek(int index);
        void pause(bool pause);
        void render(cv::Mat& frame);
        void regionSelected(QMouseEvent* event);
        Face& getFaceForEditing();
        Frame* getNearestEditedFrame(int frame_id);
        Face* getNearestEditedFace(int frame_id, int face_id);
        void getMergedRegions(int frame_id, int face_id,
            google::protobuf::Map<unsigned int, Region>& region_map);
        void getCurrMergedRegions(
            google::protobuf::Map<unsigned int, Region>& region_map);
        bool isKeyframe(int frame_id, int face_id);
        bool saveFile(const std::string& filename);

    public slots:
        void frameIndexChanged(int);
        void frameSliderPress();
        void frameSliderRelease();
        void hierarchyLevelChanged(int);
        void playButtonClicked();
        void previousKeyFrameButtonClicked();
        void nextKeyFrameButtonClicked();
        void currFaceIdChanged(const QString& text);
        void toggleKeyframe(bool checked);
        void newFile();
        void open();
        bool save();
        bool saveAs();
        void about();
        void toggleContours(bool toggled);
        void toggleBorders(bool toggled);
        void toggleSegmentation(bool toggled);
        void togglePostprocess(bool toggled);
        void alphaChanged(int n);
        
    private:
        QLabel* m_main_widget;
        QLabel* m_display_widget;
        QSlider* m_frame_slider;
        QSlider* m_hierarchy_slider;
        QLabel* m_frame_label;
        QLabel* m_curr_frame_label;
        QLabel* m_max_frame_label;
        QLabel* m_hierarchy_label;
        QLabel* m_curr_hierarchy_label;
        QLabel* m_max_hierarchy_label;
        QLabel* m_face_id_label;
        QComboBox* m_face_id_combobox;
        QToolButton* m_play_button;
        QToolButton* m_previous_keyframe_button;
        QToolButton* m_next_keyframe_button;
        QCheckBox* m_toggle_keyframe_checkbox;
        QLabel* m_keyframe_label;

        bool m_loop;
        bool m_refresh;
        bool m_slider_pause;
        bool m_update_pending;
        bool m_update_frame;
        bool m_update_face;
        bool m_debug;

        int m_curr_frame_ind;
        int m_next_frame_ind;
        int m_curr_hierarchy_level;
        int m_max_hierarchy_level;

        std::string m_curr_file;
        std::string m_output_dir;
        std::string m_video_file;
        std::string m_seg_file;
        std::string m_landmarks_file;

        // video
        std::unique_ptr<cv::VideoCapture> m_cap;
        std::unique_ptr<cv::Mat> m_scaled_frame;
        std::unique_ptr<cv::Mat> m_render_frame;
        std::unique_ptr<QImage> m_render_image;

        // View
        bool m_render_contours;
        bool m_render_borders;
        bool m_render_seg;
        bool m_postprocess;
        float m_alpha;

        // sfl
        std::shared_ptr<sfl::SequenceFaceLandmarks> m_sfl;
        std::vector<sfl::Frame*> m_sfl_frames;
        int m_main_face_id;

        // segmentation
        std::unique_ptr<segmentation::SegmentationReader> m_seg_reader;
        std::unique_ptr<segmentation::SegmentationDesc> m_seg_desc;
        std::unique_ptr<segmentation::SegmentationDesc> m_seg_hierarchy;
        int m_hierarchy_pos;

        // Face segmentation
        std::unique_ptr<Sequence> m_input_regions;
        std::unique_ptr<Sequence> m_edited_regions;
        //std::unique_ptr<Sequence> m_inserted_regions;
        //std::unique_ptr<Sequence> m_removed_regions;
        int m_edit_index;
        std::unique_ptr<std::vector<std::vector<cv::Point>>> m_face_boundary;
        std::unique_ptr<cv::Mat> m_face_map;
        std::unique_ptr<Keyframer> m_keyframer;
        //std::vector<int> m_keyframes;
        int m_curr_face_id;
        std::set<int> m_face_ids;

        int m_frame_width, m_frame_height;
        double m_fps;
        size_t m_total_frames;
	};

}   // namespace fvs

#endif  // __FVS_EDITOR_H__