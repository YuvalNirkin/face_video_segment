#ifndef __SFL_VIEWER_H__
#define __SFL_VIEWER_H__

#include "ui_fvs_editor.h"
#include "fvs_editor_states.h"

//#include <vsal/VideoStreamOpenCV.h>
#include <sfl/sequence_face_landmarks.h>

#include <string>

// Qt

namespace fvs
{
    class Editor : public QMainWindow, public Ui::Editor
    {
        Q_OBJECT

    public:
        Editor();
        void setupBl();

        void setInputPath(const std::string& input_path);
        void initLandmarks(const std::string& _landmarks_path);
        void initVideoSource(const std::string& _sequence_path);

    protected:
        void resizeEvent(QResizeEvent* event) Q_DECL_OVERRIDE;
        void timerEvent(QTimerEvent *event) Q_DECL_OVERRIDE;

    public slots:
        void open();
        void close();
        void playPause();
        void backward();
        void forward();
        void frameSliderChanged(int i);
        void toggleRenderParams(bool toggled);
        void render();

    public:
        EditorSM sm;
        std::string sequence_path;
        std::string landmarks_path;

        // Video
        //std::unique_ptr<vsal::VideoStreamOpenCV> vs;
        cv::Mat frame, resized_frame, landmarks_render_frame;
        cv::Mat render_frame;
        std::unique_ptr<QImage> render_image;
        int curr_frame_pos = 0;
        int total_frames = 0;
        double fps = 0.0;

        // sfl
        std::shared_ptr<sfl::SequenceFaceLandmarks> sfl;
        std::vector<sfl::Frame*> sfl_frames;
        cv::Scalar landmarks_color = cv::Scalar(0, 255, 0);
        cv::Scalar bbox_color = cv::Scalar(0, 0, 255);

        int timer_id = 0;
    };

}   // namespace sfl

#endif // __SFL_VIEWER_H__