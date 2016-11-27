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

#include <iostream> // Debug

// face segmentation
#include <face_video_segment.pb.h>

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
#include <QToolBar>
#include <QAction>
#include <QMenuBar>
#include <QStatusBar>
#include <QMessageBox>
#include <QSpinBox>

namespace fvs
{
    void Editor::createMenu()
    {
        //////////////////////////////////////////////////////////////////////////////
        // File Menu
        //////////////////////////////////////////////////////////////////////////////

        // New
        QMenu *fileMenu = menuBar()->addMenu(tr("&File"));
        QToolBar *fileToolBar = addToolBar(tr("File"));
        const QIcon newIcon = QIcon::fromTheme("document-new", QIcon(":/images/new.png"));
        QAction *newAct = new QAction(newIcon, tr("&New"), this);
        newAct->setShortcuts(QKeySequence::New);
        newAct->setStatusTip(tr("Create a new file"));
        connect(newAct, &QAction::triggered, this, &Editor::newFile);
        fileMenu->addAction(newAct);
        fileToolBar->addAction(newAct);

        // Open
        const QIcon openIcon = QIcon::fromTheme("document-open", QIcon(":/images/open.png"));
        QAction *openAct = new QAction(openIcon, tr("&Open..."), this);
        openAct->setShortcuts(QKeySequence::Open);
        openAct->setStatusTip(tr("Open an existing file"));
        connect(openAct, &QAction::triggered, this, &Editor::open);
        fileMenu->addAction(openAct);
        fileToolBar->addAction(openAct);

        // Save
        const QIcon saveIcon = QIcon::fromTheme("document-save", QIcon(":/images/save.png"));
        QAction *saveAct = new QAction(saveIcon, tr("&Save"), this);
        saveAct->setShortcuts(QKeySequence::Save);
        saveAct->setStatusTip(tr("Save the document to disk"));
        connect(saveAct, &QAction::triggered, this, &Editor::save);
        fileMenu->addAction(saveAct);
        fileToolBar->addAction(saveAct);

        // Save As
        const QIcon saveAsIcon = QIcon::fromTheme("document-save-as");
        QAction *saveAsAct = fileMenu->addAction(saveAsIcon, tr("Save &As..."), this, &Editor::saveAs);
        saveAsAct->setShortcuts(QKeySequence::SaveAs);
        saveAsAct->setStatusTip(tr("Save the document under a new name"));

        // Exit
        const QIcon exitIcon = QIcon::fromTheme("application-exit");
        QAction *exitAct = fileMenu->addAction(exitIcon, tr("E&xit"), this, &QWidget::close);
        exitAct->setShortcuts(QKeySequence::Quit);
        exitAct->setStatusTip(tr("Exit the application"));

        fileMenu->addSeparator();

        //////////////////////////////////////////////////////////////////////////////
        // View Menu
        //////////////////////////////////////////////////////////////////////////////
        QMenu *viewMenu = menuBar()->addMenu(tr("&View"));
        QToolBar *viewToolBar = addToolBar(tr("View"));

        // Contours
        const QIcon contoursIcon = QIcon::fromTheme("view-contours", QIcon(":/images/contours.png"));
        QAction *contoursAct = new QAction(contoursIcon, tr("&Contours"), this);
        contoursAct->setShortcut(QKeySequence(Qt::Key_C));
        contoursAct->setCheckable(true);
        contoursAct->setChecked(true);
        contoursAct->setStatusTip(tr("Show face contours"));
        connect(contoursAct, SIGNAL(toggled(bool)), this, SLOT(toggleContours(bool)));
        viewMenu->addAction(contoursAct);
        viewToolBar->addAction(contoursAct);

        // Borders
        const QIcon bordersIcon = QIcon::fromTheme("view-borders", QIcon(":/images/borders.png"));
        QAction *bordersAct = new QAction(bordersIcon, tr("&Border"), this);
        bordersAct->setShortcut(QKeySequence(Qt::Key_B));
        bordersAct->setCheckable(true);
        bordersAct->setChecked(true);
        bordersAct->setStatusTip(tr("Show borders"));
        connect(bordersAct, SIGNAL(toggled(bool)), this, SLOT(toggleBorders(bool)));
        viewMenu->addAction(bordersAct);
        viewToolBar->addAction(bordersAct);

        // Segmentation
        const QIcon segIcon = QIcon::fromTheme("view-seg", QIcon(":/images/segmentation.png"));
        QAction *segAct = new QAction(segIcon, tr("Se&gmentation"), this);
        segAct->setShortcut(QKeySequence(Qt::Key_S));
        segAct->setCheckable(true);
        segAct->setChecked(true);
        segAct->setStatusTip(tr("Show segmentation"));
        connect(segAct, SIGNAL(toggled(bool)), this, SLOT(toggleSegmentation(bool)));
        viewMenu->addAction(segAct);
        viewToolBar->addAction(segAct);

        // Alpha
        QSlider* alpha_slider = new QSlider(Qt::Horizontal, this);
        alpha_slider->setMinimum(0);
        alpha_slider->setMaximum(100);
        alpha_slider->setValue(25);
        m_alpha = alpha_slider->value() / 100.0f;
        alpha_slider->setMaximumWidth(64);
        alpha_slider->setStatusTip("Segmentation Opacity");
        alpha_slider->setFocusPolicy(Qt::NoFocus);
        connect(alpha_slider, SIGNAL(valueChanged(int)), this, SLOT(alphaChanged(int)));
        viewToolBar->addWidget(alpha_slider);

        // Postprocess
        const QIcon postIcon = QIcon::fromTheme("view-post", QIcon(":/images/postprocess.png"));
        QAction *postAct = new QAction(postIcon, tr("&Postprocess"), this);
        postAct->setShortcut(QKeySequence(Qt::Key_P));
        postAct->setCheckable(true);
        postAct->setChecked(false);
        postAct->setStatusTip(tr("Postprocess segmentation"));
        connect(postAct, SIGNAL(toggled(bool)), this, SLOT(togglePostprocess(bool)));
        viewMenu->addAction(postAct);
        viewToolBar->addAction(postAct);

        //////////////////////////////////////////////////////////////////////////////
        // Postprocessing Menu
        //////////////////////////////////////////////////////////////////////////////
        QMenu *postMenu = menuBar()->addMenu(tr("&Postprocessing"));
        QToolBar *postToolBar = addToolBar(tr("Postprocessing"));

        // Disconnected
        const QIcon disconnectedIcon = QIcon::fromTheme("Postprocessing-disconnected",
            QIcon(":/images/postprocess_disconnected.png"));
        m_disconnectedAct = new QAction(disconnectedIcon, tr("&Disconnected"), this);
        m_disconnectedAct->setCheckable(true);
        m_disconnectedAct->setChecked(true);
        m_disconnectedAct->setStatusTip(tr("Removed smaller disconnected segmented pixel components"));
        connect(m_disconnectedAct, SIGNAL(toggled(bool)), this, SLOT(toggleDisconnected(bool)));
        postMenu->addAction(m_disconnectedAct);
        postToolBar->addAction(m_disconnectedAct);

        // Holes
        const QIcon holesIcon = QIcon::fromTheme("Postprocessing-holes",
            QIcon(":/images/postprocess_holes.png"));
        m_holesAct = new QAction(holesIcon, tr("&Holes"), this);
        m_holesAct->setCheckable(true);
        m_holesAct->setChecked(true);
        m_holesAct->setStatusTip(tr("Fill holes in the segmentation"));
        connect(m_holesAct, SIGNAL(toggled(bool)), this, SLOT(toggleHoles(bool)));
        postMenu->addAction(m_holesAct);
        postToolBar->addAction(m_holesAct);

        // Smooth
        const QIcon smoothIcon = QIcon::fromTheme("Postprocessing-smooth",
            QIcon(":/images/postprocess_smooth.png"));
        m_smoothAct = new QAction(smoothIcon, tr("&Smooth"), this);
        m_smoothAct->setCheckable(true);
        m_smoothAct->setChecked(true);
        m_smoothAct->setStatusTip(tr("Smooth the segmentation"));
        connect(m_smoothAct, SIGNAL(toggled(bool)), this, SLOT(toggleSmooth(bool)));
        postMenu->addAction(m_smoothAct);
        postToolBar->addAction(m_smoothAct);

        // Smooth iterations
        postToolBar->addSeparator();
        QPixmap iterationsPixmap(":/images/iterations.png");
        m_smooth_iterations_label = new QLabel(this);
        m_smooth_iterations_label->setPixmap(iterationsPixmap);
        postToolBar->addWidget(m_smooth_iterations_label);
        m_smooth_iterations_spinbox = new QSpinBox(this);
        m_smooth_iterations_spinbox->setMinimum(1);
        m_smooth_iterations_spinbox->setMaximum(10);
        m_smooth_iterations_spinbox->setStatusTip("Smooth iterations");
        m_smooth_iterations_spinbox->setFocusPolicy(Qt::NoFocus);
        connect(m_smooth_iterations_spinbox, SIGNAL(valueChanged(int)), this, SLOT(smoothIterationsChanged(int)));
        postToolBar->addWidget(m_smooth_iterations_spinbox);

        // Smooth kernel radius
        postToolBar->addSeparator();
        QPixmap radiusPixmap(":/images/radius.png");
        m_smooth_kernel_radius_label = new QLabel(this);
        m_smooth_kernel_radius_label->setPixmap(radiusPixmap);
        postToolBar->addWidget(m_smooth_kernel_radius_label);
        m_smooth_kernel_radius_spinbox = new QSpinBox(this);
        m_smooth_kernel_radius_spinbox->setMinimum(1);
        m_smooth_kernel_radius_spinbox->setMaximum(10);
        m_smooth_kernel_radius_spinbox->setValue(2);
        m_smooth_kernel_radius_spinbox->setStatusTip("Smooth kernel radius");
        m_smooth_kernel_radius_spinbox->setFocusPolicy(Qt::NoFocus);
        connect(m_smooth_kernel_radius_spinbox, SIGNAL(valueChanged(int)), this, SLOT(smoothKernelRadiusChanged(int)));
        postToolBar->addWidget(m_smooth_kernel_radius_spinbox);

        //////////////////////////////////////////////////////////////////////////////
        // Help Menu
        //////////////////////////////////////////////////////////////////////////////
        QMenu *helpMenu = menuBar()->addMenu(tr("&Help"));
        QAction *aboutAct = helpMenu->addAction(tr("&About"), this, &Editor::about);
        aboutAct->setStatusTip(tr("Show the application's About box"));


        // Create status bar
        statusBar()->showMessage(tr("Ready"));
    }

    void Editor::newFile()
    {
    }

    void Editor::open()
    {
    }

    bool Editor::save()
    {
        if (m_curr_file.empty()) return saveAs();
        return saveFile(m_curr_file);
    }

    bool Editor::saveAs()
    {
        return false;
    }

    void Editor::about()
    {
        QMessageBox::about(this, tr("About Face Video Segmentation Editor"),
            tr("<b>Face Video Segmentation Editor</b> is used for editing "
                "the automatically selected regions for the face segmentation. "
                "This is especially important for hard cases such as: "
                "Partial occlusion, bald people, etc."));
    }

    void Editor::toggleContours(bool toggled)
    {
        m_render_contours = toggled;
        m_refresh = true;
        updateLater();
    }

    void Editor::toggleBorders(bool toggled)
    {
        m_render_borders = toggled;
        m_refresh = true;
        updateLater();
    }

    void Editor::toggleSegmentation(bool toggled)
    {
        m_render_seg = toggled;
        m_refresh = true;
        updateLater();
    }

    void Editor::togglePostprocess(bool toggled)
    {
        m_postprocess = toggled;
        m_refresh = true;
        updateLater();
    }

    void Editor::alphaChanged(int n)
    {
        m_alpha = (n / 100.0f);
        m_refresh = true;
        updateLater();
    }

    void Editor::toggleDisconnected(bool toggled)
    {
        Face& face = getFaceForEditing();
        initPostprocessing(face);
        face.mutable_postprocessing()->set_disconnected(toggled);
    }

    void Editor::toggleHoles(bool toggled)
    {
        Face& face = getFaceForEditing();
        initPostprocessing(face);
        face.mutable_postprocessing()->set_holes(toggled);
    }

    void Editor::toggleSmooth(bool toggled)
    {
        Face& face = getFaceForEditing();
        initPostprocessing(face);
        face.mutable_postprocessing()->set_smooth(toggled);
    }

    void Editor::smoothIterationsChanged(int i)
    {
        Face& face = getFaceForEditing();
        initPostprocessing(face);
        face.mutable_postprocessing()->set_smooth_iterations(i);
    }

    void Editor::smoothKernelRadiusChanged(int r)
    {
        Face& face = getFaceForEditing();
        initPostprocessing(face);
        face.mutable_postprocessing()->set_smooth_kernel_radius(r);
    }
  
}   // namespace fvs

