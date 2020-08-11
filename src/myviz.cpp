/*
 * Copyright (c) 2012, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <QColor>
#include <QSlider>
#include <QLabel>
#include <QGridLayout>
#include <QVBoxLayout>

#include "rviz/visualization_manager.h"
#include "rviz/render_panel.h"
#include "rviz/display.h"
#include "rviz/view_manager.h"
#include "qt_app/myviz.h"

// BEGIN_TUTORIAL
// Constructor for MyViz.  This does most of the work of the class.
MyViz::MyViz( QWidget* parent )
  : QWidget( parent )
{

  // Construct and lay out render panel.
  render_panel_ = new rviz::RenderPanel();
  QVBoxLayout* main_layout = new QVBoxLayout;

  main_layout->addWidget( render_panel_ );
  setLayout( main_layout );

  manager_ = new rviz::VisualizationManager( render_panel_ );
  render_panel_->initialize( manager_->getSceneManager(), manager_ );
  manager_->initialize();
  manager_->startUpdate();

  SetGlobalOptions("livox_frame", QColor(Qt::black), 30);

 //设置视角
  view_manager_ = manager_->getViewManager();
  view_manager_->setRenderPanel(render_panel_);
  view_manager_->setCurrentViewControllerType("rviz/Orbit");
  view_manager_->getCurrent()->subProp("Distance")->setValue("17.135");
  view_manager_->getCurrent()->subProp("Yaw")->setValue("6.245");
  view_manager_->getCurrent()->subProp("Pitch")->setValue("1.3948");
  view_manager_->getCurrent()->subProp("Focal Point")->setValue("3.0;0;-1.0");

  // Create a Grid display.
  grid_ = manager_->createDisplay( "rviz/Grid", "adjustable grid", true );
  ROS_ASSERT( grid_ != NULL );

  // Configure the GridDisplay the way we like it.
  grid_->subProp( "Line Style" )->setValue( "Lines" );
  grid_->subProp( "Color" )->setValue( QColor( Qt::gray ) );
  grid_->subProp("Cell Size")->setValue("1");
  grid_->subProp("Plane Cell Count")->setValue("15");
  grid_->subProp("Offset")->setValue("1;0;0");

  //Create a Axes display
  axes_ = manager_->createDisplay("rviz/Axes","axes",true);
  axes_ ->subProp("Length")->setValue("1");
  axes_ ->subProp("Radius")->setValue("0.05");

  ROI_marker_ = manager_->createDisplay("rviz/Marker", "marker", true);
  ROI_marker_ ->subProp("Marker Topic")->setValue("/start_ROI");

   EnableRawPointCloud2("/livox/lidar_1HDDH1200103421");
  //EnableModelPointCloud2("/livox/lidar_1HDDH1200103421");

}

// Destructor.
MyViz::~MyViz()
{
  delete manager_;
}




rviz::Display* MyViz::EnableRawPointCloud2(QString topic) {
    raw_pointcloud2_ = manager_->createDisplay("rviz/PointCloud2","PointCloud2", true);
    raw_pointcloud2_->subProp("Topic")->setValue(topic);
    raw_pointcloud2_->subProp("Style")->setValue("Flag Squares");
    raw_pointcloud2_->subProp("Size (m)")->setValue("0.01");
    raw_pointcloud2_->subProp("Decay Time")->setValue("1");
    raw_pointcloud2_->subProp("Position Transformer")->setValue("XYZ");
    raw_pointcloud2_->subProp("Color Transformer")->setValue("AxisColor");
    raw_pointcloud2_->subProp("Axis")->setValue("Y");
    return raw_pointcloud2_;
}

rviz::Display* MyViz::EnableModelPointCloud2(QString topic) {
    model_pointcloud2_ = manager_->createDisplay("rviz/PointCloud2","PointCloud2", true);
    model_pointcloud2_->subProp("Topic")->setValue(topic);
    model_pointcloud2_->subProp("Style")->setValue("Spheres");
    model_pointcloud2_->subProp("Size (m)")->setValue("0.01");
    model_pointcloud2_->subProp("Decay Time")->setValue("1");
    model_pointcloud2_->subProp("Position Transformer")->setValue("XYZ");
    model_pointcloud2_->subProp("Color Transformer")->setValue("Intensity");
   // model_pointcloud2_->subProp("Channel Name")->setValue("Intensity");
    return model_pointcloud2_;
}

void MyViz::SetGlobalOptions(QString frame_name, QColor backColor, int frame_rate) {
    manager_->setFixedFrame(frame_name);
    manager_->setProperty("Background Color",backColor);
    manager_->setProperty("Frame Rate",frame_rate);
    manager_->startUpdate();
}
