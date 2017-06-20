/*
 * Copyright (c) 2008, Willow Garage, Inc.
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

#include <QApplication>
#include <QMenu>
#include <QTimer>
#include <QLineEdit>

#include <OgreSceneManager.h>
#include <OgreCamera.h>


#include "rviz/display.h"
#include "rviz/view_controller.h"
#include "rviz/viewport_mouse_event.h"
#include "rviz/visualization_manager.h"
#include "rviz/window_manager_interface.h"
#include "std_msgs/Int16.h"
#include "ros/ros.h"

#include "rviz/render_panel.h"

namespace rviz
{

RenderPanel::RenderPanel( QWidget* parent )
  : QtOgreRenderWindow( parent )
  , mouse_x_( 0 )
  , mouse_y_( 0 )
  , context_( 0 )
  , scene_manager_( 0 )
  , view_controller_( 0 )
  , fake_mouse_move_event_timer_( new QTimer() )
  , default_camera_(0)
  , context_menu_visible_(false)
{
  setFocus( Qt::OtherFocusReason );
  
  //add keyinme 20170620
  //set position
  label_battery -> move(0,0);
  label_temperature -> move(0,25);
  label_speed -> move(0,50);
  label_battery -> adjustSize();
  label_temperature -> adjustSize();
  label_speed -> adjustSize();
  //set Font color
  label_battery -> setStyleSheet("QLineEdit{color:white;border:0px;background:rgba(0,0,0,127)}");  
  label_temperature -> setStyleSheet("QLineEdit{color:white;border:0px;background:rgba(0,0,0,127)}");
  label_speed -> setStyleSheet("QLineEdit{color:white;border:0px;background:rgba(0,0,0,127)}");
}

RenderPanel::~RenderPanel()
{
  delete fake_mouse_move_event_timer_;
  if( scene_manager_ && default_camera_ )
  {
    scene_manager_->destroyCamera( default_camera_ );
  }
  if( scene_manager_ )
  {
    scene_manager_->removeListener( this );
  }
}

void RenderPanel::initialize(Ogre::SceneManager* scene_manager, DisplayContext* context)
{
  context_ = context;
  scene_manager_ = scene_manager;
  scene_manager_->addListener( this );

  std::stringstream ss;
  static int count = 0;
  ss << "RenderPanelCamera" << count++;
  default_camera_ = scene_manager_->createCamera(ss.str());
  default_camera_->setNearClipDistance(0.01f);
  default_camera_->setPosition(0, 10, 15);
  default_camera_->lookAt(0, 0, 0);

  setCamera( default_camera_ );

  connect( fake_mouse_move_event_timer_, SIGNAL( timeout() ), this, SLOT( sendMouseMoveEvent() ));
  fake_mouse_move_event_timer_->start( 33 /*milliseconds*/ );

  //add keyinme 2017-06-20
  subscribe();
}

void RenderPanel::incomingStatus(const std_msgs::Int16::ConstPtr& status_info)
{
  //const std_msgs::Int16& current_status = *status_info;
  QString bat,tem,spe;
  bat="Battery: " + QString::number(status_info->data) + "%";
  tem="Temperature: " + QString::number(status_info->data) + "C";
  spe="Speed: " + QString::number(status_info->data) + "m/s";
  label_battery -> setText(bat);
  label_temperature -> setText(tem);
  label_speed -> setText(spe);   
}

void RenderPanel::subscribe()
{

  std::string status = "status";//status_topic_property_->getTopicStd();
  status_sub.shutdown();
    try
    {
      status_sub = update_nh.subscribe( status, 1000, &RenderPanel::incomingStatus, this );
      //setStatus( StatusProperty::Ok, "Topic", "OK" );
    }
    catch( ros::Exception& e )
    {
      ROS_INFO("subscribing Error"); //setStatus( StatusProperty::Error, "Topic", QString("Error subscribing: ") + e.what() );
    }
}

void RenderPanel::unsubscribe()
{
  status_sub.shutdown();
}

void RenderPanel::sendMouseMoveEvent()
{
  QPoint cursor_pos = QCursor::pos();
  QPoint mouse_rel_widget = mapFromGlobal( cursor_pos );
  if( rect().contains( mouse_rel_widget ))
  {
    bool mouse_over_this = false;
    QWidget *w = QApplication::widgetAt( cursor_pos );
    while( w )
    {
      if( w == this )
      {
        mouse_over_this = true;
        break;
      }
      w = w->parentWidget();
    }
    if( !mouse_over_this )
    {
      return;
    }

    QMouseEvent fake_event( QEvent::MouseMove,
                            mouse_rel_widget,
                            Qt::NoButton,
                            QApplication::mouseButtons(),
                            QApplication::keyboardModifiers() );
    onRenderWindowMouseEvents( &fake_event );
  }
}
void RenderPanel::leaveEvent ( QEvent * event )
{
  setCursor( Qt::ArrowCursor );
  if ( context_ )
  {
    context_->setStatus("");
  }
}

void RenderPanel::onRenderWindowMouseEvents( QMouseEvent* event )
{
  int last_x = mouse_x_;
  int last_y = mouse_y_;

  mouse_x_ = event->x();
  mouse_y_ = event->y();

  if (context_)
  {
    setFocus( Qt::MouseFocusReason );

    ViewportMouseEvent vme(this, getViewport(), event, last_x, last_y);
    context_->handleMouseEvent(vme);
    event->accept();
  }
}

void RenderPanel::wheelEvent( QWheelEvent* event )
{
  int last_x = mouse_x_;
  int last_y = mouse_y_;

  mouse_x_ = event->x();
  mouse_y_ = event->y();

  if (context_)
  {
    setFocus( Qt::MouseFocusReason );

    ViewportMouseEvent vme(this, getViewport(), event, last_x, last_y);
    context_->handleMouseEvent(vme);
    event->accept();
  }
}

void RenderPanel::keyPressEvent( QKeyEvent* event )
{
  if( context_ )
  {
    context_->handleChar( event, this );
  }
}

void RenderPanel::setViewController( ViewController* controller )
{
  view_controller_ = controller;

  if( view_controller_ )
  {
    setCamera( view_controller_->getCamera() );
    view_controller_->activate();
  }
  else
  {
    setCamera( NULL );
  }
}

void RenderPanel::showContextMenu( boost::shared_ptr<QMenu> menu )
{
  boost::mutex::scoped_lock lock(context_menu_mutex_);
  context_menu_ = menu;
  context_menu_visible_ = true;

  QApplication::postEvent( this, new QContextMenuEvent( QContextMenuEvent::Mouse, QPoint() ));
}

void RenderPanel::onContextMenuHide()
{
  context_menu_visible_ = false;
}

bool RenderPanel::contextMenuVisible()
{
  return context_menu_visible_;
}

void RenderPanel::contextMenuEvent( QContextMenuEvent* event )
{
  boost::shared_ptr<QMenu> context_menu;
  {
    boost::mutex::scoped_lock lock(context_menu_mutex_);
    context_menu.swap(context_menu_);
  }

  if ( context_menu )
  {
    connect( context_menu.get(), SIGNAL( aboutToHide() ), this, SLOT( onContextMenuHide() ) );
    context_menu->exec( QCursor::pos() );
  }
}

void RenderPanel::sceneManagerDestroyed( Ogre::SceneManager* destroyed_scene_manager )
{
  if( destroyed_scene_manager == scene_manager_ )
  {
    scene_manager_ = NULL;
    default_camera_ = NULL;
    setCamera( NULL );
  }
}

} // namespace rviz
