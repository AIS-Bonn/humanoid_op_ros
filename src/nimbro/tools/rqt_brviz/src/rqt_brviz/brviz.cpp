/*
 * Copyright (c) 2011, Dorian Scholz, TU Darmstadt
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the TU Darmstadt nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <OGRE/OgreLogManager.h>

#include <QCloseEvent>
#include <QMenuBar>

#include <pluginlib/class_list_macros.h>
#include <boost/program_options.hpp>
#include <stdlib.h>     /* getenv */
#include <rqt_brviz/brviz.h>
#include <string>
#include <ros/console.h>

namespace rqt_brviz {

BRViz::BRViz()
  : rqt_gui_cpp::Plugin()
  , context_(0)
  , widget_(0)
  , log_(0)
  , hide_menu_(false)
  , ogre_log_(false)
{
  setObjectName("BRViz");
}

BRViz::~BRViz()
{
  Ogre::LogManager* log_manager = Ogre::LogManager::getSingletonPtr();
  if (log_manager && log_)
  {
    log_manager->destroyLog(log_);
  }
}

void BRViz::initPlugin(qt_gui_cpp::PluginContext& context)
{
  context_ = &context;

  parseArguments();

  // prevent output of Ogre stuff to console
  Ogre::LogManager* log_manager = Ogre::LogManager::getSingletonPtr();
  if (!log_manager)
  {
    log_manager = new Ogre::LogManager();
  }
  QString filename = QString("rqt_brviz_ogre") + (context.serialNumber() > 1 ? QString::number(context.serialNumber()) : QString("")) + QString(".log");
  log_ = log_manager->createLog(filename.toStdString().c_str(), false, false, !ogre_log_);

  widget_ = new rviz::VisualizationFrame();

  // create own menu bar to disable native menu bars on Unity and Mac
  QMenuBar* menu_bar = new QMenuBar();
  menu_bar->setNativeMenuBar(false);
  menu_bar->setVisible(!hide_menu_);
  widget_->setMenuBar(menu_bar);
  
  // ADDITION: Load a particular rviz configuration file if required
  std::string fileToLoadPath;
  ros::param::get("/brviz_path", fileToLoadPath);
  if(fileToLoadPath == ".")
  {
    ROS_INFO("Loading default RViz configuration file");
    widget_->initialize();
  }
  else
  {
    ROS_INFO("Loading RViz configuration file: %s", fileToLoadPath.c_str());
    widget_->initialize(fileToLoadPath.c_str());
  }

  // disable quit action in menu bar
  QMenu* menu = 0;
  {
    // find first menu in menu bar
    const QObjectList& children = menu_bar->children();
    for (QObjectList::const_iterator it = children.begin(); !menu && it != children.end(); it++)
    {
      menu = dynamic_cast<QMenu*>(*it);
    }
  }
  if (menu)
  {
    // hide last action in menu
    const QObjectList& children = menu->children();
    if (!children.empty())
    {
      QAction* action = dynamic_cast<QAction*>(children.last());
      if (action)
      {
        action->setVisible(false);
      }
    }
  }

  widget_->setWindowTitle("BRViz[*]");
  if (context.serialNumber() != 1)
  {
    widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
  }
  context.addWidget(widget_);

  // trigger deleteLater for plugin when widget or frame is closed
  widget_->installEventFilter(this);
}

void BRViz::shutdownPlugin()
{
}

void BRViz::parseArguments()
{
  namespace po = boost::program_options;

  const QStringList& qargv = context_->argv();

  const int argc = qargv.count();

  // temporary storage for args obtained from qargv - since each QByteArray
  // owns its storage, we need to keep these around until we're done parsing
  // args using boost::program_options
  std::vector<QByteArray> argv_array;
  const char *argv[argc+1];
  argv[0] = ""; // dummy program name

  for (int i = 0; i < argc; ++i)
  {
    argv_array.push_back(qargv.at(i).toLocal8Bit());
    argv[i+1] = argv_array[i].constData();
  }

  po::variables_map vm;
  po::options_description options;
  options.add_options()
    ("display-config,d", po::value<std::string>(), "")
    ("hide-menu,m", "")
    ("ogre-log,l", "");

  try
  {
    po::store(po::parse_command_line(argc+1, argv, options), vm);
    po::notify(vm);

    if (vm.count("hide-menu"))
    {
      hide_menu_ = true;
    }

    if (vm.count("ogre-log"))
    {
      ogre_log_ = true;
    }
  }
  catch (std::exception& e)
  {
    ROS_ERROR("Error parsing command line: %s", e.what());
  }
}

bool BRViz::eventFilter(QObject* watched, QEvent* event)
{
  if (watched == widget_ && event->type() == QEvent::Close)
  {
    event->ignore();
    context_->closePlugin();
    return true;
  }

  return QObject::eventFilter(watched, event);
}

}

PLUGINLIB_EXPORT_CLASS(rqt_brviz::BRViz, rqt_gui_cpp::Plugin)
