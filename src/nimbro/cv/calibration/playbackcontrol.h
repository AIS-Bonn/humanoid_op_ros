// Playback control widget
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#ifndef PLAYBACKCONTROL_H
#define PLAYBACKCONTROL_H

#include <QtGui/QWidget>

#include <sensor_msgs/Image.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

class Calibration;
class QSlider;
class QMenu;
class QLabel;

class PlaybackControl : public QWidget
{
Q_OBJECT
public:
	explicit PlaybackControl(Calibration* parent);
	virtual ~PlaybackControl();

	bool isPaused();

	void takeLiveRawImage(const sensor_msgs::Image::Ptr& img);
	void takeLiveProcessedImage(const sensor_msgs::Image::Ptr& classes);

	//! Menu for application menu bar
	QMenu* menu()
	{ return m_menu; }
public slots:
	void selectSource(QAction* act);
	void selectViewMode(QAction* act);
	void toggleViewMode();

	void next();
	void prev();

	void seek(int idx);
private:
	void displayCurrent();

	Calibration* m_calib;
	QAction* m_act_pause;
	QAction* m_act_live;

	QAction* m_act_raw;
	QAction* m_act_proc;

	QSlider* m_slider;
	QLabel* m_label;

	QMenu* m_menu;

	int m_source;

	rosbag::Bag m_bag;
	rosbag::View* m_bagView;
	rosbag::View::iterator m_bagViewIterator;
	int m_cnt;
};

#endif
