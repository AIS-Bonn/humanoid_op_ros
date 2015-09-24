// Playback control widget
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include "playbackcontrol.h"

#include <QtGui/QToolBar>
#include <QtGui/QVBoxLayout>
#include <QtGui/QAction>
#include <QtGui/QFileDialog>
#include <QtGui/QMessageBox>
#include <QtGui/QInputDialog>
#include <QtGui/QSlider>
#include <QtGui/QMenu>
#include <QtGui/QLabel>

#include "calibration.h"

enum PlaybackSource {
	PS_LIVE,
	PS_FILE
};

PlaybackControl::PlaybackControl(Calibration* parent)
 : QWidget(parent)
 , m_calib(parent)
 , m_source(PS_LIVE)
 , m_bagView(0)
{
	QVBoxLayout* layout = new QVBoxLayout(this);
	QToolBar* toolBar = new QToolBar(this);
	m_menu = new QMenu(tr("Playback"), this);
	layout->addWidget(toolBar);

	// Actions
	m_act_pause = new QAction(
		QIcon::fromTheme("media-playback-pause"),
		tr("Pause"), this
	);
	m_act_pause->setCheckable(true);
	m_act_pause->setChecked(false);
	toolBar->addAction(m_act_pause);

	QAction* act_next = new QAction(
		QIcon::fromTheme("media-skip-forward"),
		tr("Next image"), this
	);
	QAction* act_prev = new QAction(
		QIcon::fromTheme("media-skip-backward"),
		tr("Previous image"), this
	);

	act_next->setShortcut(QKeySequence(Qt::Key_Right));
	act_prev->setShortcut(QKeySequence(Qt::Key_Left));

	connect(act_next, SIGNAL(triggered(bool)), SLOT(next()));
	connect(act_prev, SIGNAL(triggered(bool)), SLOT(prev()));

	toolBar->addAction(act_prev);
	toolBar->addAction(act_next);

	toolBar->addSeparator();

	// Sources
	QActionGroup* sourceGroup = new QActionGroup(this);
	m_act_live = new QAction(tr("Live"), sourceGroup);
	QAction* sourceFile = new QAction(tr("File"), sourceGroup);
	m_act_live->setData(PS_LIVE);
	m_act_live->setCheckable(true);
	m_act_live->setChecked(true);
	sourceFile->setData(PS_FILE);
	sourceFile->setCheckable(true);

	connect(
		sourceGroup, SIGNAL(selected(QAction*)),
		SLOT(selectSource(QAction*))
	);

	toolBar->addActions(sourceGroup->actions());

	toolBar->addSeparator();

	QActionGroup* viewGroup = new QActionGroup(this);
	m_act_raw = new QAction(tr("Raw"), viewGroup);
	m_act_raw->setData(Calibration::VM_RAW);
	m_act_raw->setCheckable(true);
	m_act_raw->setChecked(true);
	m_act_proc = new QAction(tr("Processed"), viewGroup);
	m_act_proc->setData(Calibration::VM_PROCESSED);
	m_act_proc->setCheckable(true);

	connect(
		viewGroup, SIGNAL(selected(QAction*)),
		SLOT(selectViewMode(QAction*))
	);

	toolBar->addActions(viewGroup->actions());

	m_menu->addActions(toolBar->actions());

	// Slider + Label
	QHBoxLayout* sliderLayout = new QHBoxLayout(this);
	m_label = new QLabel("00000", this);
	m_label->setFont(QFont("monospace"));
	m_slider = new QSlider(Qt::Horizontal, this);
	m_slider->setDisabled(true);
	sliderLayout->addWidget(m_label);
	sliderLayout->addWidget(m_slider);
	layout->addLayout(sliderLayout);

	connect(m_slider, SIGNAL(valueChanged(int)), SLOT(seek(int)));

	setSizePolicy(
		QSizePolicy::Expanding,
		QSizePolicy::Fixed
	);
}

PlaybackControl::~PlaybackControl()
{
	delete m_bagView;
}

bool PlaybackControl::isPaused()
{
	return m_act_pause->isChecked();
}

void PlaybackControl::selectSource(QAction* act)
{
	int source = act->data().toInt();
	if(source == PS_LIVE)
	{
		m_act_pause->setDisabled(false);
		m_slider->setDisabled(true);

		delete m_bagView;
		m_bagView = 0;
		m_bag.close();
	}
	else if(source == PS_FILE)
	{
		m_act_pause->setChecked(true);
		m_act_pause->setDisabled(true);

		QString file = QFileDialog::getOpenFileName(
			this, tr("Open video"),
			QString(), tr("Bag files (*.bag)")
		);

		if(file.isNull())
		{
			m_act_live->trigger();
			return;
		}

		try
		{
			m_bag.open(file.toLocal8Bit().constData());
		}
		catch(rosbag::BagException& e)
		{
			QMessageBox::critical(
				this, tr("Error"),
				tr("Could not open bag file: %1").arg(e.what())
			);
			m_act_live->trigger();
			return;
		}

		QStringList availableTopics;
		rosbag::View view(m_bag);
		int cnt = 0;
		for(rosbag::View::iterator it = view.begin(); it != view.end(); ++it)
		{
			if(++cnt > 40)
				break;
			const std::string& topic = (*it).getTopic();
			if(!availableTopics.contains(topic.c_str()))
				availableTopics << topic.c_str();
		}

		QInputDialog dialog(this);
		dialog.setWindowTitle(tr("Choose topic"));
		dialog.setLabelText(tr("Choose the topic to listen on for raw images"));
		dialog.setComboBoxItems(availableTopics);
		dialog.setComboBoxEditable(false);
		dialog.setOption(QInputDialog::UseListViewForComboBoxItems, true);

		if(dialog.exec() != QDialog::Accepted)
		{
			m_act_live->trigger();
			return;
		}

		std::vector<std::string> topics;
		topics.push_back(dialog.textValue().toLocal8Bit().constData());

		m_bagView = new rosbag::View(m_bag, rosbag::TopicQuery(topics));
		m_bagViewIterator = m_bagView->begin();

		if(m_bagViewIterator == m_bagView->end())
		{
			QMessageBox::critical(
				this, tr("Error"),
				tr("Bag file contains no video frames")
			);
			m_act_live->trigger();
			return;
		}
		m_cnt = 0;

		int numFrames = 0;
		for(rosbag::View::iterator it = m_bagView->begin(); it != m_bagView->end(); ++it)
			numFrames++;

		m_slider->setValue(0);
		m_slider->setMaximum(numFrames-1);
		m_slider->setEnabled(true);

		displayCurrent();
	}

	m_source = source;
}

void PlaybackControl::takeLiveProcessedImage(const sensor_msgs::Image::Ptr& classes)
{
	if(m_source == PS_LIVE)
		m_calib->displayClasses(classes);
}

void PlaybackControl::takeLiveRawImage(const sensor_msgs::Image::Ptr& img)
{
	if(m_source == PS_LIVE)
		m_calib->displayImage(img);
}

void PlaybackControl::displayCurrent()
{
	sensor_msgs::Image::Ptr img = (*m_bagViewIterator).instantiate<sensor_msgs::Image>();
	m_calib->displayImage(img, true);
	m_slider->setValue(m_cnt);
	m_label->setText(QString("%1").arg(m_cnt, 5, 10, QChar('0')));
}

void PlaybackControl::next()
{
	if(m_source != PS_FILE)
		return;

	rosbag::View::iterator it = m_bagViewIterator;
	it++;

	if(it == m_bagView->end())
		return;

	m_bagViewIterator = it;
	m_cnt++;

	displayCurrent();
}

void PlaybackControl::prev()
{
	if(m_source != PS_FILE)
		return;

	if(m_bagViewIterator != m_bagView->begin())
	{
		m_bagViewIterator = m_bagView->begin();
		for(int i = 0; i < m_cnt-1; ++i)
			m_bagViewIterator++;
		m_cnt--;
	}

	displayCurrent();
}

void PlaybackControl::seek(int idx)
{
	if(m_source != PS_FILE)
		return;

	m_bagViewIterator = m_bagView->begin();

	for(int i = 0; i < idx; ++i)
	{
		m_bagViewIterator++;
		if(m_bagViewIterator == m_bagView->end())
		{
			QMessageBox::critical(
				this, tr("Error"),
				tr("Seek past end of file")
			);
			return;
		}
	}

	m_cnt = idx;

	displayCurrent();
}

void PlaybackControl::selectViewMode(QAction* act)
{
	m_calib->setViewMode((Calibration::ViewMode)act->data().toInt());

	if(m_source == PS_FILE)
		displayCurrent();
}

void PlaybackControl::toggleViewMode()
{
	if(m_act_proc->isChecked())
		m_act_raw->trigger();
	else
		m_act_proc->trigger();
}




