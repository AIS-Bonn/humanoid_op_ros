//This package is based on rqt_image_view package. (for copyright notes, please read the ReadMe.txt)
//     Author: Hafez Farazi <farazi@ais.uni-bonn.de>

#include <rqt_vision_module/main.hpp>

#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/master.h>

#include <QScrollBar>
#include <QFileDialog>
#include <QMessageBox>
#include <QPainter>

#include <stdio.h>

namespace rqt_vision_module
{

ImageView::ImageView() :
		rqt_gui_cpp::Plugin(), widget_(0)
{
	setObjectName("ImageView");
}

void ImageView::initPlugin(qt_gui_cpp::PluginContext& context)
{
	widget_ = new QWidget();
	ui_.setupUi(widget_);
	
	widget_->setMouseTracking(true);

	if (context.serialNumber() > 1)
	{
		widget_->setWindowTitle(
				widget_->windowTitle() + " ("
						+ QString::number(context.serialNumber()) + ")");
	}
	context.addWidget(widget_);
	ui_.image_frame->installEventFilter(this);
	ui_.image_frame->setMouseTracking(true);
	ui_.topics_combo_box->setCurrentIndex(ui_.topics_combo_box->findText(""));
	connect(ui_.topics_combo_box, SIGNAL(currentIndexChanged(int)), this,
			SLOT(onTopicChanged(int)));

	ui_.refresh_topics_push_button->setIcon(QIcon::fromTheme("view-refresh"));
	connect(ui_.refresh_topics_push_button, SIGNAL(pressed()), this,
			SLOT(updateTopicList()));

	ui_.zoom_1_push_button->setIcon(QIcon::fromTheme("zoom-original"));
	connect(ui_.zoom_1_push_button, SIGNAL(toggled(bool)), this,
			SLOT(onZoom1(bool)));

	connect(ui_.dynamic_range_check_box, SIGNAL(toggled(bool)), this,
			SLOT(onDynamicRange(bool)));

	ui_.save_as_image_push_button->setIcon(QIcon::fromTheme("image-x-generic"));
	connect(ui_.save_as_image_push_button, SIGNAL(pressed()), this,
			SLOT(saveImage()));

	// set topic name if passed in as argument
	const QStringList& argv = context.argv();
	if (!argv.empty())
	{
		arg_topic_name = argv[0];
		// add topic name to list if not yet in
		int index = ui_.topics_combo_box->findText(arg_topic_name);
		if (index == -1)
		{
			QString label(arg_topic_name);
			label.replace(" ", "/");
			ui_.topics_combo_box->addItem(label, QVariant(arg_topic_name));
			ui_.topics_combo_box->setCurrentIndex(
					ui_.topics_combo_box->findText(arg_topic_name));
		}
	}
	
	// Save time when this instance of vision module was started
	ros::WallTime time = ros::WallTime::now();
	time_started = time.toNSec();
	
	// Set up subscribers
	ros::NodeHandle nh("~");
	console_subscriber = nh.subscribe("/rqt_vision_module/console", 10, &ImageView::consoleMsgReceived, this);
	clear_console_subscriber = nh.subscribe("/rqt_vision_module/clear_console", 10, &ImageView::handleClearConsole, this);
	
	// Set up publishers
	gui_event_publisher = nh.advertise<rqt_vision_module::GuiEvent>("/rqt_vision_module/gui_events", 1);
	
	// Set up connections
	connect(ui_.image_frame, SIGNAL(mouseEvent(MouseEvent,float,float))
				, this, SLOT(onMouseEvent(MouseEvent,float,float)));
	
	connect(ui_.clear_console_button, SIGNAL(clicked(bool)), this, SLOT(clearConsole()));
	
	updateTopicList();
}

// When new message arrives - append it to the end of console
void ImageView::consoleMsgReceived(const rqt_vision_module::ConsoleMsgConstPtr &msg)
{
	if(ui_.dont_update_console_check_box->isChecked())
		return;
	
	QString text = QString::fromStdString(msg->message);
	int scroll_bar_pos = ui_.textBrowser->verticalScrollBar()->value();
	
	// Split by 'new line' character
	QStringList pieces = text.split("\r\n");
	
	ui_.textBrowser->moveCursor(QTextCursor::End);
	ui_.textBrowser->insertHtml(prepareForConsole(pieces.first(), msg->r, msg->g, msg->b, msg->font_size));
	ui_.textBrowser->moveCursor(QTextCursor::End);
	
	for(int i = 1; i < pieces.size(); i++)
		ui_.textBrowser->append(prepareForConsole(pieces.at(i), msg->r, msg->g, msg->b, msg->font_size));
	
	if(!ui_.auto_scroll_check_box->isChecked())
		ui_.textBrowser->verticalScrollBar()->setValue(scroll_bar_pos); // Restore scroll bar position
}

void ImageView::handleClearConsole(const std_msgs::EmptyConstPtr &msg)
{
	clearConsole();
}

void ImageView::clearConsole()
{
	ui_.textBrowser->clear();
}

void ImageView::onMouseEvent(MouseEvent event, float x, float y)
{
	rqt_vision_module::GuiEvent event_msg;
	
	// Set time
	event_msg.time_started = time_started;
	
	// Set curently selected topic
	event_msg.topic = ui_.topics_combo_box->currentText().toStdString();
	
	// Set mouse position
	event_msg.mouse_x = x;
	event_msg.mouse_y = y;
	
	ui_.cursorPosLabel->setText(QString("Pos: (%1, %2)")
									.arg(QString::number(x, 'f', 3)).arg(QString::number(y, 'f', 3)));
	
	// Set event
	event_msg.mouse_event = event;
	
	// Set buttons (ctrl/shift/alt) currently pressed
	event_msg.shift_pressed = false;
	event_msg.ctrl_pressed  = false;
	event_msg.alt_pressed   = false;
	
	Qt::KeyboardModifiers modifier = QApplication::keyboardModifiers();
	
	if(modifier & Qt::ShiftModifier)
	{
		//std::cout << "shift" << std::endl;
		event_msg.shift_pressed = true;
	}
	if(modifier & Qt::ControlModifier)
	{
		//std::cout << "control" << std::endl;
		event_msg.ctrl_pressed = true;
	}
	if(modifier & Qt::AltModifier)
	{
		//std::cout << "alt" << std::endl;
		event_msg.alt_pressed = true;
	}
	
	gui_event_publisher.publish(event_msg);
}

void ImageView::shutdownPlugin()
{
	subscriber_.shutdown();
	console_subscriber.shutdown();
	gui_event_publisher.shutdown();
}

void ImageView::saveSettings(qt_gui_cpp::Settings& plugin_settings,
		qt_gui_cpp::Settings& instance_settings) const
{
	QString topic = ui_.topics_combo_box->currentText();
	//qDebug("ImageView::saveSettings() topic '%s'", topic.toStdString().c_str());
	instance_settings.setValue("topic", topic);
	instance_settings.setValue("zoom1", ui_.zoom_1_push_button->isChecked());
	instance_settings.setValue("dynamic_range",
			ui_.dynamic_range_check_box->isChecked());
	instance_settings.setValue("max_range",
			ui_.max_range_double_spin_box->value());
}

void ImageView::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
		const qt_gui_cpp::Settings& instance_settings)
{
	bool zoom1_checked = instance_settings.value("zoom1", false).toBool();
	ui_.zoom_1_push_button->setChecked(zoom1_checked);

	bool dynamic_range_checked =
			instance_settings.value("dynamic_range", false).toBool();
	ui_.dynamic_range_check_box->setChecked(dynamic_range_checked);

	double max_range = instance_settings.value("max_range",
			ui_.max_range_double_spin_box->value()).toDouble();
	ui_.max_range_double_spin_box->setValue(max_range);

	QString topic = instance_settings.value("topic", "").toString();
	// don't overwrite topic name passed as command line argument
	if (!arg_topic_name.isEmpty())
	{
		arg_topic_name = "";
	}
	else
	{
		//qDebug("ImageView::restoreSettings() topic '%s'", topic.toStdString().c_str());
		selectTopic(topic);
	}
}

void ImageView::updateTopicList()
{
	QSet<QString> message_types;
	message_types.insert("sensor_msgs/Image");
	QSet<QString> message_sub_types;
	message_sub_types.insert("sensor_msgs/CompressedImage");

	// get declared transports
	QList<QString> transports;
	image_transport::ImageTransport it(getNodeHandle());
	std::vector<std::string> declared = it.getDeclaredTransports();
	for (std::vector<std::string>::const_iterator it = declared.begin();
			it != declared.end(); it++)
	{
		//qDebug("ImageView::updateTopicList() declared transport '%s'", it->c_str());
		QString transport = it->c_str();

		// strip prefix from transport name
		QString prefix = "image_transport/";
		if (transport.startsWith(prefix))
		{
			transport = transport.mid(prefix.length());
		}
		transports.append(transport);
	}

	QString selected = ui_.topics_combo_box->currentText();

	// fill combo box
	QList<QString> topics = getTopics(message_types, message_sub_types,
			transports).values();
	topics.append("");
	qSort(topics);
	ui_.topics_combo_box->clear();
	for (QList<QString>::const_iterator it = topics.begin(); it != topics.end();
			it++)
	{
		QString label(*it);
		
		// Do not include topics with "theora" or "compressed"
		if(label.contains("theora") || label.contains("compressed"))
			continue;
		
		label.replace(" ", "/");
		ui_.topics_combo_box->addItem(label, QVariant(*it));
	}

	// restore previous selection
	selectTopic(selected);
}

QList<QString> ImageView::getTopicList(const QSet<QString>& message_types,
		const QList<QString>& transports)
{
	QSet<QString> message_sub_types;
	return getTopics(message_types, message_sub_types, transports).values();
}

QSet<QString> ImageView::getTopics(const QSet<QString>& message_types,
		const QSet<QString>& message_sub_types,
		const QList<QString>& transports)
{
	ros::master::V_TopicInfo topic_info;
	ros::master::getTopics(topic_info);

	QSet<QString> all_topics;
	for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin();
			it != topic_info.end(); it++)
	{
		all_topics.insert(it->name.c_str());
	}

	QSet<QString> topics;
	for (ros::master::V_TopicInfo::const_iterator it = topic_info.begin();
			it != topic_info.end(); it++)
	{
		if (message_types.contains(it->datatype.c_str()))
		{
			QString topic = it->name.c_str();

			// add raw topic
			if (!topic.contains("compressedDepth"))
				topics.insert(topic);
			//qDebug("ImageView::getTopics() raw topic '%s'", topic.toStdString().c_str());

			// add transport specific sub-topics
			for (QList<QString>::const_iterator jt = transports.begin();
					jt != transports.end(); jt++)
			{
				if (all_topics.contains(topic + "/" + *jt))
				{
					QString sub = topic + " " + *jt;
					if (!sub.contains("compressedDepth"))
						topics.insert(sub);
					//qDebug("ImageView::getTopics() transport specific sub-topic '%s'", sub.toStdString().c_str());
				}
			}
		}
		if (message_sub_types.contains(it->datatype.c_str()))
		{
			QString topic = it->name.c_str();
			int index = topic.lastIndexOf("/");
			if (index != -1)
			{
				topic.replace(index, 1, " ");
				if (!topic.contains("compressedDepth"))
					topics.insert(topic);
				//qDebug("ImageView::getTopics() transport specific sub-topic '%s'", topic.toStdString().c_str());
			}
		}
	}
	return topics;
}

void ImageView::selectTopic(const QString& topic)
{
	int index = ui_.topics_combo_box->findText(topic);
	if (index == -1)
	{
		index = ui_.topics_combo_box->findText("");
	}
	ui_.topics_combo_box->setCurrentIndex(index);
}

void ImageView::onTopicChanged(int index)
{
	subscriber_.shutdown();

	// reset image on topic change
	ui_.image_frame->setImage(QImage());

	QStringList parts = ui_.topics_combo_box->itemData(index).toString().split(
			" ");
	QString topic = parts.first();
	QString transport = parts.length() == 2 ? parts.last() : "raw";

	if (!topic.isEmpty())
	{
		image_transport::ImageTransport it(getNodeHandle());
		image_transport::TransportHints hints(transport.toStdString());
		try
		{
			subscriber_ = it.subscribe(topic.toStdString(), 1,
					&ImageView::callbackImage, this, hints);
			//qDebug("ImageView::onTopicChanged() to topic '%s' with transport '%s'", topic.toStdString().c_str(), subscriber_.getTransport().c_str());
		} catch (image_transport::TransportLoadException& e)
		{
			QMessageBox::warning(widget_,
					tr("Loading image transport plugin failed"), e.what());
		}
	}
}

void ImageView::onZoom1(bool checked)
{
	if (checked)
	{
		if (ui_.image_frame->getImage().isNull())
		{
			return;
		}
		ui_.image_frame->setInnerFrameFixedSize(
				ui_.image_frame->getImage().size());
		widget_->resize(ui_.image_frame->size());
		widget_->setMinimumSize(widget_->sizeHint());
		widget_->setMaximumSize(widget_->sizeHint());
	}
	else
	{
		ui_.image_frame->setInnerFrameMinimumSize(QSize(80, 60));
		ui_.image_frame->setMaximumSize(
				QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
		widget_->setMinimumSize(QSize(80, 60));
		widget_->setMaximumSize(QSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX));
	}
}

void ImageView::onDynamicRange(bool checked)
{
	ui_.max_range_double_spin_box->setEnabled(!checked);
}

void ImageView::saveImage()
{
	// take a snapshot before asking for the filename
	QImage img = ui_.image_frame->getImageCopy();

	QString file_name = QFileDialog::getSaveFileName(widget_,
			tr("Save as image"), "image.png",
			tr("Image (*.bmp *.jpg *.png *.tiff)"));
	if (file_name.isEmpty())
	{
		return;
	}

	img.save(file_name);
}

QString ImageView::prepareForConsole(QString text, int r, int g, int b, int fontSize)
{
	styledString="<span style=\" font-size:"+QString::number(fontSize)+"pt; font-weight:600; color:"
										+ rgbToHex(r, g, b) +" \" > ";
	styledString.append(text);
	styledString.append("</span>");
	styledString.append("<br/>");
	
	return styledString;
}

// Returns string of format "#******"
QString ImageView::rgbToHex(int r, int g, int b)
{
	int color = (r * 65536) + (g * 256) + b;
	
	// Convert to hex
	std::stringstream ss;
	ss << std::hex << color;
	QString color_string = QString::fromStdString(ss.str());
	
	while(color_string.length() < 6) // Add zeroes to the beginning
		color_string.prepend("0");
	
	color_string.prepend("#");
	
	return color_string;
}

void ImageView::appendToConNewLine(QString text,QString color,int fontSize)
{
	styledString="<span style=\" font-size:"+QString::number(fontSize)+"pt; font-weight:600; color:"+color+";\" > ";
//	styledString+="<span style=\" font-size:"+QString::number(fontSize)+"pt; font-weight:600; color:"+color+";\" > ";
	styledString.append(text);
	styledString.append("</span>");
	styledString.append("<br/>");
	emit conSetHtml(styledString);
//	emit conSetHtml(styledString+"<hr> <a name=\"scrollToMe\" href=\"#word\">--------------------------------------------------</a>");
	//emit conApp("");
}

void ImageView::callbackImage(const sensor_msgs::Image::ConstPtr& msg)
{
	try
	{
		// First let cv_bridge do its magic
		cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg,
				sensor_msgs::image_encodings::RGB8);
		conversion_mat_ = cv_ptr->image;
	} catch (cv_bridge::Exception& e)
	{
		try
		{
			// If we're here, there is no conversion that makes sense, but let's try to imagine a few first
			cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
			if (msg->encoding == "CV_8UC3")
			{
				// assuming it is rgb
				conversion_mat_ = cv_ptr->image;
			}
			else if (msg->encoding == "8UC1")
			{
				// convert gray to rgb
				cv::cvtColor(cv_ptr->image, conversion_mat_, CV_GRAY2RGB);
			}
			else if (msg->encoding == "16UC1" || msg->encoding == "32FC1")
			{
				// scale / quantify
				double min = 0;
				double max = ui_.max_range_double_spin_box->value();
				if (msg->encoding == "16UC1")
					max *= 1000;
				if (ui_.dynamic_range_check_box->isChecked())
				{
					// dynamically adjust range based on min/max in image
					cv::minMaxLoc(cv_ptr->image, &min, &max);
					if (min == max)
					{
						// completely homogeneous images are displayed in gray
						min = 0;
						max = 2;
					}
				}
				cv::Mat img_scaled_8u;
				cv::Mat(cv_ptr->image - min).convertTo(img_scaled_8u, CV_8UC1,
						255. / (max - min));
				cv::cvtColor(img_scaled_8u, conversion_mat_, CV_GRAY2RGB);
			}
			else
			{
				qWarning(
						"ImageView.callback_image() could not convert image from '%s' to 'rgb8' (%s)",
						msg->encoding.c_str(), e.what());
				ui_.image_frame->setImage(QImage());
				return;
			}
		} catch (cv_bridge::Exception& e)
		{
			qWarning(
					"ImageView.callback_image() while trying to convert image from '%s' to 'rgb8' an exception was thrown (%s)",
					msg->encoding.c_str(), e.what());
			ui_.image_frame->setImage(QImage());
			return;
		}
	}


	Point2d c = ui_.image_frame->ClickedPoint;

	c.x = (c.x * conversion_mat_.size().width - 1);
	c.y = (c.y * conversion_mat_.size().height - 1);

	appendToConNewLine("["+QString::number(c.x)+", "+QString::number(c.y)+"]");
	cv::circle(conversion_mat_, c, 1, Scalar(0, 0, 255), 1);

	// image must be copied since it uses the conversion_mat_ for storage which is asynchronously overwritten in the next callback invocation
	QImage image(conversion_mat_.data, conversion_mat_.cols,
			conversion_mat_.rows, conversion_mat_.step[0],
			QImage::Format_RGB888);
	ui_.image_frame->setImage(image);



	if (!ui_.zoom_1_push_button->isEnabled())
	{
		ui_.zoom_1_push_button->setEnabled(true);
		onZoom1(ui_.zoom_1_push_button->isChecked());
	}
}



}

PLUGINLIB_EXPORT_CLASS(rqt_vision_module::ImageView, rqt_gui_cpp::Plugin)
