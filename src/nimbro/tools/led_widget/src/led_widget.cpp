//Small rqt plugin to control walking
//Author: Sebastian Schüller

#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>

#include <QColor>

#include <led_widget/led_widget.h>
#include "ui_led_widget.h"

namespace led_widget
{

LEDWidget::LEDWidget()
{
	view = new QWidget();
	
	ui = new Ui::LEDWidget;
	ui->setupUi(view);
	
	// Create LEDs
	for(int i = 0; i < 5; i++) // Usual leds
	{
		leds.push_back(new LED(30, 60));
		ui->LEDLayout->addWidget(leds.back());
	}
	
	for(int i = 0; i < 2; i++) // RGB leds
	{
		leds.push_back(new LED(60, 60));
		ui->LEDLayout->addWidget(leds.back());
	}
	
	// Set 'on' colors for usual LEDs
	leds.at(0)->setOnColor(QColor(0, 255, 0));    // Green 
	leds.at(1)->setOnColor(QColor(255, 120, 1));  // Orange 
	leds.at(2)->setOnColor(QColor(255, 0, 0));    // Red
	leds.at(3)->setOnColor(QColor(0, 0, 255));    // Blue
	leds.at(4)->setOnColor(QColor(0, 255, 0));    // Green
	
	// Init ros stuff
	subscriber = n.subscribe("/led_state", 1000 ,&LEDWidget::stateReceived, this);
	
	// Set up connections
	connect(ui->button0, SIGNAL(clicked(bool)), this, SLOT(handleButton_0()));
	connect(ui->button1, SIGNAL(clicked(bool)), this, SLOT(handleButton_1()));
	connect(ui->button2, SIGNAL(clicked(bool)), this, SLOT(handleButton_2()));
}

void LEDWidget::stateReceived(const nimbro_op_interface::LEDCommand &state)
{
	// Turn on/off LEDs
	for(int pos = 6; pos >=0; pos--)
	{
		int bit = ((char)state.state & ( 1 << pos )) >> pos;
		leds.at(6-pos)->turn(bit);
	}
	
	// Set color for RGB LEDs
	leds.at(5)->setColor(QColor(state.rgb5.r*255, state.rgb5.g*255, state.rgb5.b*255));
	leds.at(6)->setColor(QColor(state.rgb6.r*255, state.rgb6.g*255, state.rgb6.b*255));
}

void LEDWidget::initPlugin(qt_gui_cpp::PluginContext& context)
{
	context.addWidget(view);
}

void LEDWidget::shutdownPlugin()
{
	rqt_gui_cpp::Plugin::shutdownPlugin();
	
	subscriber.shutdown();
}

void LEDWidget::handleButton_0()
{
	//printf("CLICKED_0\n");
	
	n.setParam("//robotcontrol/nopInterface/button/pressButton0", true);
	
	/*if (n.hasParam("/robotcontrol/nopInterface/button/pressButton0"))
	{
		printf("EXISTS\n");
	}*/
}

void LEDWidget::handleButton_1()
{
	n.setParam("//robotcontrol/nopInterface/button/pressButton1", true);
}
void LEDWidget::handleButton_2()
{
	n.setParam("//robotcontrol/nopInterface/button/pressButton2", true);
}

LEDWidget::~LEDWidget()
{
	//delete view;
}

}

PLUGINLIB_EXPORT_CLASS(led_widget::LEDWidget, rqt_gui_cpp::Plugin)
