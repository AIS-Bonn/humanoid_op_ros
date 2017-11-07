// Small rqt plugin to show the robot buttons and LEDs
// Author: Dmytro Pavlichenko

#include "ui_led_widget.h"
#include <ros/node_handle.h>
#include <pluginlib/class_list_macros.h>
#include <led_widget/led_widget.h>
#include <QColor>

using namespace nimbro_op_interface;

namespace led_widget
{

LEDWidget::LEDWidget()
 : n("~")
 , m_pressButton0("/nimbro_op_interface/button/pressButton0", false)
 , m_pressButton1("/nimbro_op_interface/button/pressButton1", false)
 , m_pressButton2("/nimbro_op_interface/button/pressButton2", false)
{
	view = new QWidget();
	
	ui = new Ui::LEDWidget;
	ui->setupUi(view);
	
	// Create LEDs
	for(int i = 0; i < 5; i++) // Usual leds
	{
		LED* led = new LED(30, 60);
		leds.push_back(led);
		ui->LEDLayout->addWidget(led);
	}
	
	for(int i = 0; i < 2; i++) // RGB leds
	{
		LED* led = new LED(60, 60);
		leds.push_back(led);
		ui->LEDLayout->addWidget(led);
	}
	
	// Set 'on' colors for LEDs
	leds.at(0)->setOnColor(QColor(0, 255, 0));    // Green 
	leds.at(1)->setOnColor(QColor(255, 120, 1));  // Orange 
	leds.at(2)->setOnColor(QColor(255, 0, 0));    // Red
	leds.at(3)->setOnColor(QColor(0, 0, 255));    // Blue
	leds.at(4)->setOnColor(QColor(0, 255, 0));    // Green
	
	// Set initial colors for RGB LEDs
	leds.at(5)->setOnColor(QColor(0, 0, 0));      // Black
	leds.at(6)->setOnColor(QColor(0, 255, 0));    // Green
	
	// Set initial LED states
	for(unsigned int i = 0; i < leds.size(); i++)
		leds.at(i)->turn(false);
	leds.at(5)->turn(true);
	leds.at(6)->turn(true);
	
	// Subscribe to LED state ROS topic
	subscriber = n.subscribe("/led_state", 1, &LEDWidget::stateReceived, this);
	
	// Configure the blink timer
	m_blinkState = true;
	m_blinkTimer.setSingleShot(true);
	connect(&m_blinkTimer, SIGNAL(timeout()), this, SLOT(handleBlinkTimerTimeout()));
	updateBlinking();
	
	// Set up connections
	connect(ui->button0, SIGNAL(clicked(bool)), this, SLOT(handleButton_0()));
	connect(ui->button1, SIGNAL(clicked(bool)), this, SLOT(handleButton_1()));
	connect(ui->button2, SIGNAL(clicked(bool)), this, SLOT(handleButton_2()));
}

void LEDWidget::stateReceived(const nimbro_op_interface::LEDCommandConstPtr& state)
{
	// Process the received state (Note: The LED numbers cannot be assumed to be *sequential* powers of two)
	if(state->mask & LEDCommand::LED0) updateLED(0, state->state & LEDCommand::LED0);
	if(state->mask & LEDCommand::LED1) updateLED(1, state->state & LEDCommand::LED1);
	if(state->mask & LEDCommand::LED2) updateLED(2, state->state & LEDCommand::LED2);
	if(state->mask & LEDCommand::LED3) updateLED(3, state->state & LEDCommand::LED3);
	if(state->mask & LEDCommand::LED4) updateLED(4, state->state & LEDCommand::LED4);
	if(state->mask & LEDCommand::LED5)
	{
		LED* led5 = leds.at(5);
		led5->setOnColor(QColor(state->rgb5.r*255.0, state->rgb5.g*255.0, state->rgb5.b*255.0));
		bool shouldBlink = (state->rgb5Blink != 0);
		if(led5->isBlinking() != shouldBlink)
		{
			led5->setBlinking(shouldBlink);
			updateBlinking();
		}
		if(!led5->isBlinking() && led5->turn(true))
			led5->update();
	}
}

void LEDWidget::updateLED(int led, bool state)
{
	LED* pLED = leds.at(led);
	if(pLED->turn(state))
		pLED->update();
}

void LEDWidget::updateBlinking()
{
	bool needTimer = false;
	for(unsigned int i = 0; i < leds.size(); i++)
	{
		if(leds.at(i)->isBlinking())
		{
			needTimer = true;
			break;
		}
	}

	if(needTimer && !m_blinkTimer.isActive())
	{
		m_blinkState = true;
		handleBlinkTimerTimeout();
		m_blinkTimer.start();
	}
	else if(!needTimer && m_blinkTimer.isActive())
		m_blinkTimer.stop();
}

void LEDWidget::initPlugin(qt_gui_cpp::PluginContext& context)
{
	context.addWidget(view);
}

void LEDWidget::shutdownPlugin()
{
	m_blinkTimer.stop();
	subscriber.shutdown();
}

void LEDWidget::handleBlinkTimerTimeout()
{
	m_blinkState = !m_blinkState;
	for(unsigned int i = 0; i < leds.size(); i++)
	{
		LED* led = leds.at(i);
		if(led->isBlinking())
		{
			if(led->turn(m_blinkState))
				led->update();
		}
	}
	m_blinkTimer.start(m_blinkState ? 100 : 75);
}

void LEDWidget::handleButton_0()
{
	m_pressButton0.set(true);
}

void LEDWidget::handleButton_1()
{
	m_pressButton1.set(true);
}

void LEDWidget::handleButton_2()
{
	m_pressButton2.set(true);
}

LEDWidget::~LEDWidget()
{
	// Note: Deleting the ui automatically deletes the view and the LEDs in the leds vector (these were passed to ui in the constructor), so DO NOT try to delete them explicitly here or you will get a segfault...
	m_blinkTimer.stop();
	delete ui;
}

}

PLUGINLIB_EXPORT_CLASS(led_widget::LEDWidget, rqt_gui_cpp::Plugin)
