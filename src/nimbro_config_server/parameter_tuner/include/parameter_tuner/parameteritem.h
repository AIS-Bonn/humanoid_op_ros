// Parameter item
// Author: Sebastian Schüller <schuell1@cs.uni-bonn.de>
//         Philipp Allgeuer <pallgeuer@ais.uni-bonn.de>

#ifndef PARAMETERITEM_H
#define PARAMETERITEM_H

#include <config_server/parameter.h>
#include <parameter_tuner/historykeeper.h>

#include <QTreeWidget>
#include <QLineEdit>
#include <QCheckBox>
#include <QSpinBox>
#include <QEvent>

namespace parametertuner
{

//
// Wheel event filter
//

class WheelFilter : public QObject
{
Q_OBJECT
public:
	explicit WheelFilter(QObject* parent = NULL) : QObject(parent) {}

protected:
    virtual bool eventFilter(QObject* object, QEvent* event) { return (event->type() == QEvent::Wheel); }
};

//
// Parameter widget base class
//

class ParameterWidgetBase : public QWidget
{
Q_OBJECT
public:
	explicit ParameterWidgetBase(QTreeWidgetItem* treeItem) : m_treeItem(treeItem), m_history(40, 1.0, 3.0) {}

public:
	virtual void incValue() = 0;
	virtual void decValue() = 0;

protected Q_SLOTS:
	virtual void handleUndoRedo(QVariant variant) = 0;

protected:
	QTreeWidgetItem* m_treeItem;
	HistoryKeeper m_history;
};

//
// Float parameter widget
//

class FloatParameterWidget : public ParameterWidgetBase
{
Q_OBJECT
public:
	explicit FloatParameterWidget(QTreeWidgetItem* treeItem, ros::NodeHandle& nh, const config_server::ParameterDescription& description);
	virtual ~FloatParameterWidget() {}

Q_SIGNALS:
	void called(float);

public:
	virtual void decValue();
	virtual void incValue();

protected Q_SLOTS:
	virtual void handleUndoRedo(QVariant variant);

private Q_SLOTS:
	void handleSlider();
	void handleSpinbox();
	void handleCallback(float value);

private:
	config_server::ParameterDescription m_desc;
	config_server::Parameter<float> m_parameter;

	double sliderTicksToValue(int ticks);
	int sliderValueToTicks(double value);

	QSlider* m_slider;
	QDoubleSpinBox* m_spinbox;
};

//
// Int parameter widget
//

class IntParameterWidget : public ParameterWidgetBase
{
Q_OBJECT
public:
	explicit IntParameterWidget(QTreeWidgetItem* treeItem, ros::NodeHandle& nh, const config_server::ParameterDescription& description);
	virtual ~IntParameterWidget() {}

Q_SIGNALS:
	void called(int);

public:
	virtual void decValue();
	virtual void incValue();

protected Q_SLOTS:
	virtual void handleUndoRedo(QVariant variant);

private Q_SLOTS:
	void handleSlider();
	void handleSpinbox();
	void handleCallback(int value);

private:
	config_server::Parameter<int> m_parameter;

	QSlider* m_slider;
	QSpinBox* m_spinbox;
};

//
// String parameter widget
//

class StringParameterWidget : public ParameterWidgetBase
{
Q_OBJECT
public:
	explicit StringParameterWidget(QTreeWidgetItem* treeItem, ros::NodeHandle& nh, const config_server::ParameterDescription& description);
	virtual ~StringParameterWidget() {}

Q_SIGNALS:
	void called(std::string);

public:
	virtual void decValue() {}
	virtual void incValue() {}

protected Q_SLOTS:
	virtual void handleUndoRedo(QVariant variant);

private Q_SLOTS:
	void handleLineEdit();
	void handleCallback(std::string text);

private:
	config_server::Parameter<std::string> m_parameter;

	QLineEdit* m_lineEdit;
};

//
// Bool parameter widget
//

class BoolParameterWidget : public ParameterWidgetBase
{
Q_OBJECT
public:
	explicit BoolParameterWidget(QTreeWidgetItem* treeItem, ros::NodeHandle& nh, const config_server::ParameterDescription& description);
	virtual ~BoolParameterWidget() {}

Q_SIGNALS:
	void called(bool);

public:
	virtual void decValue();
	virtual void incValue();

protected Q_SLOTS:
	virtual void handleUndoRedo(QVariant variant);

private Q_SLOTS:
	void handleCheckbox();
	void handleCallback(bool value);

private:
	config_server::Parameter<bool> m_parameter;

	QCheckBox* m_checkBox;
};

}

#endif