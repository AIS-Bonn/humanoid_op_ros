//subclass for slider
//Author: Sebastian Schüller

#ifndef PARAMETERITEM_H
#define PARAMETERITEM_H

#include <QTreeWidget>
#include <QSpinBox>
#include <QLineEdit>
#include <QCheckBox>

#include <config_server/ParameterValue.h>

namespace remote_tuner
{

class WheelFilter : public QObject
{
Q_OBJECT
public:
	explicit WheelFilter(QObject* parent = 0);

protected:
    virtual bool eventFilter(QObject* object, QEvent* event);
};



class ParameterWidgetBase : public QWidget
{
Q_OBJECT
public:
	ParameterWidgetBase();

	virtual void IncValue() = 0;
	virtual void DecValue() = 0;
	virtual QString value() const = 0;

	void setValue(const config_server::ParameterValue& value);
	void notify();
Q_SIGNALS:
	void setRequested(const QString& name, const QString& value);
protected:
	virtual void update(const config_server::ParameterValue& value) = 0;
private:
	std::string m_name;
};

class IntParameterWidget : public ParameterWidgetBase
{
Q_OBJECT
public:
	explicit IntParameterWidget();
	virtual ~IntParameterWidget();
    virtual void DecValue();
    virtual void IncValue();
	virtual QString value() const override;
	virtual void update(const config_server::ParameterValue& value) override;

Q_SIGNALS:
	void called(int);

private Q_SLOTS:
	void handleSlider();
	void handleSpinbox();
	void handleCallback(int value);

private:
	QSlider* m_slider;
	QSpinBox* m_spinbox;
};


class FloatParameterWidget : public ParameterWidgetBase
{
Q_OBJECT
public:
	FloatParameterWidget();
	virtual ~FloatParameterWidget();
    virtual void DecValue();
    virtual void IncValue();
	virtual QString value() const override;
	virtual void update(const config_server::ParameterValue& value) override;

Q_SIGNALS:
	void called(float);

private Q_SLOTS:
	void handleSlider();
	void handleSpinbox();
	void handleCallback(float value);

private:
	double m_sliderStepRatio;
	QSlider* m_slider;
	QDoubleSpinBox* m_spinbox;
};


class StringParameterWidget : public ParameterWidgetBase
{
Q_OBJECT
public:
	StringParameterWidget();
	virtual ~StringParameterWidget();
	virtual void DecValue();
	virtual void IncValue();
	virtual QString value() const override;
	virtual void update(const config_server::ParameterValue& value) override;

Q_SIGNALS:
	void called(std::string);

private Q_SLOTS:
	void handleLineEdit();
	void handleCallback(std::string text);

private:
	QLineEdit* m_lineEdit;
};

class BoolParameterWidget : public ParameterWidgetBase
{
Q_OBJECT
public:
	BoolParameterWidget();
	virtual ~BoolParameterWidget();
	virtual void DecValue();
	virtual void IncValue();
	virtual QString value() const override;
	virtual void update(const config_server::ParameterValue& value) override;

Q_SIGNALS:
	void called(bool);

private Q_SLOTS:
	void handleCheckbox();
	void handleCallback(bool value);

private:
	QCheckBox* m_checkBox;
};

}

#endif