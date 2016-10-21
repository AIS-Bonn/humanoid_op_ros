// Wrapper for service calls
// Author: Dmytro Pavlichenko dm.mark999@gmail.com

#ifndef SERVICE_H
#define SERVICE_H

#include <string>
#include <QDoubleSpinBox>
#include <QLineEdit>

namespace control_widget
{

namespace service
{
	
	namespace color
	{
		static const QString failure = QString("Red");
		static const QString success = QString("Green");
		static const QString input = QString("Blue");
		static const QString output = QString("MediumVioletRed");
	};

// Base class
class Service
{ 
public:
	Service(std::string name);
	virtual ~Service();
	
	virtual bool call(QStringList &output);
	
protected:
	template<typename Req, typename Res>
	bool callService(Req &request, Res &response, QStringList &output);
	
	template<typename Req>
	bool callService(Req &request, QStringList &output);
	
	void prependTime(QString &text) const;
	QString separator() const;
	void setHtmlColor(QString &text, QString color) const;
	
protected:
	std::string serviceName;
};


class ResetConfig : public Service
{
public:
	ResetConfig(std::string name);
	bool call(QStringList &output);
};

class SaveConfig : public Service
{
public:
	SaveConfig(std::string name);
	bool call(QStringList &output);
};

class AttEstCalib : public Service
{
public:
	AttEstCalib(std::string name);
	bool call(QStringList &output);
};

class ShowDeadVars : public Service
{
public:
	ShowDeadVars(std::string name, QLineEdit *path_);
	bool call(QStringList &output);
	
private:
	QLineEdit *path;
};

class MagCalib2D : public Service
{
public:
	MagCalib2D(std::string name);
	bool call(QStringList &output);
};

class MagCalib3D : public Service
{
public:
	MagCalib3D(std::string name);
	bool call(QStringList &output);
};

class MagCalibShow : public Service
{
public:
	MagCalibShow(std::string name);
	bool call(QStringList &output);
};

class WarpAddPoint : public Service
{
public:
	WarpAddPoint(std::string name, float value_); // value_ must be in degrees
	bool call(QStringList &output);
	
private:
	float value;
};

class SetOdom : public Service
{
public:
	SetOdom(std::string name, QDoubleSpinBox *x_, QDoubleSpinBox *y_, QDoubleSpinBox *theta_);
	bool call(QStringList &output);

private:
	QDoubleSpinBox *x;
	QDoubleSpinBox *y;
	QDoubleSpinBox *theta;
};

class ReadOffset : public Service
{
public:
	ReadOffset(std::string name, QLineEdit *joint_);
	bool call(QStringList &output);

private:
	QLineEdit *joint;
};

class UseLastServerIP : public Service
{
public:
	UseLastServerIP(std::string name);
	bool call(QStringList &output);
};

class Empty : public Service
{
public:
	Empty(std::string name);
	bool call(QStringList &output);
};

}

}

#endif