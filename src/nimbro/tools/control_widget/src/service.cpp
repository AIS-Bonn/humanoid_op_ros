// Wrapper for service calls
// Author: Dmytro Pavlichenko dm.mark999@gmail.com

#include <control_widget/service.h>

#include <ros/service.h>
#include <ros/console.h>

#include <config_server/Save.h>
#include <config_server/Load.h>
#include <config_server/ShowDeadVars.h>
#include <config_server/SetParameterRequest.h>
#include <config_server/SetParameterResponse.h>

#include <nimbro_op_interface/AttEstCalib.h>
#include <nimbro_op_interface/ReadOffset.h>

#include <robotcontrol/WarpAddPoint.h>
#include <robotcontrol/MagCalib2D.h>
#include <robotcontrol/MagCalib3D.h>
#include <robotcontrol/MagCalibShow.h>

#include <std_srvs/Empty.h>
#include <gait_msgs/SetOdom.h>

#include <QTime>
#include <stdio.h>

namespace control_widget
{

namespace service
{
	
// BASE
	
Service::Service(std::string name)
{
	serviceName = name;
}

Service::~Service()
{

}

bool Service::call(QStringList &output)
{
	printf("Base service. Should not be called\n");
	return false;
}

template<typename Req, typename Res>
bool Service::callService(Req &request, Res &response, QStringList &output)
{
	QString feedback("");
	prependTime(feedback);
	
	if (ros::service::call(serviceName, request, response) == false)
	{
		feedback.append(QString("<c>FAILED</c> to call "));
		feedback.append(QString::fromStdString(serviceName));
		setHtmlColor(feedback, color::failure);
		
		output.append(feedback);
		return false;
	}
	
	feedback.append(QString("<c>Successfully</c> called "));
	feedback.append(QString::fromStdString(serviceName));
	setHtmlColor(feedback, color::success);
	
	output.append(feedback);
	return true;
}

template<typename Req>
bool Service::callService(Req &request, QStringList &output)
{
	QString feedback("");
	prependTime(feedback);
	
	if (ros::service::call(serviceName, request) == false)
	{
		feedback.append(QString("<c>FAILED</c> to call "));
		feedback.append(QString::fromStdString(serviceName));
		setHtmlColor(feedback, color::failure);
		
		output.append(feedback);	
		return false;
	}
	
	feedback.append(QString("<c>Successfully</c> called "));
	feedback.append(QString::fromStdString(serviceName));
	setHtmlColor(feedback, color::success);
	
	output.append(feedback);
	return true;
}

void Service::prependTime(QString& text) const
{
	QString suffix = QString("[%1] ").arg(QTime::currentTime().toString());
	text.prepend(suffix);
}

QString Service::separator() const
{
	QString sep;
	prependTime(sep);
	return sep;
}

void Service::setHtmlColor(QString &text, QString color) const
{
	QString opening = QString("<font color=\"%1\">").arg(color);
	QString closing("</font>");
	
	int openingPos = 1;
	int closingPos = 1;
	
	while(openingPos!= -1 || closingPos != -1)
	{
		openingPos = text.indexOf(QString("<c>"));
		if(openingPos != -1)
			text.replace(openingPos, 3, opening);
		
		closingPos = text.indexOf(QString("</c>"));
		if(closingPos != -1)
			text.replace(closingPos, 4, closing);
	}
}

// RESET CONFIG

ResetConfig::ResetConfig(std::string name) : Service(name)
{
	
}

bool ResetConfig::call(QStringList &output)
{
	config_server::Load request;
	request.request.filename = "";

	return callService(request, output);
}

// SAVE CONFIG

SaveConfig::SaveConfig(std::string name) : Service(name)
{
 
}

bool SaveConfig::call(QStringList &output)
{
	config_server::Load request;

	return callService(request, output);
}

// AttEstCalibrate

AttEstCalib::AttEstCalib(std::string name)
	: Service(name)
{

}

bool AttEstCalib::call(QStringList &output)
{
	nimbro_op_interface::AttEstCalibRequest  request;
	nimbro_op_interface::AttEstCalibResponse response;
	
	bool result = callService(request, response, output);
	
	QString feedback = QString("<c>Returned</c> magCalib = <c>(%1, %2, %3)</c>, gyroBias = <c>(%4, %5, %6)</c>")
		.arg(QString::number(response.magCalibX, 'f', 3))
		.arg(QString::number(response.magCalibY, 'f', 3))
		.arg(QString::number(response.magCalibZ, 'f', 3))
		.arg(QString::number(response.gyroBiasX, 'f', 4))
		.arg(QString::number(response.gyroBiasY, 'f', 4))
		.arg(QString::number(response.gyroBiasZ, 'f', 4));
	
	prependTime(feedback);
	setHtmlColor(feedback, color::output);
	
	output.append(feedback);
	output.append(separator());
	
	return result;
}

// ShowDeadVars

ShowDeadVars::ShowDeadVars(std::string name, QLineEdit *path_): Service(name)
{
	path = path_;
}

bool ShowDeadVars::call(QStringList &output)
{
	config_server::ShowDeadVarsRequest  request;
	config_server::ShowDeadVarsResponse response;
	
	request.configPath = path->text().toStdString();
	
	bool result = callService(request, response, output);
	
	if(output.size() > 0)
		output.last().append(QString(" <font color=\"Blue\">with path '%1'</font>").arg(path->text()));
	
	return result;
}

// SetOdom

SetOdom::SetOdom(std::string name, QDoubleSpinBox *x_, QDoubleSpinBox *y_, QDoubleSpinBox *theta_)
	: Service(name)
{
	x = x_;
	y = y_;
	theta = theta_;
}

bool SetOdom::call(QStringList &output)
{
	gait_msgs::SetOdomRequest  request;
	gait_msgs::SetOdomResponse response;
	
	request.posX = x->value();
	request.posY = y->value();
	request.rotZ = theta->value();

	bool result = callService(request, response, output);
	
	if(output.size() > 0)
	{
		output.last().append(QString(" <c>with (x, y, &theta;) = (%1, %2, %3)</c>")
		.arg(request.posX).arg(request.posY).arg(request.rotZ));
		
		setHtmlColor(output.last(), color::input);
	}
	
	return result;
}

// WarpAddPoint

WarpAddPoint::WarpAddPoint(std::string name, float value_): Service(name)
{
	value = value_;
}

bool WarpAddPoint::call(QStringList &output)
{
	robotcontrol::WarpAddPointRequest  request;
	robotcontrol::WarpAddPointResponse response;
	
	request.trueHeading = value;

	bool result = callService(request, response, output);
	
	if(output.size() > 0)
	{
		output.last().append(QString(" <c>with %1&deg;</c>").arg(value));
		setHtmlColor(output.last(), color::input);
	}
	
	QString magHeading = QString("<c>Returned</c> magHeading = <c>%1</c>, trueHeading = <c>%2</c>")
		.arg(QString::number(response.magHeading, 'f', 2))
		.arg(QString::number(response.trueHeading, 'f', 2));
		
	prependTime(magHeading);
	setHtmlColor(magHeading, color::output);
	output.append(magHeading);
	
	QString newString = QString("<c>Returned</c> new warp string = <c>'%1'</c>")
		.arg(QString::fromStdString(response.newParamString));
		
	prependTime(newString);
	setHtmlColor(newString, color::output);
	output.append(newString);
	output.append(separator());
	
	return result;
}

// MagCalib2D and 3D and Show

MagCalib2D::MagCalib2D(std::string name): Service(name)
{
}

bool MagCalib2D::call(QStringList &output)
{
	robotcontrol::MagCalib2DRequest  request;
	robotcontrol::MagCalib2DResponse response;

	bool result = callService(request, response, output);
	
	QString hardOffset = QString("<c>Returned</c> hardOffset = <c>(%1, %2, %3)</c>, 2D offset Z = <c>%4</c>")
		.arg(QString::number(response.hardOffsetX, 'f', 4))
		.arg(QString::number(response.hardOffsetY, 'f', 4))
		.arg(QString::number(response.hardOffsetZ, 'f', 4))
		.arg(QString::number(response.hardOffset2DZ, 'f', 4));
		
	prependTime(hardOffset);
	setHtmlColor(hardOffset, color::output);
	output.append(hardOffset);
	
	QString radius2D = QString("<c>Returned</c> 2D hard radius = <c>%1</c>, 2D error = <c>%2</c>")
		.arg(QString::number(response.hardRadius, 'f', 4))
		.arg(QString::number(response.hardErr2D, 'f', 4));
		
	prependTime(radius2D);
	setHtmlColor(radius2D, color::output);
	output.append(radius2D);
	
	QString radius3D = QString("<c>Returned</c> 3D hard radius = <c>%1</c>, 3D error = <c>%2</c>")
		.arg(QString::number(response.hardField, 'f', 4))
		.arg(QString::number(response.hardErr3D, 'f', 4));
		
	prependTime(radius3D);
	setHtmlColor(radius3D, color::output);
	output.append(radius3D);
	output.append(separator());
	
	return result;
}

MagCalib3D::MagCalib3D(std::string name): Service(name)
{
}

bool MagCalib3D::call(QStringList &output)
{
	robotcontrol::MagCalib3DRequest  request;
	robotcontrol::MagCalib3DResponse response;

	bool result = callService(request, response, output);
	
	QString hardOffset = QString("<c>Returned</c> hardOffset = <c>(%1, %2, %3)</c>")
		.arg(QString::number(response.hardOffsetX, 'f', 4))
		.arg(QString::number(response.hardOffsetY, 'f', 4))
		.arg(QString::number(response.hardOffsetZ, 'f', 4));
		
	prependTime(hardOffset);
	setHtmlColor(hardOffset, color::output);
	output.append(hardOffset);
	
	QString softOffset = QString("<c>Returned</c> softOffset = <c>(%1, %2, %3)</c>")
		.arg(QString::number(response.softOffsetX, 'f', 4))
		.arg(QString::number(response.softOffsetY, 'f', 4))
		.arg(QString::number(response.softOffsetZ, 'f', 4));
		
	prependTime(softOffset);
	setHtmlColor(softOffset, color::output);
	output.append(softOffset);
	
	QString radius2D = QString("<c>Returned</c> 3D hard radius = <c>%1</c>, 3D hard error = <c>%2</c>")
		.arg(QString::number(response.hardField, 'f', 4))
		.arg(QString::number(response.hardError, 'f', 4));
		
	prependTime(radius2D);
	setHtmlColor(radius2D, color::output);
	output.append(radius2D);
	
	QString radius3D = QString("<c>Returned</c> 3D soft radius = <c>%1</c>, 3D soft error = <c>%2</c>")
		.arg(QString::number(response.softField, 'f', 4))
		.arg(QString::number(response.softError, 'f', 4));
		
	prependTime(radius3D);
	setHtmlColor(radius3D, color::output);
	output.append(radius3D);
	
	output.append(separator());
	
	return result;
}

MagCalibShow::MagCalibShow(std::string name): Service(name)
{
}

bool MagCalibShow::call(QStringList &output)
{
	robotcontrol::MagCalibShowRequest  request;
	robotcontrol::MagCalibShowResponse response;

	bool result = callService(request, response, output);
	
	QString hardOffset = QString("<c>Returned</c> hardOffset = <c>(%1, %2, %3)</c>, 2D offset Z = <c>%4</c>")
		.arg(QString::number(response.hardOffsetX, 'f', 4))
		.arg(QString::number(response.hardOffsetY, 'f', 4))
		.arg(QString::number(response.hardOffsetZ, 'f', 4))
		.arg(QString::number(response.hardOffset2DZ, 'f', 4));
		
	prependTime(hardOffset);
	setHtmlColor(hardOffset, color::output);
	output.append(hardOffset);
	
	QString hardRadius = QString("<c>Returned</c> 3D hard radius = <c>%1</c>, 2D hard radius = <c>%2</c>")
		.arg(QString::number(response.hardField, 'f', 4))
		.arg(QString::number(response.hardRadius, 'f', 4));
		
	prependTime(hardRadius);
	setHtmlColor(hardRadius, color::output);
	output.append(hardRadius);
	
	QString softOffset = QString("<c>Returned</c> softOffset = <c>(%1, %2, %3)</c>, 3D soft radius = <c>%4</c>")
		.arg(QString::number(response.softOffsetX, 'f', 4))
		.arg(QString::number(response.softOffsetY, 'f', 4))
		.arg(QString::number(response.softOffsetZ, 'f', 4))
		.arg(QString::number(response.softField, 'f', 4));
		
	prependTime(softOffset);
	setHtmlColor(softOffset, color::output);
	output.append(softOffset);
	
	output.append(separator());
	
	return result;
}

// ReadOffset

ReadOffset::ReadOffset(std::string name, QLineEdit* joint_): Service(name)
{
	joint = joint_;
}

bool ReadOffset::call(QStringList &output)
{
	nimbro_op_interface::ReadOffsetRequest  request;
	nimbro_op_interface::ReadOffsetResponse response;
	
	request.joint = joint->text().toStdString();

	bool result = callService(request, response, output);
	
	if(output.size() > 0)
	{
		output.last().append(QString("<c> with joint '%1'</c>").arg(joint->text()));
		setHtmlColor(output.last(), color::input);
	}
	
	QString feedback = QString("<c>Returned</c> %1 = <c>%2</c> ticks")
		.arg(joint->text())
		.arg(response.ticks);
		
	prependTime(feedback);
	setHtmlColor(feedback, color::output);
	output.append(feedback);
	output.append(separator());
	
	return result;
}

// UseLastServerIP

UseLastServerIP::UseLastServerIP(std::string name): Service(name)
{

}

bool UseLastServerIP::call(QStringList& output)
{
	config_server::SetParameterRequest  request;
	config_server::SetParameterResponse response;
	
	request.name      = "/game_controller/useLastServerIP";
	request.value     = "1";
	request.no_notify = "0";
	
	return false;
// 	return callService(request, response, output); // FIXME: doesnt work
}


// EMPTY

Empty::Empty(std::string name): Service(name)
{
	
}

bool Empty::call(QStringList &output)
{
	std_srvs::Empty empty;
	return callService(empty, output);
}
	
}

}