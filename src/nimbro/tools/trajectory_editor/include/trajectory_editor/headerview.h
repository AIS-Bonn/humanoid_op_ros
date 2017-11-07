#ifndef HEADERVIEW_H
#define HEADERVIEW_H

// Widget to edit motionName, preState, playState, postState
// If some variable is being set to invalid value, corresponding QLineEdit becomes red
// If value is valid, QLineEdit is white
// Author: Dmytro Pavlichenko <dm.mark999@gmail.com>

#include <QWidget>
#include <QObject>

#include <config_server/parameter.h>
#include <head_control/HeadControlStatus.h>

struct HeaderData
{
	std::string motion_name;
	
	std::string pre_state;
	std::string play_state;
	std::string post_state;
	
	bool pid_enabled;
};

namespace Ui
{
	class HeaderView; 
}

class HeaderView : public QWidget
{
Q_OBJECT
public:
	HeaderView(QWidget *parent = 0);
	virtual ~HeaderView();
	
	void enableEdit(bool flag); // set all edits editable/not editable
	void setValues(std::string const name, std::string const preState
		, std::string const playState, std::string const postState, bool pidEnabled);
	
	// If values are valid - return true. If not - show warning and ask if user still would like to save it
	// If user click 'yes' - return true. Otherwise - false
	bool requestSave();
	
	void setFileName(QString name);
	std::string getFileNameFromPath(QString path); // Parse name from full path to file
	
	std::string getMotionName();
	std::string getPreState();
	std::string getPlayState();
	std::string getPostState();
	std::string getNameForMirrored();
	
	double getFactor();
	bool   getPIDEnabled();
	
	QString getFileName();
	QString getWarningString(); // String includes warning for each invalid value. Empty if everything is ok
	
Q_SIGNALS:
	void dataChanged(HeaderData); // Emmited when data is changed
	void pidEnabledChanged(bool);
	
public Q_SLOTS:
	void setData(HeaderData data);
	
private Q_SLOTS:
	void checkValues();
	void handleFieldChanged();
	void switchHeadControl();
	void switchFallProtection();
	
private:
	void updateHeadControlStatus(const head_control::HeadControlStatus &status);
	void updateFallProtectionStatus();
	
private:
	Ui::HeaderView *ui;

	QString m_file_name; // Name of file without ".yaml"
	QString m_warning_string;
	
	ros::NodeHandle m_nh;
	ros::Subscriber m_head_control_subscriber;
	ros::Publisher  m_head_control_publisher;
	config_server::Parameter<bool>  m_fall_protection_param;
};


#endif // HEADERVIEW_H
