// Diagnostics GUI
// Author: Max Schwarz <max.schwarz@uni-bonn.de>

#include <rqt_gui_cpp/plugin.h>

#include <ros/subscriber.h>
#include <actionlib/client/simple_action_client.h>

#include <robotcontrol/Diagnostics.h>
#include <robotcontrol/FadeTorqueAction.h>

#include <QMutex>

#include "ui_diagnostics.h"

namespace diagnostics
{

class Diagnostics : public rqt_gui_cpp::Plugin
{
Q_OBJECT
public:
	Diagnostics();
	virtual ~Diagnostics();

	virtual void initPlugin(qt_gui_cpp::PluginContext& context);

	void handleDiagnostics(const robotcontrol::DiagnosticsPtr& diag);
Q_SIGNALS:
	void updateRequested();
private Q_SLOTS:
	void update();
	void handleFade();
	void handleFadeFeedback(const robotcontrol::FadeTorqueFeedbackConstPtr& fb);
	void handleFadeDone(const actionlib::SimpleClientGoalState&, const robotcontrol::FadeTorqueResultConstPtr& ptr);
	void updateFadeButton();
private:
	Ui::Diagnostics m_ui;

	ros::Subscriber m_sub_diag;
	QMutex m_mutex;
	robotcontrol::DiagnosticsPtr m_diag;
	robotcontrol::FadeTorqueFeedbackConstPtr m_fadeFB;

	boost::shared_ptr<actionlib::SimpleActionClient<robotcontrol::FadeTorqueAction> > m_torqueClient;

	bool m_fadeStatus;
	bool m_fadeDone;
};


}
