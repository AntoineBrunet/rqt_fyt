#include "rqt_fyt/fs_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include "fyt_mae/fyt_commons.h"
#include "fyt_mae/CheckState.h"

namespace rqt_fyt
{

	FSPlugin::FSPlugin()
		: rqt_gui_cpp::Plugin()
		  , widget_(0)
	{
		// Constructor is called first before initPlugin function, needless to say.

		// give QObjects reasonable names
		setObjectName("FSPlugin");
	}

	void FSPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
	{
		// access standalone command line arguments
		QStringList argv = context.argv();
		// create QWidget
		widget_ = new QWidget();
		// extend the widget with all attributes and children from UI file
		ui_.setupUi(widget_);
		// add widget to the user interface
		context.addWidget(widget_);

		sub_state = node.subscribe("/mae/state", 1, &FSPlugin::state_callback, this);
		sub_agcfg = node.subscribe("/parabola/agconfig", 1, &FSPlugin::agcfg_callback, this);
		sub_guidg = node.subscribe("/parabola/guidage", 5, &FSPlugin::guidg_callback, this);
		sub_fwcmd = node.subscribe("/fw/cmd", 5, &FSPlugin::fwcmd_callback, this);
		sub_gimst = node.subscribe("/gimbal/state", 1, &FSPlugin::gimst_callback, this);
		pub_sig = node.advertise<cmg_msgs::Signal>("/mae/signal",1);
	
		tpara = 12;
		parabola_timer = new QTimer(this);
		parabola_timer->setInterval(1000);
		QObject::connect(parabola_timer, SIGNAL(timeout()), this, SLOT(incrProgress()));
		QObject::connect( this, SIGNAL(stopTimer()),
				parabola_timer, SLOT(stop()) );
		QObject::connect( this, SIGNAL(startTimer()),
				parabola_timer, SLOT(start()) );
		QObject::connect( this, SIGNAL(setProgressValue(int)),
				ui_.progressBar, SLOT(setValue(int)) );
		ui_.progressBar->setMaximum(100);
		emit setProgressValue(0);
		

		QObject::connect(this, SIGNAL(setStateText(const QString)),
				ui_.state, SLOT(setText(const QString))	);
		QObject::connect( this, SIGNAL(setStateStyle(const QString)),
				ui_.state, SLOT(setStyleSheet(const QString)) );
		QObject::connect( this, SIGNAL(setAG1Style(const QString)),
				ui_.ag1, SLOT(setStyleSheet(const QString)) );
		QObject::connect( this, SIGNAL(setAG2Style(const QString)),
				ui_.ag2, SLOT(setStyleSheet(const QString)) );
		QObject::connect( this, SIGNAL(setAG3Style(const QString)),
				ui_.ag3, SLOT(setStyleSheet(const QString)) );
		QObject::connect( this, SIGNAL(setAG4Style(const QString)),
				ui_.ag4, SLOT(setStyleSheet(const QString)) );
		QObject::connect( this, SIGNAL(setAG5Style(const QString)),
				ui_.ag5, SLOT(setStyleSheet(const QString)) );
		QObject::connect( this, SIGNAL(setAG6Style(const QString)),
				ui_.ag6, SLOT(setStyleSheet(const QString)) );

		QObject::connect( this, SIGNAL(setGimbalAngle1(int)),
				ui_.pos1, SLOT(setValue(int)) );
		QObject::connect( this, SIGNAL(setGimbalAngle2(int)),
				ui_.pos2, SLOT(setValue(int)) );
		QObject::connect( this, SIGNAL(setGimbalAngle3(int)),
				ui_.pos3, SLOT(setValue(int)) );
		QObject::connect( this, SIGNAL(setGimbalAngle4(int)),
				ui_.pos4, SLOT(setValue(int)) );
		QObject::connect( this, SIGNAL(setGimbalAngle5(int)),
				ui_.pos5, SLOT(setValue(int)) );
		QObject::connect( this, SIGNAL(setGimbalAngle6(int)),
				ui_.pos6, SLOT(setValue(int)) );

		QObject::connect( this, SIGNAL(setStartDisabled(bool)),
				ui_.startButton, SLOT(setDisabled(bool)) );
		QObject::connect( this, SIGNAL(setEndDisabled(bool)),
				ui_.endButton, SLOT(setDisabled(bool)) );
		QObject::connect( this, SIGNAL(setGoodDisabled(bool)),
				ui_.goodButton, SLOT(setDisabled(bool)) );
		QObject::connect(ui_.alarmButton, SIGNAL(clicked(bool)), this, SLOT(triggerAlarm(bool)));
		QObject::connect(ui_.startButton, SIGNAL(clicked(bool)), this, SLOT(triggerStart(bool)));
		QObject::connect(ui_.endButton, SIGNAL(clicked(bool)), this, SLOT(triggerEnd(bool)));
		QObject::connect(ui_.goodButton, SIGNAL(clicked(bool)), this, SLOT(triggerGood(bool)));

		QObject::connect( this, SIGNAL(logManeuver(const QString)),
				ui_.pbDesc, SLOT(append(const QString)) );
		QObject::connect( this, SIGNAL(logStatus(const QString)),
				ui_.status, SLOT(append(const QString)) );



		ros::ServiceClient checkstate_client = node.serviceClient<fyt_mae::CheckState>("/mae/check_state");
		fyt_mae::CheckState state_msg;
		if (checkstate_client.call(state_msg)) {
			cmg_msgs::State s;
			s.state = state_msg.response.state;
			state_callback(s);
		} else {
			ROS_WARN("Could not get current state info...");
		}

	}

	void FSPlugin::shutdownPlugin()
	{
		// unregister all publishers here
	}

	void FSPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
			qt_gui_cpp::Settings& instance_settings) const
	{
		// instance_settings.setValue(k, v)
	}

	void FSPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
			const qt_gui_cpp::Settings& instance_settings)
	{
		// v = instance_settings.value(k)
	}

	void FSPlugin::state_callback(const cmg_msgs::State & msg) {
		switch (msg.state) {
			case STATE_SAFE:
				emit setStateText("SAFE");
				emit setStateStyle("background-color: red;");
				set_agstates(std::vector<bool>({false,false,false,false,false,false}));
				emit stopTimer();
				setGoodDisabled(false);
				setStartDisabled(true);
				setEndDisabled(true);
				break;
			case STATE_READY:
				emit setStateText("READY");
				emit setStateStyle("background-color: green;");
				setGoodDisabled(true);
				setEndDisabled(true);
				break;
			case STATE_MISS:
				emit setStateText("RUNNING");
				emit setStateStyle("background-color: blue; color: white;");
				emit startTimer();
				setGoodDisabled(true);
				setStartDisabled(true);
				setEndDisabled(false);
				break;
			case STATE_POST:
				emit setProgressValue(100);
				emit setStateText("DONE");
				emit setStateStyle("background-color: pink;");
				setEndDisabled(true);
				setStartDisabled(true);
				setGoodDisabled(false);
				emit stopTimer();
				break;
		}
	}

	template <typename T> void FSPlugin::set_agstates(const T& states) {
		QString running_style("background-color:green; color:white;");
		QString halted_style("background-color:red; color:white;");

		emit setAG1Style(states[0]?running_style:halted_style);
		emit setAG2Style(states[1]?running_style:halted_style);
		emit setAG3Style(states[2]?running_style:halted_style);
		emit setAG4Style(states[3]?running_style:halted_style);
		emit setAG5Style(states[4]?running_style:halted_style);
		emit setAG6Style(states[5]?running_style:halted_style);
	}

	void FSPlugin::agcfg_callback(const cmg_msgs::AGConfig::ConstPtr & msg) {
		tpara = msg->tpara;
		emit setProgressValue(0);
		set_agstates(msg->running);
		setStartDisabled(false);
	}

	void FSPlugin::guidg_callback(const cmg_msgs::Guidage::ConstPtr & msg){
		static const QString tnames[3] = {"Normal", "Tracking", "STOP"};
		QString info = QString("New %1 segment maneuver. A=[%2,%3,%4] B=[%5,%6,%7] KP=[%8,%9,%10] KV=[%11,%12,%13]")
			.arg(tnames[msg->type])
			.arg(msg->a[0])
			.arg(msg->a[1])
			.arg(msg->a[2])
			.arg(msg->b[0])
			.arg(msg->b[1])
			.arg(msg->b[2])
			.arg(msg->kp[0])
			.arg(msg->kp[1])
			.arg(msg->kp[2])
			.arg(msg->kv[0])
			.arg(msg->kv[1])
			.arg(msg->kv[2])
			;
		emit logManeuver(info);
	}
	void FSPlugin::fwcmd_callback(const cmg_msgs::SpeedList::ConstPtr & msg){
		QString info = QString("New requested flywheel speeds: [%1,%2,%3,%4,%5,%6]")
			.arg(msg->speeds[0].speed)
			.arg(msg->speeds[1].speed)
			.arg(msg->speeds[2].speed)
			.arg(msg->speeds[3].speed)
			.arg(msg->speeds[4].speed)
			.arg(msg->speeds[5].speed);
		emit logStatus(info);
	}
	void FSPlugin::gimst_callback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr &msg){
		emit setGimbalAngle1(msg->dynamixel_state[0].present_position);
		emit setGimbalAngle2(msg->dynamixel_state[1].present_position);
		emit setGimbalAngle3(msg->dynamixel_state[2].present_position);
		emit setGimbalAngle4(msg->dynamixel_state[3].present_position);
		emit setGimbalAngle5(msg->dynamixel_state[4].present_position);
		emit setGimbalAngle6(msg->dynamixel_state[5].present_position);
	}

	void FSPlugin::triggerAlarm(bool checked) {
		cmg_msgs::Signal s;
		s.signal = SIG_ALARM;
		pub_sig.publish(s);
	}
	void FSPlugin::triggerStart(bool checked) {
		cmg_msgs::Signal s;
		s.signal = SIG_START;
		pub_sig.publish(s);
	}
	void FSPlugin::triggerEnd(bool checked) {
		cmg_msgs::Signal s;
		s.signal = SIG_END;
		pub_sig.publish(s);
	}
	void FSPlugin::triggerGood(bool checked) {
		cmg_msgs::Signal s;
		s.signal = SIG_GOOD;
		pub_sig.publish(s);
	}

	void FSPlugin::incrProgress() {
		int value = ui_.progressBar->value() + 100/tpara;
		if (value > 100) { 
			value = 100; 
			emit setProgressValue(value);
			parabola_timer->stop();
		}
		emit setProgressValue(value);
	}
	/*bool hasConfiguration() const
	  {
	  return true;
	  }
	  void triggerConfiguration()
	  {
	// Usually used to open a dialog to offer the user a set of configuration
	}*/

}  // namespace rqt_fyt
PLUGINLIB_DECLARE_CLASS(rqt_fyt, FSPlugin, rqt_fyt::FSPlugin, rqt_gui_cpp::Plugin)
