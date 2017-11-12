#include "rqt_fyt/fs_plugin.h"
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include "fyt_mae/fyt_commons.h"

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
		pub_sig = node.advertise<cmg_msgs::Signal>("/mae/signal",1);

		QObject::connect(this, SIGNAL(setStateText(const QString)),
				ui_.state, SLOT(setText(const QString))	);
		QObject::connect( this, SIGNAL(setStateStyle(const QString)),
				ui_.state, SLOT(setStyleSheet(const QString)) );
		QObject::connect(ui_.alarmButton, SIGNAL(clicked(bool)), this, SLOT(triggerAlarm(bool)));
		QObject::connect(ui_.startButton, SIGNAL(clicked(bool)), this, SLOT(triggerStart(bool)));
		QObject::connect(ui_.endButton, SIGNAL(clicked(bool)), this, SLOT(triggerEnd(bool)));
		QObject::connect(ui_.goodButton, SIGNAL(clicked(bool)), this, SLOT(triggerGood(bool)));

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

	void FSPlugin::state_callback(const cmg_msgs::State::ConstPtr & msg) {
		switch (msg->state) {
			case STATE_SAFE:
				emit setStateText("SAFE");
				emit setStateStyle("background-color: red;");
				break;
			case STATE_READY:
				emit setStateText("READY");
				emit setStateStyle("background-color: green;");
				break;
			case STATE_MISS:
				emit setStateText("RUNNING");
				emit setStateStyle("background-color: blue; color: white;");
				break;
			case STATE_POST:
				emit setStateText("DONE");
				emit setStateStyle("background-color: pink;");
				break;
		}
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
