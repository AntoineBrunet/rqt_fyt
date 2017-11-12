#ifndef RQT_FYT_FS_PLUGIN_H
#define RQT_FYT_FS_PLUGIN_H

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <rqt_fyt/ui_ifx.h>
#include <QWidget>
#include <QTimer>
#include "cmg_msgs/State.h"
#include "cmg_msgs/Signal.h"
#include "cmg_msgs/AGConfig.h"

namespace rqt_fyt
{

	class FSPlugin
		: public rqt_gui_cpp::Plugin
	{
		Q_OBJECT
		public:
			FSPlugin();
			virtual void initPlugin(qt_gui_cpp::PluginContext& context);
			virtual void shutdownPlugin();
			virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
					qt_gui_cpp::Settings& instance_settings) const;
			virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
					const qt_gui_cpp::Settings& instance_settings);

			// Comment in to signal that the plugin has a way to configure it
			// bool hasConfiguration() const;
			// void triggerConfiguration();
		private:
			Ui::FSIfx ui_;
			QWidget* widget_;
			QTimer* parabola_timer;
			unsigned int tpara;
			ros::NodeHandle node;
			ros::Subscriber sub_state;
			ros::Subscriber sub_agcfg;
			ros::Publisher pub_sig;
			void state_callback(const cmg_msgs::State::ConstPtr & msg);
			void agcfg_callback(const cmg_msgs::AGConfig::ConstPtr & msg);
			template <typename T> void set_agstates(const T& states);

		private	Q_SLOTS:
			void triggerAlarm(bool checked);
			void triggerStart(bool checked);
			void triggerEnd(bool checked);
			void triggerGood(bool checked);
			void incrProgress();

		Q_SIGNALS:
			void setStateText(const QString str);
			void setStateStyle(const QString str);
			
			void setAG1Style(const QString str);
			void setAG2Style(const QString str);
			void setAG3Style(const QString str);
			void setAG4Style(const QString str);
			void setAG5Style(const QString str);
			void setAG6Style(const QString str);
			
			void setStartDisabled(bool disable);
			void setEndDisabled(bool disable);
			void setGoodDisabled(bool disable);

			void setProgressValue(int v);
	};
}  // namespace rqt_example_cpp
#endif  // RQT_FYT_FS_PLUGIN_H
