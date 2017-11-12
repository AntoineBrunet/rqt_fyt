#ifndef RQT_FYT_FS_PLUGIN_H
#define RQT_FYT_FS_PLUGIN_H

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <rqt_fyt/ui_ifx.h>
#include <QWidget>
#include "cmg_msgs/State.h"

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
			ros::NodeHandle node;
			ros::Subscriber sub_state;
			void state_callback(const cmg_msgs::State::ConstPtr & msg);

		Q_SIGNALS:
			void setStateText(const QString str);

	};
}  // namespace rqt_example_cpp
#endif  // RQT_FYT_FS_PLUGIN_H