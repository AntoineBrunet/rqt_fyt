#ifndef RQT_FYT_FS_PLUGIN_H
#define RQT_FYT_FS_PLUGIN_H

#include <ros/ros.h>
#include <rqt_gui_cpp/plugin.h>
#include <rqt_fyt/ui_ifx.h>
#include <QWidget>
#include <QTimer>
#include "cmg_msgs/SpeedList.h"
#include "cmg_msgs/State.h"
#include "cmg_msgs/Signal.h"
#include "cmg_msgs/Guidage.h"
#include "cmg_msgs/AGConfig.h"
#include "dynamixel_workbench_msgs/DynamixelStateList.h"

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
			bool turning;
			ros::NodeHandle node;
			ros::Subscriber sub_state;
			ros::Subscriber sub_agcfg;
			ros::Subscriber sub_guidg;
			ros::Subscriber sub_fwcmd;
			ros::Subscriber sub_gimst;
			ros::Publisher pub_sig;
			ros::Publisher pub_gim;
			ros::ServiceClient setid_client;
			ros::ServiceClient calib_client;
			void state_callback(const cmg_msgs::State & msg);
			void agcfg_callback(const cmg_msgs::AGConfig::ConstPtr & msg);
			void guidg_callback(const cmg_msgs::Guidage::ConstPtr & msg);
			void fwcmd_callback(const cmg_msgs::SpeedList::ConstPtr & msg);
			void gimst_callback(const dynamixel_workbench_msgs::DynamixelStateList::ConstPtr &msg);
			template <typename T> void set_agstates(const T& states);

		private	Q_SLOTS:
			void triggerAlarm(bool checked);
			void triggerStart(bool checked);
			void triggerEnd(bool checked);
			void triggerGood(bool checked);
			void selectedId();
			void incrProgress();
			void calibrateImu(bool checked);
			void testGimbals(bool checked);

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
			void setIdDisabled(bool disable);

			void stopTimer();
			void startTimer();
			void setProgressValue(int v);

			void logManeuver(const QString str);
			void logStatus(const QString str);

			void setGimbalAngle1(int value);
			void setGimbalAngle2(int value);
			void setGimbalAngle3(int value);
			void setGimbalAngle4(int value);
			void setGimbalAngle5(int value);
			void setGimbalAngle6(int value);

			void setCurrentPara(int value);
	};
}  // namespace rqt_example_cpp
#endif  // RQT_FYT_FS_PLUGIN_H
