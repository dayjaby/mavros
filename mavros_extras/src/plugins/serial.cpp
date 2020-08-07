/**
 * @brief GPS RTK plugin
 * @file gps_rtk.cpp
 * @author Alexis Paques <alexis.paques@gmail.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2018 Alexis Paques.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>
#include <mavros_msgs/Serial.h>
#include <std_msgs/String.h>
#include <algorithm>

namespace mavros {
namespace extra_plugins {
/**
 * @brief GPS RTK plugin
 *
 * Publish the RTCM messages from ROS to the FCU
 */
class SerialPlugin : public plugin::PluginBase {
public:
	SerialPlugin() : PluginBase(),
		serial_nh("~serial")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);
		serial_sub = serial_nh.subscribe("send", 10, &SerialPlugin::serial_send_cb, this);
		serial_str_sub = serial_nh.subscribe("send_str", 10, &SerialPlugin::serial_send_str_cb, this);
		serial_pub = serial_nh.advertise<mavros_msgs::Serial>("recv", 10);
		serial_str_pub = serial_nh.advertise<std_msgs::String>("recv_str", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			       make_handler(&SerialPlugin::handle_serial_control),
		};
	}

private:
	ros::NodeHandle serial_nh;
	ros::Subscriber serial_sub;
	ros::Publisher serial_pub;
	ros::Subscriber serial_str_sub;
	ros::Publisher serial_str_pub;

	/* -*- callbacks -*- */
	/**
	 * @brief Handle mavros_msgs::RTCM message
	 * It converts the message to the MAVLink GPS_RTCM_DATA message for GPS injection.
	 * Message specification: https://mavlink.io/en/messages/common.html#GPS_RTCM_DATA
	 * @param msg		Received ROS msg
	 */
	void serial_send_cb(const mavros_msgs::Serial::ConstPtr &msg)
	{
		mavlink::common::msg::SERIAL_CONTROL serial_control;

		auto data_it = msg->data.begin();
		auto end_it = msg->data.end();

		serial_control.device = static_cast<uint8_t>(mavlink::common::SERIAL_CONTROL_DEV::SHELL);
		serial_control.timeout = 1000;
		serial_control.count = msg->data.size();
		std::copy(data_it, end_it, serial_control.data.begin());
		std::fill(serial_control.data.begin() + serial_control.count, serial_control.data.end(), 0);
		UAS_FCU(m_uas)->send_message(serial_control);
	}

	void serial_send_str_cb(const std_msgs::String::ConstPtr &msg)
	{
		mavlink::common::msg::SERIAL_CONTROL serial_control;

		auto data_it = msg->data.begin();
		auto end_it = msg->data.end();

		serial_control.device = static_cast<uint8_t>(mavlink::common::SERIAL_CONTROL_DEV::SHELL);
		serial_control.timeout = 1000;
		serial_control.count = msg->data.size();
		std::copy(data_it, end_it, serial_control.data.begin());
		std::fill(serial_control.data.begin() + serial_control.count, serial_control.data.end(), 0);
		UAS_FCU(m_uas)->send_message(serial_control);
	}

	void handle_serial_control(const mavlink::mavlink_message_t *msg, mavlink::common::msg::SERIAL_CONTROL &serial_control)
	{
		auto serial_msg = boost::make_shared<mavros_msgs::Serial>();
		auto str_msg = boost::make_shared<std_msgs::String>();
		auto data_it = serial_msg->data.begin();
		auto end_it = serial_msg->data.end();

		if(serial_control.count > 70) {
			return;
		}
		std::copy(serial_control.data.begin(), serial_control.data.begin() + serial_control.count, data_it);
		std::fill(data_it + serial_control.count, end_it, 0);
		serial_pub.publish(serial_msg);

		str_msg->data.resize(serial_control.count);
		std::copy(serial_control.data.begin(), serial_control.data.begin() + serial_control.count, str_msg->data.begin());
		serial_str_pub.publish(str_msg);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::SerialPlugin, mavros::plugin::PluginBase)
