/**
 * @brief Mouny Control plugin
 * @file mount_control.cpp
 * @author Jaeyoung Lim <jaeyoung@auterion.com>
 *
 * @addtogroup plugin
 * @{
 */
/*
 * Copyright 2019 Jaeyoung Lim.
 *
 * This file is part of the mavros package and subject to the license terms
 * in the top-level LICENSE file of the mavros repository.
 * https://github.com/mavlink/mavros/tree/master/LICENSE.md
 */

#include <mavros/mavros_plugin.h>

#include <mavros_msgs/MountControl.h>
#include <geometry_msgs/Quaternion.h>

namespace mavros {
namespace extra_plugins {

//! Mavlink enumerations
using mavlink::common::MAV_MOUNT_MODE;
using mavlink::common::MAV_CMD;
using utils::enum_value;

/**
 * @brief Mount Control plugin
 *
 * Publishes Mission commands to control the camera or antenna mount.
 * @see command_cb()
 */
class MountControlPlugin : public plugin::PluginBase {
public:
	MountControlPlugin() : PluginBase(),
	mount_nh("~mount_control")
	{ }

	void initialize(UAS &uas_)
	{
		PluginBase::initialize(uas_);

		command_sub = mount_nh.subscribe("command", 10, &MountControlPlugin::command_cb, this);
		mount_orientation_pub = mount_nh.advertise<geometry_msgs::Quaternion>("orientation", 10);
	}

	Subscriptions get_subscriptions()
	{
		return {
			make_handler(&MountControlPlugin::handle_mount_orientation)
		};
	}

private:
	ros::NodeHandle mount_nh;
	ros::Subscriber command_sub;
	ros::Publisher mount_orientation_pub;

	/**
	 * @brief Publish the mount orientation
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#MOUNT_ORIENTATION
	 * @param msg   the mavlink message
	 * @param mo	received MountOrientation msg
	 */
	void handle_mount_orientation(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MOUNT_ORIENTATION &mo)
	{
		auto q = ftf::quaternion_from_rpy(Eigen::Vector3d(mo.roll, mo.pitch, mo.yaw) * M_PI / 180.0);
		geometry_msgs::Quaternion quaternion_msg;
		tf::quaternionEigenToMsg(q, quaternion_msg);
		mount_orientation_pub.publish(quaternion_msg);
	}

	/**
	 * @brief Send mount control commands to vehicle
	 *
	 * Message specification: https://mavlink.io/en/messages/common.html#MAV_CMD_DO_MOUNT_CONTROL
	 * @param req	received MountControl msg
	 */
	void command_cb(const mavros_msgs::MountControl::ConstPtr &req)
	{
		mavlink::common::msg::COMMAND_LONG cmd {};

		cmd.target_system = m_uas->get_tgt_system();
		cmd.target_component = m_uas->get_tgt_component();
		cmd.command = enum_value(MAV_CMD::DO_MOUNT_CONTROL);
		cmd.param1 = req->pitch;
		cmd.param2 = req->roll;
		cmd.param3 = req->yaw;
		cmd.param4 = req->altitude; // 
		cmd.param5 = req->latitude; // lattitude in degrees * 1E7
		cmd.param6 = req->longitude; // longitude in degrees * 1E7
		cmd.param7 = req->mode; // MAV_MOUNT_MODE

		UAS_FCU(m_uas)->send_message_ignore_drop(cmd);
	}
};
}	// namespace extra_plugins
}	// namespace mavros

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::MountControlPlugin, mavros::plugin::PluginBase)
