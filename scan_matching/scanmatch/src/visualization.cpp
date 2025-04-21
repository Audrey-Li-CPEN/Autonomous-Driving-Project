#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include "scan_matching_skeleton/visualization.h"

PointVisualizer::PointVisualizer(rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub, string ns, string frame_id) : pub(pub), ns(ns),
      frame_id(frame_id) {
  dots.header.frame_id = frame_id;
  dots.ns = ns;
  dots.action = visualization_msgs::msg::Marker::ADD;
  dots.pose.orientation.w = 1.0;
  dots.id = num_visuals;
  dots.type = visualization_msgs::msg::Marker::POINTS;
  dots.scale.x = dots.scale.y = 0.05;
  ++num_visuals;
}

void PointVisualizer::addPoints(vector<Point>& points, std_msgs::msg::ColorRGBA color) {
  for (Point p : points) {
    dots.points.push_back(p.getPoint());
    dots.colors.push_back(color);
  }
}

void PointVisualizer::publishPoints() {
  dots.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  pub->publish(dots);
  RCLCPP_INFO(rclcpp::get_logger("visualization"), "%s %d", "published dots", dots.points.size());
  dots.points.clear();
  dots.colors.clear();
}