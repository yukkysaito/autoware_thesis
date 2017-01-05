/*
 *  Copyright (c) 2015, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice,
 * this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/* ----header---- */
/* common header */
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Header.h"
#include "std_msgs/Time.h"
#include <boost/circular_buffer.hpp>
#include <iostream>
#include <sstream>
/* user header */
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud2.h"
#include "synchronization/time_diff.h"

//#define OUTPUT_LOG
#ifdef OUTPUT_LOG
#include <fstream>
std::ofstream image_writing_file;
std::ofstream points_writing_file;
std::ofstream diff_writing_file;
#endif

/* ----var---- */
/* common var */
/* user var */
sensor_msgs::Image::Ptr image_raw_buf;
sensor_msgs::PointCloud2::Ptr points_raw_buf;

ros::Publisher points_raw__pub;
ros::Publisher image_raw__pub;
ros::Publisher time_diff_pub;
ros::Publisher lidar_cycle_pub;
ros::Publisher camera_cycle_pub;

bool is_sim;
ros::Time prev_image_time;
ros::Time prev_points_time;

double fabs_time_diff(const ros::Time *timespec1, const ros::Time *timespec2) {
  double time1 = (double)timespec1->sec + (double)timespec1->nsec / 1000000000L;
  double time2 = (double)timespec2->sec + (double)timespec2->nsec / 1000000000L;
  return fabs(time1 - time2);
}

void image_raw_callback(sensor_msgs::Image::Ptr image_raw_msg) {
  // std::cout << "image :" << image_raw_msg->header.stamp.sec << "."
  //          << image_raw_msg->header.stamp.nsec << std::endl;
  if (image_raw_buf) {
    std_msgs::Float64 camera_cycle_msg;
    camera_cycle_msg.data =
        fabs_time_diff(&prev_image_time, &(image_raw_msg->header.stamp)) *
        1000.0; // msec
    camera_cycle_pub.publish(camera_cycle_msg);
#ifdef OUTPUT_LOG
    image_writing_file << camera_cycle_msg.data << std::endl;
#endif
  }

  prev_image_time = image_raw_msg->header.stamp;
  image_raw_buf = image_raw_msg;
}

void points_raw_callback(sensor_msgs::PointCloud2::Ptr points_raw_msg) {
  // std::cout << "points:" << points_raw_msg->header.stamp.sec << "."
  //           << points_raw_msg->header.stamp.nsec << std::endl;
  if (points_raw_buf) {
    std_msgs::Float64 lidar_cycle_msg;
    lidar_cycle_msg.data = fabs_time_diff(&(points_raw_msg->header.stamp),
                                          &(points_raw_buf->header.stamp)) *
                           1000.0; // msec
    lidar_cycle_pub.publish(lidar_cycle_msg);
#ifdef OUTPUT_LOG
    points_writing_file << lidar_cycle_msg.data << std::endl;
#endif
  }
  prev_points_time = points_raw_msg->header.stamp;
  points_raw_buf = points_raw_msg;

  if (!image_raw_buf)
    return;
  synchronization::time_diff time_diff_msg;
  time_diff_msg.header.frame_id = "world";
  time_diff_msg.header.stamp = points_raw_msg->header.stamp;
  time_diff_msg.time_diff =
      fabs_time_diff(&(prev_points_time), &(prev_image_time)) * 1000.0; // msec
  time_diff_msg.camera = image_raw_buf->header.stamp;
  time_diff_msg.lidar = points_raw_msg->header.stamp;
  time_diff_pub.publish(time_diff_msg);

  image_raw_buf->header.stamp = points_raw_msg->header.stamp;
  image_raw__pub.publish(image_raw_buf);
  points_raw__pub.publish(points_raw_msg);
#ifdef OUTPUT_LOG
  diff_writing_file << time_diff_msg.time_diff << std::endl;
#endif
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "sync_drivers");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
#ifdef OUTPUT_LOG
  image_writing_file.open("camera_cycle.csv");
  points_writing_file.open("lidar_cycle.csv");
  diff_writing_file.open("time_diff.csv");
#endif
  ros::Subscriber image_raw_sub =
      nh.subscribe("/image_raw", 1, image_raw_callback);
  ros::Subscriber points_raw_sub;
  points_raw_sub = nh.subscribe("/points_raw", 1, points_raw_callback);

  image_raw__pub = nh.advertise<sensor_msgs::Image>("image_raw", 1);
  points_raw__pub = nh.advertise<sensor_msgs::PointCloud2>("points_raw", 1);
  time_diff_pub =
      nh.advertise<synchronization::time_diff>("/time_difference", 1);
  camera_cycle_pub = nh.advertise<std_msgs::Float64>("/camera_cycle", 1);
  lidar_cycle_pub = nh.advertise<std_msgs::Float64>("/lidar_cycle", 1);

  ros::spin();
#ifdef OUTPUT_LOG
  image_writing_file.close();
  points_writing_file.close();
  diff_writing_file.close();
#endif
  return 0;
}
