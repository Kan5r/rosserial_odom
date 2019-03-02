#include <ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include "imu.h"

ros::NodeHandle nh;
geometry_msgs::Pose2D pose;
geometry_msgs::Twist twist;
geometry_msgs::TransformStamped odom_trans;
tf::TransformBroadcaster odom_broadcaster;
ros::Publisher pose2D_pub("localization", &pose);
ros::Publisher twist_pub("velocity", &twist);

constexpr int ENC0_A = 8;
constexpr int ENC0_B = 9;
constexpr int ENC1_A = 12;
constexpr int ENC1_B = 13;
constexpr int ENC2_A = 10;
constexpr int ENC2_B = 11;

constexpr double L = 110.0;
constexpr double TIRE_R = 40.0;
constexpr int PPR = 1200;

constexpr double ROOT3 = 1.732050808;

int pulse[3] = {};
int previous_pulse[3] = {};

Imu imu(Serial3);

void setup() {
  nh.initNode();
  nh.advertise(pose2D_pub);
  nh.advertise(twist_pub);
  odom_broadcaster.init(nh);

  Serial3.begin(115200);

  pinMode(ENC0_A, INPUT_PULLUP);
  pinMode(ENC0_B, INPUT_PULLUP);
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC1_B, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  pinMode(ENC2_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC0_A), encorder0Callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), encorder1Callback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encorder2Callback, CHANGE);

  pose.x = 0.0;
  pose.y = 0.0;
  pose.theta = 0.0;

  twist.linear.x = 0.0;
  twist.linear.y = 0.0;
  twist.angular.z = 0.0;
}

void loop() {
  calcOdometry();

  pose2D_pub.publish(&pose);
  //twist_pub.publish(&twist);
  broadcastTransform();
  
  nh.spinOnce();
}

void calcOdometry() {
  double v[3] = {};

  for (int i = 0; i < 3; i++) v[i] = (pulse[i] - previous_pulse[i]) * 2 * PI * TIRE_R / PPR / 1000; //[m/dt]

  twist.linear.x = (-v[0]/3 - v[1]/3 + 2 * v[2]/3);
  twist.linear.y = (ROOT3/3 * v[0] - ROOT3/3 * v[1] + 0/3  * v[2]);
  twist.angular.z = (v[0] + v[1] - v[2]) / L;

  //pose.theta +=  twist.angular.z;
  pose.theta = imu.readAngle() * PI / 180;
  pose.x += twist.linear.x * cos(pose.theta) + twist.linear.y * sin(pose.theta);
  pose.y += -twist.linear.x * sin(pose.theta) + twist.linear.y * cos(pose.theta);

  for (int i = 0; i < 3; i++) previous_pulse[i] = pulse[i];
}

void broadcastTransform() {
  odom_trans.header.frame_id = "odom";
  odom_trans.child_frame_id = "base_link";

  odom_trans.transform.translation.x = pose.x;
  odom_trans.transform.translation.y = pose.y;

  odom_trans.transform.rotation = tf::createQuaternionFromYaw(pose.theta);
  odom_trans.header.stamp = nh.now();

  odom_broadcaster.sendTransform(odom_trans);
}

void encorder0Callback() {
  if (digitalRead(ENC0_A) == HIGH) {
    if (digitalRead(ENC0_B) == LOW) {
      pulse[0]++;
    } else {
      pulse[0]--;
    }
  } else {
    if (digitalRead(ENC0_B) == HIGH) {
      pulse[0]++;
    } else {
      pulse[0]--;
    }
  }
}

void encorder1Callback() {
  if (digitalRead(ENC1_A) == HIGH) {
    if (digitalRead(ENC1_B) == LOW) {
      pulse[1]++;
    } else {
      pulse[1]--;
    }
  } else {
    if (digitalRead(ENC1_B) == HIGH) {
      pulse[1]++;
    } else {
      pulse[1]--;
    }
  }
}

void encorder2Callback() {
  if (digitalRead(ENC2_A) == HIGH) {
    if (digitalRead(ENC2_B) == LOW) {
      pulse[2]++;
    } else {
      pulse[2]--;
    }
  } else {
    if (digitalRead(ENC2_B) == HIGH) {
      pulse[2]++;
    } else {
      pulse[2]--;
    }
  }
}
