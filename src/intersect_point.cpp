#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/Pose2D.h>
#include <cmath>
#include "Eigen/Dense"
#ifndef PI
#define PI 3.14159265
#endif
using namespace std;
using namespace Eigen;
//场地宽度，改成实际测量值（y方向）
float WIDTH = 6.0f;
//己方场地高度，改成实际测量值（x方向）
float HEIGHT = 4.0f;
//撞到右边界后顺时针旋转角度
float clockwise_rotate_angle = -PI/2;
//撞到左边界后逆时针旋转角度
float conterclockwise_rotate_angle = PI/2;

struct robot_pose
{
	float x;
	float x_history[4];
	float y;
	float y_history[4];
	float theta;
};

robot_pose robot;		//机器人位置
robot_pose drone;		//飞机位置
robot_pose intersect;	//拦截位置

geometry_msgs::Pose2D inter_point;

class intersect_point
{
public:
	intersect_point();
private:
	ros::NodeHandle n;
		ros::Publisher point_pub;
		ros::Subscriber robot_pose_sub;
		ros::Subscriber drone_pose_sub;

	void robotCallBack(const geometry_msgs::Pose2D::ConstPtr& robot_position);
	//ARDrone的位置，消息类型要改
	void droneCallBack(const geometry_msgs::Pose2D::ConstPtr& drone_pose);
	void calc_intersect(const robot_pose robot, const robot_pose drone);

};

intersect_point::intersect_point()
{
	point_pub = n.advertise<geometry_msgs::Pose2D>("/intersect_point", 5);		//消息类型要改
	robot_pose_sub = n.subscribe<geometry_msgs::Pose2D>("/position_filtered", 5, &intersect_point::robotCallBack, this);
	drone_pose_sub = n.subscribe<geometry_msgs::Pose2D>("/drone_pose", 5, &intersect_point::droneCallBack, this);		//消息类型要改
}

void intersect_point::robotCallBack(const geometry_msgs::Pose2D::ConstPtr& robot_pose)
{
	static float last_time;
	float time_now = ros::Time::now().toSec();
	if(time_now - last_time > 0.5){
		last_time = time_now;
		for(int i=3;i>0;i--){
			robot.x_history[i] = robot.x_history[i-1];
		}
		robot.x_history[0] = robot.x;
		for(int i=3;i>0;i--){
			robot.y_history[i] = robot.y_history[i-1];
		}
	}


	robot.y_history[0] = robot.y;
	robot.x = robot_pose->x;
	robot.y = robot_pose->y;
	robot.theta = robot_pose->theta;
	intersect_point::calc_intersect(robot, drone);
}

void intersect_point::droneCallBack(const geometry_msgs::Pose2D::ConstPtr& drone_pose)
{
	drone.x = drone_pose->x;
	drone.y = drone_pose->y;
}

void intersect_point::calc_intersect(robot_pose robot, const robot_pose drone)
{
	if (robot.theta < PI/2 || robot.theta > 3 * PI / 2)
	{
		intersect.x = drone.x;
		intersect.y = drone.y;
	}
	else
	{
		float dist = 1000;
		float last_dist = 1000;
		float temp_x;
		float temp_y;
		bool update = true;

		while(update)
		{
			//机器人运行的直线方程
		//	float k = tan(robot.theta);
		//	float b = robot.y - robot.x * k;
			Matrix<float, 4, 2> A;
			Matrix<float, 4, 1> Y;
			for(int i=0;i<4;i++){
				A(i,0) = robot.x_history[i];
				A(i,1) = 1;
				Y(i,0) = robot.y_history[i];
			}
			MatrixXf B = A.transpose();
			MatrixXf AA = B*A;
			MatrixXf LU = AA.lu() .solve(B);
			MatrixXf KB = LU * Y;
			float k = KB(0);
			float b = KB(1);
			ROS_INFO("\nk: %f\nb: %f", k, b);
			
			
// first get k and b according to several points passed by
			//计算轨迹与场地边界的交点
			float left_intersect_x = (WIDTH / 2 - b) / k;
			float right_intersect_x = (-WIDTH / 2 - b) / k;
			if (left_intersect_x < -HEIGHT || right_intersect_x < -HEIGHT)
			{
				if(robot.x_history[3]>robot.x_history[0]){
					//如果交点在底线后面，直接计算飞机位置到机器人运行轨迹的垂线段交点
					if(robot.x>-1.0){
						intersect.x = -1.0;
					}
					else if(robot.x>-2.0){
						intersect.x = -2.0;
					}
					else if(robot.x>-3.0){
						intersect.x = -3.0;
					}
					else{
						intersect.x = -3.3;
					}
				}
			//	intersect.x = (drone.x / k + drone.y - b) / (k + 1/k);
				intersect.y = k * intersect.x + b;
				update = false;
				break;
			} 
			else if (left_intersect_x > right_intersect_x)
			{
				//如果交点在场地侧壁，把机器人位置更新成交点
				robot.theta += clockwise_rotate_angle;
				robot.x = right_intersect_x;
				robot.y = -WIDTH / 2;
				//更新飞机到轨迹线段的距离
				last_dist = dist;
				dist = fabs(k * drone.x - drone.y + b) / sqrt(k * k + 1);
				//如果距离比上一次大，则认为上一次是最近的拦截点，否则更新飞机到机器人运行轨迹的垂线段交点
				if (dist > last_dist)
				{
					intersect.x = temp_x;
					intersect.y = temp_y;
					update = false;
					break;
				}
				else
				{
					temp_x = (drone.x / k + drone.y - b) / (k + 1/k);
					temp_y = k * intersect.x + b;
				}
			}
			//场地另一侧边界的情况
			else
			{
				robot.theta += conterclockwise_rotate_angle;
				robot.x = left_intersect_x;
				robot.y = WIDTH / 2;
				last_dist = dist;
				dist = fabs(k * drone.x - drone.y + b) / sqrt(k * k + 1);
				if (dist > last_dist)
				{
					intersect.x = temp_x;
					intersect.y = temp_y;
					update = false;
					break;
				}
				else
				{
					temp_x = (drone.x / k + drone.y - b) / (k + 1/k);
					temp_y = k * intersect.x + b;
				}
			}
		}
	}

	//计算拦截点的移动距离，太小则仍然发布上一次的拦截位置
	float e = (inter_point.x - intersect.x) * (inter_point.x - intersect.x) + (inter_point.y - intersect.y) * (inter_point.y - intersect.y);
	if (e > 0.1)
	{
		inter_point.x = intersect.x;
		inter_point.y = intersect.y;
	}

	point_pub.publish(inter_point);
}

int main(int argc, char **argv)
{
	// float x_history[4] = {0,1,2,3};
	// 		float y_history[4] = {1,2.2,2.8,4};
	// 		Matrix<float, 4, 2> A;
	// 		Matrix<float, 4, 1> Y;
	// 		for(int i=0;i<4;i++){
	// 			A(i,0) = x_history[i];
	// 			A(i,1) = 1;
	// 			Y(i,0) = y_history[i];
	// 		}
	// 		MatrixXf B = A.transpose();
	// 		MatrixXf AA = B*A;
	// 		MatrixXf LU = AA.lu() .solve(B);
	// 		MatrixXf KB = LU * Y;
	// 		float k = KB(0);
	// 		float b = KB(1);
	// 		ROS_INFO("\nk: %f\nb: %f", k, b);

	ros::init(argc, argv, "CalculateIntersectPpoint");
	intersect_point intersect_point;
	ros::spin();
}