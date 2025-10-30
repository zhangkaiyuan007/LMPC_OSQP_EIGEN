#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

bool receive_traj = false;
double path_duration;
nav_msgs::msg::Path traj;
std::vector<PathPoint> path_points;
Eigen::Vector2d current_state; 
rclcpp::Time start_time;
const int N = 30; // 预测步数
const double dt = 0.1; // 采样时间间隔
const double V_target = 1.0; // 目标速度

struct PathPoint
{
	double x;
	double y;
	double t;
};

void pathCallback(const nav_msgs::msg::Path & pose_msg) 
{
	traj = pose_msg;
	path_points.clear();

	// 添加初始路径点
	path_points.push_back({
		pose_msg.poses[0].pose.position.x,
		pose_msg.poses[0].pose.position.y,
		0.0
	});

	// 添加之后的路径点
	for (int i = 0; i < pose_msg.poses.size(); ++i)
	{
		double dx = pose_msg.poses[i].pose.position.x - pose_msg.poses[i-1].pose.position.x;
		double dy = pose_msg.poses[i].pose.position.y - pose_msg.poses[i-1].pose.position.y;
		double dist = std::sqrt(dx*dx + dy*dy);
		double dt_step = dist / V_target;

		path_points.push_back({
			pose_msg.poses[i].pose.position.x,
			pose_msg.poses[i].pose.position.y,
			path_points.back().t + dt_step
		});
	}

	// 当前路径的总时间
	path_duration = path_points.back().t;

	start_time = pose_msg.header.stamp;

	receive_traj = true;
}

void odomCallback(const nav_msgs::msg::Odometry & msg)
{
	current_state(0) = msg.pose.pose.position.x;
	current_state(1) = msg.pose.pose.position.y;
}

void publish_control_cmd()
{
	if (!receive_traj || traj.poses.size() < 3) return;

	// 当前时间
	rclcpp::Time time_now = rclcpp::Clock().now();
	double t_cur = (time_now - start_time).seconds();

	// px,py,vx,vy,ax,ay
	Eigen::MatrixXd desired_state(N + 1, 6);

	// 采样窗口内的N+1个期望状态
	for (int i = 0; i <= N; ++i)
	{
		double t_sample = t_cur + i * dt;

		// 终止采样
		if (t_sample > path_duration) t_sample = path_duration;

		// 找到采样时间所属的路径点区间
		int k = 0;
		for (k = 0; k < path_points.size(); ++k)
		{
			if (path_points[k+1].t >= t_sample) break;
		}

		// 防止溢出
		if (k >= path_points.size() - 1) k = path_points.size() - 2;

		const auto& P_k = path_points[k];
		const auto& P_k1 = path_points[k+1];

		double t_k = P_k.t;
		double t_k1 = P_k1.t;

		
	}
}