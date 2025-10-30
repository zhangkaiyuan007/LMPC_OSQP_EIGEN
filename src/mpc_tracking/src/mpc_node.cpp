#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <cmath>
#include <vector>

#include <mpc_tracking/mpc.h>

class MpcNode : public rclcpp::Node
{
public:
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

	MpcNode() : Node("mpc_node")
	{
		// Sub
		path_sub = this->create_subscription<nav_msgs::msg::Path>(
							 "/path",
							 10,
							 std::bind(&MpcNode::pathCallback,
							 this,
							 std::placeholders:: 1)
		);
		odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
							 "/odom",
							 10,
							 std::bind(&MpcNode::odomCallback,
							 this,
							 std::placeholders:: 1)
		);

		// Pub
		cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
		predict_path_pub = this->create_publisher<nav_msgs::msg::Path>("predict_path", 10);
	}

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
		for (int i = 1; i < pose_msg.poses.size(); ++i)
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
			const auto& P_ka1 = path_points[k+1];

			double t_k = P_k.t;
			double t_ka1 = P_ka1.t;

			// 位置
			double alpha = (t_sample - t_k) / (t_ka1 - t_k);
	    double Px = (1.0 - alpha) * P_k.x + alpha * P_ka1.x;
	    double Py = (1.0 - alpha) * P_k.y + alpha * P_ka1.y;

	    // 速度使用 t_sample 附近的中心差分
	    double Vx = 0.0;
	    double Vy = 0.0;
	    double Ax = 0.0;
	    double Ay = 0.0;

	    // 确保 k-1 和 k+1 索引有效
	    if (k > 0 && k < path_points.size() - 2) 
	    {
	      const auto& P_km1 = path_points[k-1];
	      const auto& P_ka2 = path_points[k+2];
	      
	      // 取前后两个点的差分
	      double dt_V = P_ka1.t - P_k.t; // 速度间隔时间
	      if (dt_V > 1e-6) 
	      {
	      	Vx = (P_ka1.x - P_k.x) / dt_V;
	        Vy = (P_ka1.y - P_k.y) / dt_V;
	      } 
	      else 
	      {
	        Vx = V_target * std::cos(std::atan2(P_ka1.y - P_k.y, P_ka1.x - P_k.x));
	        Vy = V_target * std::sin(std::atan2(P_ka1.y - P_k.y, P_ka1.x - P_k.x));
	      }
	     
	      // 加速度使用相邻速度的差分
	      double V_next_x = (P_ka2.x - P_ka1.x) / (P_ka2.t - P_ka1.t);
	      double V_next_y = (P_ka2.y - P_ka1.y) / (P_ka2.t - P_ka1.t);
	      
	      double V_prev_x = (P_k.x - P_km1.x) / (P_k.t - P_km1.t);
	      double V_prev_y = (P_k.y - P_km1.y) / (P_k.t - P_km1.t);
	      
	      double dt_A = P_ka1.t - P_k.t; // 假设加速度在 [k, k+1] 之间是恒定的
	      if (dt_A > 1e-6) 
	      {
	        Ax = (V_next_x - V_prev_x) / dt_A; 
	        Ay = (V_next_y - V_prev_y) / dt_A; 
	      }
	    } 

	    else if (k == 0) 
	    {
	      // 开始
	      Vx = V_target * std::cos(std::atan2(P_ka1.y - P_k.y, P_ka1.x - P_k.x));
	      Vy = V_target * std::sin(std::atan2(P_ka1.y - P_k.y, P_ka1.x - P_k.x));
	      Ax = 0.0;
	      Ay = 0.0;
	    } 
	    else 
	    { 
	      // 结束
	      Vx = 0.0;
	      Vy = 0.0;
	      Ax = 0.0;
	      Ay = 0.0;
	    }

	    // [ Px, Py, Vx, Vy, Ax, Ay ]
	    desired_state(i, 0) = Px;
	    desired_state(i, 1) = Py;
	    desired_state(i, 2) = Vx;
	    desired_state(i, 3) = Vy;
	    desired_state(i, 4) = Ax;
	    desired_state(i, 5) = Ay;
		}
	}
private:
	rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub;
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr predict_path_pub;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MpcNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}