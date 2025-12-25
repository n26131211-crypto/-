
#ifndef ROBOT_ARM_CONTROLLER_H
#define ROBOT_ARM_CONTROLLER_H

#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>
#include <memory>
#include <string>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <brics_actuator/JointPositions.h>

// ============================================================================
// 1. 軌跡生成器基類（策略模式）
// ============================================================================
class TrajectoryGenerator {
public:
	virtual ~TrajectoryGenerator() = default;

	virtual std::vector<double> Generate(
		double start_position,
		double target_position,
		double duration,
		double timestep) = 0;
};

// ============================================================================
// 2. S-Curve 軌跡生成器
// ============================================================================
class SCurveTrajectory : public TrajectoryGenerator {
private:
	double max_velocity_;
	double max_acceleration_;

public:
	SCurveTrajectory(double max_velocity, double max_acceleration)
		: max_velocity_(max_velocity),
		max_acceleration_(max_acceleration) {
		if (max_velocity <= 0 || max_acceleration <= 0) {
			throw std::invalid_argument("Velocity and acceleration must be positive");
		}
	}

	std::vector<double> Generate(
		double start_position,
		double target_position,
		double duration,
		double timestep) override {

		std::vector<double> positions;

		// 計算移動方向和距離
		double delta_position = target_position - start_position;
		double direction = (delta_position >= 0) ? 1.0 : -1.0;
		double abs_delta = std::fabs(delta_position);

		// 動態調整速度和加速度參數
		double max_velocity = max_velocity_;
		double max_acceleration = max_acceleration_;

		// 計算理想加速時間
		double t_acc = max_velocity / max_acceleration;
		double s_acc = 0.5 * max_acceleration * t_acc * t_acc;
		double t_total_acc_dec = 2 * t_acc;

		// 計算恆速段時間
		double t_const = duration - t_total_acc_dec;

		// 如果時間不足或距離不足（三角形軌跡）
		if (t_const <= 0 || abs_delta < 2 * s_acc) {
			t_acc = duration / 2.0;
			s_acc = abs_delta / 2.0;
			t_const = 0.0;
			max_velocity = max_acceleration * t_acc;
		}
		else {
			// 調整恆速段速度
			double v_const = (abs_delta - 2 * s_acc) / t_const;
			if (v_const < max_velocity) {
				max_velocity = v_const;
				t_acc = max_velocity / max_acceleration;
				t_const = duration - 2 * t_acc;
				s_acc = 0.5 * max_acceleration * t_acc * t_acc;
			}
		}

		// 生成軌跡點
		double t = 0.0;
		while (t <= duration) {
			double pos = 0.0;

			if (t < t_acc) {
				// 加速段
				pos = start_position + direction * 0.5 * max_acceleration * t * t;
			}
			else if (t >= t_acc && t <= t_acc + t_const) {
				// 恆速段
				if (!positions.empty()) {
					pos = positions.back() + direction * max_velocity * timestep;
				}
				else {
					pos = start_position + direction * s_acc;
				}
			}
			else {
				// 減速段
				double t_dec = (2 * t_acc + t_const) - t;
				pos = target_position - direction * 0.5 * max_acceleration * t_dec * t_dec;
			}

			positions.push_back(pos);
			t += timestep;
		}

		return positions;
	}
};

// ============================================================================
// 3. 線性軌跡生成器
// ============================================================================
class LinearTrajectory : public TrajectoryGenerator {
public:
	std::vector<double> Generate(
		double start_position,
		double target_position,
		double duration,
		double timestep) override {

		std::vector<double> positions;
		size_t steps = static_cast<size_t>(duration / timestep);

		for (size_t i = 0; i <= steps; ++i) {
			double t = i * timestep;
			double ratio = t / duration;
			double pos = start_position + ratio * (target_position - start_position);
			positions.push_back(pos);
		}

		return positions;
	}
};

// ============================================================================
// 4. 單一關節類別
// ============================================================================
class Joint {
private:
	std::string name_;
	double current_position_;
	double target_position_;
	std::unique_ptr<TrajectoryGenerator> trajectory_generator_;
	std::vector<double> command_history_;
	std::vector<double> actual_history_;

public:
	Joint(const std::string& name,
		std::unique_ptr<TrajectoryGenerator> trajectory_generator)
		: name_(name),
		current_position_(0.0),
		target_position_(0.0),
		trajectory_generator_(std::move(trajectory_generator)) {}

	// 更新實際位置
	void UpdateActualPosition(double position) {
		current_position_ = position;
		actual_history_.push_back(position);
	}

	// 設定目標位置
	void SetTarget(double target_position) {
		target_position_ = target_position;
	}

	// 設定當前位置（用於初始化）
	void SetCurrentPosition(double position) {
		current_position_ = position;
	}

	// 規劃軌跡
	std::vector<double> PlanTrajectory(double duration, double timestep) {
		return trajectory_generator_->Generate(
			current_position_, target_position_, duration, timestep);
	}

	// 記錄命令值
	void RecordCommand(double command) {
		command_history_.push_back(command);
	}

	// Getters
	const std::string& GetName() const { return name_; }
	double GetCurrentPosition() const { return current_position_; }
	double GetTargetPosition() const { return target_position_; }
	const std::vector<double>& GetCommandHistory() const { return command_history_; }
	const std::vector<double>& GetActualHistory() const { return actual_history_; }
};

// ============================================================================
// 5. 機械臂控制器
// ============================================================================
class RobotArmController {
private:
	ros::NodeHandle node_handle_;
	ros::Publisher command_publisher_;
	ros::Subscriber state_subscriber_;
	std::vector<std::unique_ptr<Joint>> joints_;
	bool joints_initialized_;

public:
	RobotArmController(ros::NodeHandle& node_handle)
		: node_handle_(node_handle), joints_initialized_(false) {

		// 初始化發佈器
		command_publisher_ = node_handle_.advertise<brics_actuator::JointPositions>(
			"arm_1/arm_controller/position_command", 10);

		// 初始化訂閱器
		state_subscriber_ = node_handle_.subscribe<sensor_msgs::JointState>(
			"/joint_states", 1,
			&RobotArmController::JointStateCallback, this);
	}

	// 添加關節
	void AddJoint(std::unique_ptr<Joint> joint) {
		joints_.push_back(std::move(joint));
	}

	// 等待關節狀態初始化
	bool WaitForInitialization(double timeout_seconds = 5.0) {
		ROS_INFO("Waiting for joint_states messages...");

		ros::Time start_time = ros::Time::now();
		ros::Rate rate(20); // 20 Hz

		while (ros::ok() && !joints_initialized_) {
			ros::spinOnce();

			if ((ros::Time::now() - start_time).toSec() > timeout_seconds) {
				ROS_ERROR("Timeout waiting for joint states initialization");
				return false;
			}

			rate.sleep();
		}

		ROS_INFO("Joint states initialized successfully.");
		return true;
	}

	// 執行軌跡
	void ExecuteTrajectories(const std::vector<double>& durations, double timestep) {
		if (joints_.empty()) {
			ROS_ERROR("No joints added to controller");
			return;
		}

		if (durations.size() != joints_.size()) {
			ROS_ERROR("Duration vector size mismatch with number of joints");
			return;
		}

		// 規劃所有關節的軌跡
		std::vector<std::vector<double>> trajectories;
		for (size_t i = 0; i < joints_.size(); ++i) {
			trajectories.push_back(
				joints_[i]->PlanTrajectory(durations[i], timestep));
		}

		// 找出最長的軌跡
		size_t max_steps = GetMaxSteps(trajectories);

		// 執行軌跡
		ros::Rate rate(1.0 / timestep);

		for (size_t step = 0; step < max_steps && ros::ok(); ++step) {
			PublishCommand(step, trajectories);

			// 等待反饋
			ros::Time start_time = ros::Time::now();
			while (ros::ok() && (ros::Time::now() - start_time).toSec() < 0.1) {
				ros::spinOnce();
				ros::Duration(0.01).sleep();
			}

			// 打印當前狀態
			PrintCurrentState();

			rate.sleep();
		}

		ROS_INFO("Trajectory execution completed.");
	}

	// 獲取關節（用於資料記錄）
	const std::vector<std::unique_ptr<Joint>>& GetJoints() const {
		return joints_;
	}

private:
	// 關節狀態回調函數
	void JointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
		if (msg->position.size() < joints_.size()) {
			ROS_WARN("Received joint state has fewer joints than expected");
			return;
		}

		for (size_t i = 0; i < joints_.size(); ++i) {
			joints_[i]->UpdateActualPosition(msg->position[i]);
		}

		joints_initialized_ = true;
	}

	// 發佈命令
	void PublishCommand(size_t step,
		const std::vector<std::vector<double>>& trajectories) {
		brics_actuator::JointPositions command;
		std::vector<brics_actuator::JointValue> joint_positions(joints_.size());

		for (size_t i = 0; i < joints_.size(); ++i) {
			// 取得命令值（如果超出軌跡長度，使用目標位置）
			double command_value = (step < trajectories[i].size())
				? trajectories[i][step]
				: joints_[i]->GetTargetPosition();

			// 設定關節命令
			joint_positions[i].joint_uri = joints_[i]->GetName();
			joint_positions[i].value = command_value;
			joint_positions[i].unit = "rad";

			// 記錄命令
			joints_[i]->RecordCommand(command_value);
		}

		command.positions = joint_positions;
		command_publisher_.publish(command);
	}

	// 取得最大步數
	size_t GetMaxSteps(const std::vector<std::vector<double>>& trajectories) const {
		size_t max_steps = 0;
		for (const auto& trajectory : trajectories) {
			max_steps = std::max(max_steps, trajectory.size());
		}
		return max_steps;
	}

	// 打印當前狀態
	void PrintCurrentState() const {
		for (size_t i = 0; i < joints_.size(); ++i) {
			const auto& cmd_history = joints_[i]->GetCommandHistory();
			const auto& actual_history = joints_[i]->GetActualHistory();

			double command_value = cmd_history.empty() ? 0.0 : cmd_history.back();
			double actual_value = actual_history.empty() ? 0.0 : actual_history.back();

			ROS_INFO("Joint %lu (%s): Command = %.4f, Actual = %.4f, Error = %.4f",
				i + 1,
				joints_[i]->GetName().c_str(),
				command_value,
				actual_value,
				std::fabs(command_value - actual_value));
		}
	}
};

// ============================================================================
// 6. 資料記錄器
// ============================================================================
class DataLogger {
private:
	std::string filename_;

public:
	explicit DataLogger(const std::string& filename) : filename_(filename) {}

	// 儲存資料到 CSV
	bool SaveToCSV(const std::vector<std::unique_ptr<Joint>>& joints) {
		std::ofstream file(filename_);

		if (!file.is_open()) {
			ROS_ERROR("Failed to open file for writing: %s", filename_.c_str());
			return false;
		}

		// 寫入表頭
		file << "Joint,JointName,Step,Command,Actual,Error\n";

		// 寫入每個關節的資料
		for (size_t i = 0; i < joints.size(); ++i) {
			const auto& command_history = joints[i]->GetCommandHistory();
			const auto& actual_history = joints[i]->GetActualHistory();

			size_t num_steps = command_history.size();

			for (size_t step = 0; step < num_steps; ++step) {
				double command_value = command_history[step];
				double actual_value = (step < actual_history.size())
					? actual_history[step]
					: 0.0;
				double error = std::fabs(command_value - actual_value);

				file << (i + 1) << ","
					<< joints[i]->GetName() << ","
					<< step << ","
					<< command_value << ","
					<< actual_value << ","
					<< error << "\n";
			}
		}

		file.close();
		ROS_INFO("Joint positions successfully saved to %s", filename_.c_str());
		return true;
	}

	// 產生統計報告
	void GenerateStatisticsReport(const std::vector<std::unique_ptr<Joint>>& joints) {
		ROS_INFO("=== Trajectory Statistics ===");

		for (size_t i = 0; i < joints.size(); ++i) {
			const auto& command_history = joints[i]->GetCommandHistory();
			const auto& actual_history = joints[i]->GetActualHistory();

			if (command_history.empty() || actual_history.empty()) {
				continue;
			}

			// 計算誤差統計
			double max_error = 0.0;
			double total_error = 0.0;
			size_t count = std::min(command_history.size(), actual_history.size());

			for (size_t j = 0; j < count; ++j) {
				double error = std::fabs(command_history[j] - actual_history[j]);
				max_error = std::max(max_error, error);
				total_error += error;
			}

			double avg_error = total_error / count;

			ROS_INFO("Joint %lu (%s):",
				i + 1, joints[i]->GetName().c_str());
			ROS_INFO("  Start: %.4f, Target: %.4f, Final: %.4f",
				command_history.front(),
				joints[i]->GetTargetPosition(),
				actual_history.back());
			ROS_INFO("  Max Error: %.4f, Avg Error: %.4f",
				max_error, avg_error);
		}
	}
};

// ============================================================================
// 7. 關節配置結構
// ============================================================================
struct JointConfig {
	std::string name;
	double max_velocity;
	double max_acceleration;
	double start_position;
	double target_position;
	double duration;

	JointConfig(const std::string& n, double max_vel, double max_acc,
		double start, double target, double dur)
		: name(n), max_velocity(max_vel), max_acceleration(max_acc),
		start_position(start), target_position(target), duration(dur) {}
};

#endif // ROBOT_ARM_CONTROLLER_H

// ============================================================================
// 8. 主程式
// ============================================================================
int main(int argc, char** argv) {
	// 初始化 ROS
	ros::init(argc, argv, "s_curve_position_control_refactored");
	ros::NodeHandle node_handle;

	try {
		// 建立控制器
		RobotArmController controller(node_handle);

		// 定義關節配置
		std::vector<JointConfig> joint_configs = {
			JointConfig("arm_joint_1", 1.5, 1.5, 0.011, 4.511, 4.0),
			JointConfig("arm_joint_2", 0.5, 1.0, 0.011, 1.761, 4.0),
			JointConfig("arm_joint_3", 1.5, 1.5, -0.016, -4.516, 4.0),
			JointConfig("arm_joint_4", 0.5, 1.0, 0.023, 1.773, 4.0),
			JointConfig("arm_joint_5", 1.5, 1.5, 0.111, 4.611, 4.0)
		};

		// 添加關節到控制器
		std::vector<double> durations;
		for (const auto& config : joint_configs) {
			auto joint = std::make_unique<Joint>(
				config.name,
				std::make_unique<SCurveTrajectory>(
					config.max_velocity,
					config.max_acceleration
					)
				);

			joint->SetCurrentPosition(config.start_position);
			joint->SetTarget(config.target_position);

			controller.AddJoint(std::move(joint));
			durations.push_back(config.duration);
		}

		// 等待關節狀態初始化
		if (!controller.WaitForInitialization()) {
			ROS_ERROR("Failed to initialize joint states");
			return 1;
		}

		// 執行軌跡
		double timestep = 0.05; // 50 ms
		controller.ExecuteTrajectories(durations, timestep);

		// 記錄資料
		DataLogger logger("./joint_positions.csv");
		logger.SaveToCSV(controller.GetJoints());
		logger.GenerateStatisticsReport(controller.GetJoints());

		ROS_INFO("S-curve trajectory control completed successfully.");

	}
	catch (const std::exception& e) {
		ROS_ERROR("Exception: %s", e.what());
		return 1;
	}

	return 0;
}
