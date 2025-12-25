#include <iostream>
#include <iomanip>
#include <assert.h>
#include <vector>
#include <cmath>
#include <memory>
#include <string>
#include <sstream>
#include <stdexcept>
#include <algorithm>
#include <fstream>


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

// 輔助函式：檢查數值是否接近
bool ExpectNear(double actual, double expected, double tol, const std::string& msg) {
	if (std::fabs(actual - expected) <= tol) {
		std::cout << "  [PASS] " << msg << std::endl;
		return true;
	}
	else {
		std::cout << "  [FAIL] " << msg << " (Expected: " << expected << ", Got: " << actual << ")" << std::endl;
		return false;
	}
}

int main() {
	std::cout << "=== 開始測試 RobotArmController 核心邏輯 ===" << std::endl;
	int pass_count = 0;

	// 測試項目 1: 驗證 SCurveTrajectory 類別
	std::cout << "\n[測試 1] SCurveTrajectory 數學生成測試" << std::endl;
	{
		SCurveTrajectory s_curve(1.5, 1.5);
		// 參數: start=0, target=4.5, duration=4.0, timestep=0.05
		auto path = s_curve.Generate(0.0, 4.5, 4.0, 0.05);

		if (ExpectNear(path.front(), 0.0, 1e-6, "起點位置正確")) pass_count++;
		if (ExpectNear(path.back(), 4.5, 1e-2, "終點位置正確")) pass_count++;

		// 驗證速度是否符合限制 (max_v = 1.5)
		double max_v_measured = 0;
		for (size_t i = 1; i < path.size(); ++i) {
			max_v_measured = std::max(max_v_measured, std::abs(path[i] - path[i - 1]) / 0.05);
		}
		if (ExpectNear(max_v_measured, 1.5, 0.15, "最大速度約束檢查")) pass_count++;
	}

	// 測試項目 2: 驗證 Joint 類別的封裝
	std::cout << "\n[測試 2] Joint 類別與策略模式測試" << std::endl;
	{
		// 建立一個使用 S-Curve 的關節
		auto joint = std::make_unique<Joint>("joint_1", std::make_unique<SCurveTrajectory>(1.5, 1.5));
		joint->SetCurrentPosition(0.011);
		joint->SetTarget(4.511);

		auto traj = joint->PlanTrajectory(4.0, 0.05);
		if (ExpectNear(traj.back(), 4.511, 1e-2, "Joint 透過介面生成軌跡成功")) pass_count++;

		joint->RecordCommand(1.234);
		if (ExpectNear(joint->GetCommandHistory().back(), 1.234, 1e-6, "指令紀錄功能正常")) pass_count++;
	}

	// 測試項目 3: 驗證多關節配置參數 (與你的 JointConfig 一致)
	std::cout << "\n[測試 3] 實際 5 軸參數邊界檢查" << std::endl;
	{
		double test_dur = 4.0;
		double test_dt = 0.05;
		// 測試 Joint 2 (低速關節)
		SCurveTrajectory j2_curve(0.5, 1.0);
		auto j2_path = j2_curve.Generate(0.011, 1.761, test_dur, test_dt);
		if (ExpectNear(j2_path.back(), 1.761, 1e-2, "關節 2 (低速) 終點抵達")) pass_count++;

		// 測試 Joint 3 (負向移動關節)
		SCurveTrajectory j3_curve(1.5, 1.5);
		auto j3_path = j3_curve.Generate(-0.016, -4.516, test_dur, test_dt);
		if (ExpectNear(j3_path.back(), -4.516, 1e-2, "關節 3 (負向) 終點抵達")) pass_count++;
	}

	std::cout << "\n=== 測試結束，通過總數: " << pass_count << " ===" << std::endl;
	return 0;
}