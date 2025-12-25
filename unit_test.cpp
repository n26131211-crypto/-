/*
#include <iostream>
#include <vector>
#include <cmath>
#include <iomanip>
#include <string>      
#include <sstream>     

// 從 position_control_test_0116.cpp 複製的結構
struct SCurveParams {
	double max_velocity;
	double max_acceleration;
	double start_position;
	double target_position;
};

// 複製線性軌跡函數
std::vector<double> computeLinearPositionsWithDuration(const SCurveParams &params, double duration, double time_step) {
	std::vector<double> positions;
	size_t steps = static_cast<size_t>(duration / time_step);

	for (size_t i = 0; i <= steps; ++i) {
		double t = i * time_step;
		double ratio = t / duration;
		double pos = params.start_position + ratio * (params.target_position - params.start_position);
		positions.push_back(pos);
	}

	return positions;
}

// 複製 S 曲線軌跡函數
std::vector<double> computeSCurvePositionsWithDuration(
	const SCurveParams &params, double duration, double time_step) {

	std::vector<double> positions;
	double delta_position = params.target_position - params.start_position;
	double direction = (delta_position >= 0) ? 1.0 : -1.0;
	double abs_delta = fabs(delta_position);

	double max_velocity = params.max_velocity;
	double max_acceleration = params.max_acceleration;

	double t_acc = max_velocity / max_acceleration;
	double s_acc = 0.5 * max_acceleration * t_acc * t_acc;
	double t_total_acc_dec = 2 * t_acc;

	double t_const = 0.0;
	t_const = duration - t_total_acc_dec;

	if (t_const <= 0 || abs_delta < 2 * s_acc) {
		t_acc = duration / 2.0;
		s_acc = abs_delta / 2.0;
		t_const = 0.0;
		max_velocity = max_acceleration * t_acc;
	}
	else {
		double v_const = (abs_delta - 2 * s_acc) / t_const;
		if (v_const < max_velocity) {
			max_velocity = v_const;
			t_acc = max_velocity / max_acceleration;
			t_const = duration - 2 * t_acc;
			s_acc = 0.5 * max_acceleration * t_acc * t_acc;
		}
	}

	double t = 0.0;
	while (t <= duration) {
		double pos = 0.0;
		if (t < t_acc) {
			pos = params.start_position + direction * 0.5 * max_acceleration * t * t;
		}
		else if (t >= t_acc && t <= t_acc + t_const) {
			pos = positions.back() + direction * max_velocity * time_step;
		}
		else if (t > t_acc + t_const) {
			double t_dec = (2 * t_acc + t_const) - t;
			pos = params.target_position - direction * 0.5 * max_acceleration * t_dec * t_dec;
		}
		positions.push_back(pos);
		t += time_step;
	}

	return positions;
}

// 替代 to_string 的函數
std::string doubleToString(double value) {
	std::ostringstream oss;
	oss << value;
	return oss.str();
}

// 簡單的測試輔助函數
bool assertNear(double actual, double expected, double tolerance, const std::string& test_name) {
	if (fabs(actual - expected) <= tolerance) {
		std::cout << "  [PASS] " << test_name << std::endl;
		return true;
	}
	else {
		std::cout << "  [FAIL] " << test_name << std::endl;
		std::cout << "    Expected: " << expected << ", Got: " << actual
			<< ", Diff: " << fabs(actual - expected) << std::endl;
		return false;
	}
}

int main() {
	int total_tests = 0;
	int passed_tests = 0;

	std::cout << "========================================" << std::endl;
	std::cout << "軌跡規劃單元測試（簡化版）" << std::endl;
	std::cout << "========================================" << std::endl << std::endl;

	// ========== 測試 1: 線性軌跡 - 正向移動 ==========
	std::cout << "[測試 1] 線性軌跡 - 正向移動" << std::endl;
	{
		SCurveParams params = { 1.0, 0.5, 0.0, 10.0 };
		auto positions = computeLinearPositionsWithDuration(params, 2.0, 0.1);

		total_tests += 3;
		if (assertNear(positions.front(), 0.0, 1e-6, "起點檢查")) passed_tests++;
		if (assertNear(positions.back(), 10.0, 1e-6, "終點檢查")) passed_tests++;

		bool monotonic = true;
		for (size_t i = 1; i < positions.size(); ++i) {
			if (positions[i] < positions[i - 1]) {
				monotonic = false;
				break;
			}
		}
		if (monotonic) {
			std::cout << "  [PASS] 單調遞增檢查" << std::endl;
			passed_tests++;
		}
		else {
			std::cout << "  [FAIL] 單調遞增檢查" << std::endl;
		}
	}
	std::cout << std::endl;

	// ========== 測試 2: 線性軌跡 - 負向移動 ==========
	std::cout << "[測試 2] 線性軌跡 - 負向移動" << std::endl;
	{
		SCurveParams params = { 1.0, 0.5, 10.0, 0.0 };
		auto positions = computeLinearPositionsWithDuration(params, 2.0, 0.1);

		total_tests += 2;
		if (assertNear(positions.front(), 10.0, 1e-6, "起點檢查")) passed_tests++;
		if (assertNear(positions.back(), 0.0, 1e-6, "終點檢查")) passed_tests++;
	}
	std::cout << std::endl;

	// ========== 測試 3: S 曲線 - 基本功能 ==========
	std::cout << "[測試 3] S 曲線軌跡 - 基本功能" << std::endl;
	{
		SCurveParams params = { 1.0, 0.5, 0.0, 10.0 };
		auto positions = computeSCurvePositionsWithDuration(params, 12.0, 0.1);

		total_tests += 3;
		if (assertNear(positions.front(), 0.0, 1e-2, "起點檢查")) passed_tests++;
		if (assertNear(positions.back(), 10.0, 1e-2, "終點檢查")) passed_tests++;

		if (positions.size() > 0) {
			std::cout << "  [PASS] 軌跡非空" << std::endl;
			passed_tests++;
		}
		else {
			std::cout << "  [FAIL] 軌跡非空" << std::endl;
		}
	}
	std::cout << std::endl;

	// ========== 測試 4: S 曲線 - 負向移動 ==========
	std::cout << "[測試 4] S 曲線軌跡 - 負向移動" << std::endl;
	{
		SCurveParams params = { 1.5, 1.5, 0.0, -4.5 };
		auto positions = computeSCurvePositionsWithDuration(params, 4.0, 0.05);

		total_tests += 2;
		if (assertNear(positions.front(), 0.0, 1e-2, "起點檢查")) passed_tests++;
		if (assertNear(positions.back(), -4.5, 1e-2, "終點檢查")) passed_tests++;
	}
	std::cout << std::endl;

	// ========== 測試 5-9: 實際機械臂 5 個關節 ==========
	std::cout << "[測試 5-9] 實際機械臂參數測試（5 個關節）" << std::endl;
	std::vector<SCurveParams> joint_params = {
		{1.5, 1.5, 0.011, 4.511},   // Joint 1
		{0.5, 1.0, 0.011, 1.761},   // Joint 2
		{1.5, 1.5, -0.016, -4.516}, // Joint 3
		{0.5, 1.0, 0.023, 1.773},   // Joint 4
		{1.5, 1.5, 0.111, 4.611}    // Joint 5
	};

	for (size_t i = 0; i < joint_params.size(); ++i) {
		auto positions = computeSCurvePositionsWithDuration(joint_params[i], 4.0, 0.05);

		std::cout << "  關節 " << (i + 1) << ":" << std::endl;
		total_tests += 2;

		std::ostringstream start_test, end_test;
		start_test << "    起點 (" << joint_params[i].start_position << ")";
		end_test << "    終點 (" << joint_params[i].target_position << ")";

		if (assertNear(positions.front(), joint_params[i].start_position, 1e-2, start_test.str()))
			passed_tests++;
		if (assertNear(positions.back(), joint_params[i].target_position, 1e-2, end_test.str()))
			passed_tests++;
	}
	std::cout << std::endl;

	// ========== 測試 10: 速度約束 ==========
	std::cout << "[測試 10] S 曲線速度約束驗證" << std::endl;
	{
		SCurveParams params = { 1.0, 0.5, 0.0, 10.0 };
		auto positions = computeSCurvePositionsWithDuration(params, 12.0, 0.01);

		// 計算速度
		double max_v = 0.0;
		for (size_t i = 1; i < positions.size(); ++i) {
			double v = fabs((positions[i] - positions[i - 1]) / 0.01);
			if (v > max_v) max_v = v;
		}

		total_tests++;
		std::cout << "  最大速度: " << max_v << " (限制: " << params.max_velocity << ")" << std::endl;
		if (max_v <= params.max_velocity * 1.1) {  // 允許 10% 誤差
			std::cout << "  [PASS] 速度約束" << std::endl;
			passed_tests++;
		}
		else {
			std::cout << "  [FAIL] 速度約束" << std::endl;
		}
	}
	std::cout << std::endl;

	// ========== 總結 ==========
	std::cout << "========================================" << std::endl;
	std::cout << "測試總結" << std::endl;
	std::cout << "========================================" << std::endl;
	std::cout << "通過: " << passed_tests << " / " << total_tests << std::endl;
	std::cout << "失敗: " << (total_tests - passed_tests) << std::endl;

	double pass_rate = (double)passed_tests / total_tests * 100.0;
	std::cout << std::fixed << std::setprecision(1);
	std::cout << "通過率: " << pass_rate << "%" << std::endl;

	if (passed_tests == total_tests) {
		std::cout << "\n>>> 所有測試通過！<<<" << std::endl;
		return 0;
	}
	else {
		std::cout << "\n部分測試失敗，請檢查程式碼。" << std::endl;
		return 1;
	}
}

*/