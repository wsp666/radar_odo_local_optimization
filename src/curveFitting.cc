#include <iostream>
#include <ceres/ceres.h>

// 定义残差类
struct CostFunctor {
  template <typename T>
  bool operator()(const T* const x, T* residual) const {
    residual[0] = T(2.0) * x[0] * x[0] + T(3.0) * x[0];
    return true;
  }
};

int main() {
  // 初始化问题
  ceres::Problem problem;

  // 添加残差项
  double x = 0.0; // 初始值
  problem.AddResidualBlock(new ceres::AutoDiffCostFunction<CostFunctor, 1, 1>(new CostFunctor), nullptr, &x);

  // 配置求解器选项
  ceres::Solver::Options options;
  options.minimizer_progress_to_stdout = true;

  // 求解问题
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // 输出结果
  std::cout << summary.BriefReport() << "\n";
  std::cout << "Final x: " << x << "\n";

  return 0;
}
