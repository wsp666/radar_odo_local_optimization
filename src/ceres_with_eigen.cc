#include <ceres/ceres.h>
#include <eigen3/Eigen/Core>

struct CostFunctor {
  template <typename T>
  bool operator()(const T* const x, T* residual) const {
    // 使用 Eigen 的函数和类定义 cost function
    Eigen::Matrix<T, 2, 1> vec;
    vec << T(1.0), T(2.0);
    std::cout << "vec:\n" <<vec << std::endl << std::endl;
    Eigen::Matrix<T, 2, 1> result = vec * x[0];
    std::cout << "result:\n" << result << std::endl;

    // 计算残差
    residual[0] = result[0] - T(2.0);
    residual[1] = result[1] - T(4.0);
    std::cout << "oprating this for one time!" << std::endl;
    return true;
  }
};

int main() {
  // 创建 Ceres 优化问题
  ceres::Problem problem;

  // 创建参数
  double x = 1.0;

  // 添加 cost function 到问题中
  problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CostFunctor, 2, 1>(new CostFunctor),
      nullptr, &x);

  // 运行优化
  ceres::Solver::Options options;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  // 输出优化结果
  std::cout << "Optimized x: " << x << std::endl;

  return 0;
}
