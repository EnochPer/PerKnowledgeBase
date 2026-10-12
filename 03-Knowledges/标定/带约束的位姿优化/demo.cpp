#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <glog/logging.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <vector>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr int kPointCount = 180;
constexpr int kMaxOuterIterations = 20;
constexpr double kMaxCorrespondenceDistance = 0.30;
constexpr double kHuberDelta = 0.02;
constexpr double kConvergenceAngle = 1e-5;
constexpr double kNoiseSigma = 0.0015;

double Clamp(double value, double lower, double upper) {
  return std::max(lower, std::min(value, upper));
}

struct Correspondence {
  Eigen::Vector3d source;
  Eigen::Vector3d target;
};

// ZYX 约定：输入和输出顺序均为 [roll, pitch, yaw]，单位为弧度。
Eigen::Matrix3d RpyToRotation(const Eigen::Vector3d& rpy) {
  const double roll = rpy[0];
  const double pitch = rpy[1];
  const double yaw = rpy[2];
  return Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix() *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

Eigen::Vector3d RotationToRpy(const Eigen::Matrix3d& rotation) {
  const double sin_pitch = Clamp(-rotation(2, 0), -1.0, 1.0);
  const double pitch = std::asin(sin_pitch);
  const double cos_pitch = std::cos(pitch);
  double roll = 0.0;
  double yaw = 0.0;
  if (std::abs(cos_pitch) > 1e-8) {
    roll = std::atan2(rotation(2, 1), rotation(2, 2));
    yaw = std::atan2(rotation(1, 0), rotation(0, 0));
  } else {
    // 万向锁时 roll/yaw 不唯一，固定 roll=0 只输出一个等价解。
    roll = 0.0;
    yaw = std::atan2(-rotation(0, 1), rotation(1, 1));
  }
  return Eigen::Vector3d(roll, pitch, yaw);
}

Eigen::Matrix3d DeltaToRotation(const double delta[3]) {
  double row_major[9];
  ceres::AngleAxisToRotationMatrix(delta,
                                   ceres::RowMajorAdapter3x3(row_major));
  Eigen::Matrix3d result;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      result(r, c) = row_major[3 * r + c];
    }
  }
  return result;
}

double RotationDistance(const Eigen::Matrix3d& a, const Eigen::Matrix3d& b) {
  const double trace_term = (a.transpose() * b).trace();
  const double cosine = Clamp((trace_term - 1.0) * 0.5, -1.0, 1.0);
  return std::acos(cosine);
}

struct PointToPointResidual {
  Eigen::Vector3d source;
  Eigen::Vector3d target;
  Eigen::Vector3d fixed_translation;
  Eigen::Matrix3d current_rotation;

  template <typename T>
  bool operator()(const T* delta, T* residual) const {
    T angle_axis[3] = {delta[0], delta[1], delta[2]};
    T delta_rotation[9];
    ceres::AngleAxisToRotationMatrix(
        angle_axis, ceres::RowMajorAdapter3x3(delta_rotation));

    T rotated_in_current[3];
    for (int r = 0; r < 3; ++r) {
      rotated_in_current[r] =
          T(current_rotation(r, 0)) * T(source.x()) +
          T(current_rotation(r, 1)) * T(source.y()) +
          T(current_rotation(r, 2)) * T(source.z());
    }

    for (int r = 0; r < 3; ++r) {
      residual[r] = delta_rotation[3 * r + 0] * rotated_in_current[0] +
                    delta_rotation[3 * r + 1] * rotated_in_current[1] +
                    delta_rotation[3 * r + 2] * rotated_in_current[2] +
                    T(fixed_translation[r]) - T(target[r]);
    }
    return true;
  }
};

struct RotationPriorResidual {
  double sqrt_information;

  template <typename T>
  bool operator()(const T* delta, T* residual) const {
    for (int k = 0; k < 3; ++k) {
      residual[k] = T(sqrt_information) * delta[k];
    }
    return true;
  }
};

std::vector<Eigen::Vector3d> MakeSourceCloud() {
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> uniform(-1.0, 1.0);
  std::vector<Eigen::Vector3d> points;
  points.reserve(kPointCount);
  for (int i = 0; i < kPointCount; ++i) {
    // 非共线、非对称点集，避免旋转只在某个轴上可观测。
    const double x = uniform(rng);
    const double y = uniform(rng);
    const double z = 0.35 * x + 0.20 * y + 0.4 * uniform(rng);
    points.emplace_back(x, y, z);
  }
  return points;
}

std::vector<Eigen::Vector3d> MakeTargetCloud(
    const std::vector<Eigen::Vector3d>& source,
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& translation) {
  std::mt19937 rng(7);
  std::normal_distribution<double> noise(0.0, kNoiseSigma);
  std::vector<Eigen::Vector3d> target;
  target.reserve(source.size());
  for (const Eigen::Vector3d& point : source) {
    Eigen::Vector3d transformed = rotation * point + translation;
    transformed += Eigen::Vector3d(noise(rng), noise(rng), noise(rng));
    target.push_back(transformed);
  }
  std::shuffle(target.begin(), target.end(), rng);
  return target;
}

std::vector<Correspondence> FindCorrespondences(
    const std::vector<Eigen::Vector3d>& source,
    const std::vector<Eigen::Vector3d>& target,
    const Eigen::Matrix3d& rotation,
    const Eigen::Vector3d& translation) {
  std::vector<Correspondence> result;
  const double max_distance_squared =
      kMaxCorrespondenceDistance * kMaxCorrespondenceDistance;
  for (const Eigen::Vector3d& source_point : source) {
    const Eigen::Vector3d transformed = rotation * source_point + translation;
    double best_distance_squared = std::numeric_limits<double>::max();
    const Eigen::Vector3d* best_target = nullptr;
    for (const Eigen::Vector3d& target_point : target) {
      const double distance_squared =
          (transformed - target_point).squaredNorm();
      if (distance_squared < best_distance_squared) {
        best_distance_squared = distance_squared;
        best_target = &target_point;
      }
    }
    if (best_target != nullptr && best_distance_squared <= max_distance_squared) {
      result.push_back({source_point, *best_target});
    }
  }
  return result;
}

double ComputeRms(const std::vector<Correspondence>& correspondences,
                  const Eigen::Matrix3d& rotation,
                  const Eigen::Vector3d& translation) {
  if (correspondences.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double sum_squared = 0.0;
  for (const Correspondence& correspondence : correspondences) {
    const Eigen::Vector3d error =
        rotation * correspondence.source + translation - correspondence.target;
    sum_squared += error.squaredNorm();
  }
  return std::sqrt(sum_squared / static_cast<double>(correspondences.size()));
}

}  // namespace

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  const Eigen::Vector3d ground_truth_rpy(
      5.0 * kPi / 180.0, -3.0 * kPi / 180.0, 8.0 * kPi / 180.0);
  const Eigen::Vector3d initial_rpy(
      4.0 * kPi / 180.0, -2.0 * kPi / 180.0, 7.0 * kPi / 180.0);
  const Eigen::Vector3d fixed_translation(0.35, -0.10, 0.25);
  const Eigen::Matrix3d ground_truth_rotation =
      RpyToRotation(ground_truth_rpy);
  Eigen::Matrix3d current_rotation = RpyToRotation(initial_rpy);

  const std::vector<Eigen::Vector3d> source = MakeSourceCloud();
  const std::vector<Eigen::Vector3d> target =
      MakeTargetCloud(source, ground_truth_rotation, fixed_translation);

  for (int outer = 0; outer < kMaxOuterIterations; ++outer) {
    const std::vector<Correspondence> correspondences =
        FindCorrespondences(source, target, current_rotation, fixed_translation);
    if (correspondences.size() < 20) {
      std::cerr << "too few correspondences: " << correspondences.size()
                << std::endl;
      return 1;
    }

    double delta[3] = {0.0, 0.0, 0.0};
    ceres::Problem problem;
    for (const Correspondence& correspondence : correspondences) {
      auto* cost = new ceres::AutoDiffCostFunction<PointToPointResidual, 3, 3>(
          new PointToPointResidual{correspondence.source,
                                   correspondence.target,
                                   fixed_translation,
                                   current_rotation});
      problem.AddResidualBlock(cost, new ceres::HuberLoss(kHuberDelta), delta);
    }

    // 可选的局部姿态先验；设为 0 表示完全由 ICP 数据决定。
    constexpr double rotation_prior_lambda = 0.0;
    if (rotation_prior_lambda > 0.0) {
      auto* prior =
          new ceres::AutoDiffCostFunction<RotationPriorResidual, 3, 3>(
              new RotationPriorResidual{std::sqrt(rotation_prior_lambda)});
      problem.AddResidualBlock(prior, nullptr, delta);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 50;
    options.function_tolerance = 1e-12;
    options.gradient_tolerance = 1e-12;
    options.parameter_tolerance = 1e-12;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    if (summary.termination_type == ceres::FAILURE) {
      std::cerr << "Ceres failed: " << summary.BriefReport() << std::endl;
      return 1;
    }

    const Eigen::Matrix3d delta_rotation = DeltaToRotation(delta);
    current_rotation = delta_rotation * current_rotation;
    const double increment = std::sqrt(delta[0] * delta[0] +
                                        delta[1] * delta[1] +
                                        delta[2] * delta[2]);
    const double rms = ComputeRms(correspondences, current_rotation,
                                  fixed_translation);
    std::cout << "outer=" << outer << " correspondences="
              << correspondences.size() << " rms=" << rms
              << " delta(rad)=" << increment << std::endl;
    if (increment < kConvergenceAngle) {
      break;
    }
  }

  const Eigen::Vector3d estimated_rpy = RotationToRpy(current_rotation);
  const double rotation_error =
      RotationDistance(current_rotation, ground_truth_rotation);
  std::cout << "estimated rpy(deg): "
            << estimated_rpy.transpose() * 180.0 / kPi << std::endl;
  std::cout << "ground truth rpy(deg): "
            << ground_truth_rpy.transpose() * 180.0 / kPi << std::endl;
  std::cout << "rotation geodesic error(deg): "
            << rotation_error * 180.0 / kPi << std::endl;
  std::cout << "translation is fixed at: " << fixed_translation.transpose()
            << std::endl;
  return 0;
}
