#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/reduction_omp.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Eigenvalues>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>
#include <stdexcept>

struct HessianLogger : small_gicp::NullFactor {
  mutable int iteration = 0;

  template <typename Target, typename Source, typename Tree>
  void update_linearized_system(
      const Target&, const Source&, const Tree&, const Eigen::Isometry3d&,
      Eigen::Matrix<double, 6, 6>* H, Eigen::Matrix<double, 6, 1>*,
      double* e) const {
    if (!H->allFinite()) {
      throw std::runtime_error("Non-finite GICP Hessian");
    }
    const Eigen::Matrix<double, 6, 6> symmetric = 0.5 * (*H + H->transpose());
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(symmetric);
    if (solver.info() != Eigen::Success) {
      throw std::runtime_error("GICP Hessian eigendecomposition failed");
    }
    const auto& values = solver.eigenvalues();
    std::cout << std::scientific << std::setprecision(6)
              << "[Hessian] iter=" << iteration++ << " e=" << *e
              << "\n  eigenvalues (ascending): " << values.transpose();
    if (values[5] > 0.0) {
      std::cout << "\n  raw lambda_min/lambda_max: " << values[0] / values[5];
    } else {
      std::cout << "\n  ratio undefined: no positive maximum eigenvalue";
    }
    // Right-increment coordinates; raw mixed rotation/translation scale.
    std::cout << "\n  weakest direction [rx ry rz tx ty tz]: "
              << solver.eigenvectors().col(0).transpose() << '\n';
  }
};

int main() {
  using Point = Eigen::Vector3d;
  using small_gicp::RegistrationSetting;

  // Build a non-degenerate synthetic cloud: three planes forming a corner.
  std::vector<Point> source;
  for (int i = 0; i <= 20; ++i) {
    for (int j = 0; j <= 20; ++j) {
      const double a = -1.0 + 0.1 * i;
      const double b = -1.0 + 0.1 * j;
      source.emplace_back(a, b, 0.0);
      source.emplace_back(a, 0.0, b);
      source.emplace_back(0.0, a, b);
    }
  }

  Eigen::Isometry3d expected = Eigen::Isometry3d::Identity();
  expected.linear() =
      (Eigen::AngleAxisd(0.08, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(-0.05, Eigen::Vector3d::UnitY()))
          .toRotationMatrix();
  expected.translation() = Eigen::Vector3d(0.12, -0.08, 0.06);

  std::vector<Point> target;
  target.reserve(source.size());
  for (const Point& point : source) {
    target.push_back(expected * point);
  }

  RegistrationSetting setting;
  setting.num_threads = 2;
  setting.downsampling_resolution = 0.05;
  setting.max_correspondence_distance = 0.5;

  const Eigen::Isometry3d initial_guess = Eigen::Isometry3d::Identity();
  auto [target_cloud, target_tree] = small_gicp::preprocess_points(
      target, setting.downsampling_resolution, 10, setting.num_threads);
  auto [source_cloud, source_tree] = small_gicp::preprocess_points(
      source, setting.downsampling_resolution, 10, setting.num_threads);
  small_gicp::Registration<small_gicp::GICPFactor,
                           small_gicp::ParallelReductionOMP, HessianLogger> registration;
  registration.reduction.num_threads = setting.num_threads;
  registration.rejector.max_dist_sq =
      setting.max_correspondence_distance * setting.max_correspondence_distance;
  registration.criteria.rotation_eps = setting.rotation_eps;
  registration.criteria.translation_eps = setting.translation_eps;
  registration.optimizer.max_iterations = setting.max_iterations;
  registration.optimizer.verbose = true;
  const auto result = registration.align(
      *target_cloud, *source_cloud, *target_tree, initial_guess);

  const Eigen::Isometry3d error = result.T_target_source.inverse() * expected;
  const double translation_error = error.translation().norm();
  const double rotation_error =
      Eigen::AngleAxisd(error.linear()).angle();

  std::cout << std::fixed << std::setprecision(6);
  std::cout << "converged: " << std::boolalpha << result.converged << '\n'
            << "iterations: " << result.iterations << '\n'
            << "inliers: " << result.num_inliers << '\n'
            << "estimated T_target_source:\n"
            << result.T_target_source.matrix() << '\n'
            << "translation error [m]: " << translation_error << '\n'
            << "rotation error [rad]: " << rotation_error << '\n';

  return result.converged && translation_error < 1e-2 &&
                 rotation_error < 1e-2
             ? 0
             : 1;
}
