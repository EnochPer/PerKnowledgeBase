#include <small_gicp/registration/registration_helper.hpp>
#include <small_gicp/registration/registration.hpp>
#include <small_gicp/registration/reduction_omp.hpp>
#include <small_gicp/factors/gicp_factor.hpp>
#include <small_gicp/util/lie.hpp>
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <vector>

// Only the rotation block is solved; no translation increment is estimated.
struct RotationOnlyLM {
  template <typename Target, typename Source, typename Tree, typename Rejector,
            typename Criteria, typename Reduction, typename Factor, typename GeneralFactor>
  small_gicp::RegistrationResult optimize(
      const Target& target, const Source& source, const Tree& tree,
      const Rejector& rejector, const Criteria& criteria, Reduction& reduction,
      const Eigen::Isometry3d& initial, std::vector<Factor>& factors,
      GeneralFactor& general_factor) const {
    if (!initial.matrix().allFinite()) {
      throw std::runtime_error("Non-finite initial pose");
    }
    small_gicp::RegistrationResult result(initial);
    const Eigen::Vector3d fixed_translation = initial.translation();
    double damping = 1e-3;
    for (int iteration = 0; iteration < 30; ++iteration) {
      auto [H, b, e] = reduction.linearize(
          target, source, tree, rejector, result.T_target_source, factors);
      general_factor.update_linearized_system(
          target, source, tree, result.T_target_source, &H, &b, &e);
      result.num_inliers = std::count_if(factors.begin(), factors.end(),
                                        [](const auto& f) { return f.inlier(); });
      if (result.num_inliers < 20 || !H.allFinite() || !b.allFinite() || !std::isfinite(e)) {
        throw std::runtime_error("Insufficient correspondences or non-finite linearization");
      }
      const Eigen::Matrix3d block = H.template topLeftCorner<3, 3>();
      const Eigen::Matrix3d Hrr = 0.5 * (block + block.transpose());
      const Eigen::Vector3d br = b.template head<3>();
      const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen(Hrr);
      if (eigen.info() != Eigen::Success || !eigen.eigenvalues().allFinite() ||
          eigen.eigenvalues()[2] <= 0.0) {
        throw std::runtime_error("Invalid rotational Hessian spectrum");
      }
      const auto& values = eigen.eigenvalues();
      if (values[0] < -1e-10 * values[2]) {
        throw std::runtime_error("Rotational Hessian is not positive semidefinite");
      }
      std::cout << "[rotation Hessian] iter=" << iteration
                << " inliers=" << result.num_inliers << " e=" << e
                << "\n  eigenvalues: " << values.transpose()
                << "\n  min/max: " << values[0] / values[2]
                << "\n  weakest [rx ry rz]: " << eigen.eigenvectors().col(0).transpose()
                << '\n';
      if (values[0] / values[2] < 1e-4) {
        std::cout << "  warning: weak rotational constraint (diagnostic threshold)\n";
      }
      // Keep undamped information at the last linearization, like the upstream result.
      result.H = H;
      result.b = b;
      result.error = e;
      result.iterations = iteration + 1;
      bool accepted = false;
      for (int inner = 0; inner < 10; ++inner) {
        const Eigen::Matrix3d system = Hrr + damping * Eigen::Matrix3d::Identity();
        const Eigen::LDLT<Eigen::Matrix3d> solver(system);
        if (solver.info() != Eigen::Success || !solver.isPositive()) {
          throw std::runtime_error("Rotation linear solve factorization failed");
        }
        const Eigen::Vector3d rotation_step = solver.solve(-br);
        if (solver.info() != Eigen::Success || !rotation_step.allFinite()) {
          throw std::runtime_error("Invalid rotation increment");
        }
        Eigen::Matrix<double, 6, 1> step = Eigen::Matrix<double, 6, 1>::Zero();
        step.head<3>() = rotation_step;
        Eigen::Isometry3d candidate = result.T_target_source;
        candidate.linear() = result.T_target_source.linear() *
                             small_gicp::se3_exp(step).linear();
        candidate.translation() = fixed_translation;
        double next_error = reduction.error(target, source, candidate, factors);
        general_factor.update_error(target, source, candidate, &next_error);
        const bool improves = std::isfinite(next_error) && next_error <= e;
        std::cout << "iter=" << iteration << " inner=" << inner
                  << " new_e=" << next_error << " damping=" << damping
                  << " dr=" << rotation_step.norm()
                  << " translation_drift=" << (candidate.translation() - fixed_translation).norm()
                  << " accepted=" << improves << '\n';
        if (improves) {
          result.T_target_source = candidate;
          result.error = next_error;
          result.converged = rotation_step.norm() < criteria.rotation_eps;
          damping /= 10.0;
          accepted = true;
          break;
        }
        damping *= 10.0;
      }
      if (!accepted) {
        throw std::runtime_error("No acceptable rotation step after 10 LM trials");
      }
      if (result.converged) {
        return result;
      }
    }
    return result;
  }
};

int main() {
  try {
    std::cout << std::scientific << std::setprecision(6) << std::boolalpha;
    std::vector<Eigen::Vector3d> source, target;
    for (int i = 0; i <= 20; ++i) {
      for (int j = 0; j <= 20; ++j) {
        const double a = -1.0 + 0.1 * i;
        const double b = -1.0 + 0.1 * j;
        source.emplace_back(a, b, 0.0);
        source.emplace_back(a, 0.0, b);
        source.emplace_back(0.0, a, b);
      }
    }
    Eigen::Isometry3d truth = Eigen::Isometry3d::Identity();
    truth.linear() = (Eigen::AngleAxisd(0.08, Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(-0.05, Eigen::Vector3d::UnitY())).toRotationMatrix();
    truth.translation() = Eigen::Vector3d(0.12, -0.08, 0.06);
    for (const auto& point : source) {
      target.push_back(truth * point);
    }
    // Single-threaded preprocessing makes the synthetic comparisons reproducible.
    auto [target_cloud, tree] = small_gicp::preprocess_points(target, 0.05, 10, 1);
    auto source_preprocessed = small_gicp::preprocess_points(source, 0.05, 10, 1);
    small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP,
                             small_gicp::NullFactor, small_gicp::DistanceRejector,
                             RotationOnlyLM> registration;
    registration.reduction.num_threads = 1;
    registration.rejector.max_dist_sq = 0.5 * 0.5;
    registration.criteria.rotation_eps = 1e-7;
    bool passed = true;
    for (const bool biased : {false, true}) {
      Eigen::Isometry3d initial = Eigen::Isometry3d::Identity();
      initial.translation() = truth.translation();
      if (biased) {
        initial.translation().x() += 0.02;
      }
      std::cout << "\n--- " << (biased ? "biased translation (+0.02 m x)" : "correct translation")
                << " ---\n";
      const auto result = registration.align(*target_cloud, *source_preprocessed.first, *tree, initial);
      const double drift = (result.T_target_source.translation() - initial.translation()).norm();
      const double rotation_error = Eigen::AngleAxisd(
          truth.linear().transpose() * result.T_target_source.linear()).angle();
      double squared_error = 0.0;
      // Known synthetic correspondences give a diagnostic independent of nearest-neighbor matching.
      for (std::size_t i = 0; i < source.size(); ++i) {
        squared_error += (result.T_target_source * source[i] - target[i]).squaredNorm();
      }
      std::cout << "converged: " << result.converged
                << "\nouter iterations (count): " << result.iterations
                << "\nT_target_source:\n" << result.T_target_source.matrix()
                << "\ntranslation drift [m]: " << drift
                << "\nrotation error [rad]: " << rotation_error
                << "\nknown-pair RMSE [m]: " << std::sqrt(squared_error / source.size()) << '\n';
      passed = passed && result.converged && drift == 0.0 &&
               result.T_target_source.matrix().allFinite() &&
               (biased || rotation_error < 1e-3);
    }
    if (!passed) {
      std::cerr << "Synthetic self-check failed\n";
    }
    return passed ? 0 : 1;
  } catch (const std::exception& error) {
    std::cerr << "Error: " << error.what() << '\n';
    return 1;
  }
}
