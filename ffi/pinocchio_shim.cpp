#include "pinocchio_shim.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace airbot::native {
namespace {

constexpr std::size_t kArmDof = 6;
constexpr std::size_t kPoseVectorLen = 7;
constexpr std::size_t kIkErrorDim = 6;

const std::array<std::string, kArmDof> &arm_joint_names() {
  static const std::array<std::string, kArmDof> names = {
      "joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
  return names;
}

std::unordered_set<std::string> arm_joint_name_set() {
  const auto &names = arm_joint_names();
  return std::unordered_set<std::string>(names.begin(), names.end());
}

Eigen::VectorXd slice_to_vector(rust::Slice<const double> values, std::size_t expected,
                                const std::string &label) {
  if (values.size() != expected) {
    throw std::runtime_error(label + " size mismatch: expected " +
                             std::to_string(expected) + ", got " +
                             std::to_string(values.size()));
  }

  Eigen::VectorXd result(static_cast<Eigen::Index>(expected));
  for (std::size_t i = 0; i < expected; ++i) {
    result[static_cast<Eigen::Index>(i)] = values[i];
  }
  return result;
}

rust::Vec<double> eigen_to_rust_vec(const Eigen::VectorXd &vector) {
  rust::Vec<double> result;
  result.reserve(static_cast<std::size_t>(vector.size()));
  for (Eigen::Index i = 0; i < vector.size(); ++i) {
    result.push_back(vector[i]);
  }
  return result;
}

Eigen::Quaterniond pose_quaternion_from_vector(rust::Slice<const double> pose) {
  if (pose.size() != kPoseVectorLen) {
    throw std::runtime_error("target pose must contain 7 elements");
  }

  Eigen::Quaterniond quat(pose[6], pose[3], pose[4], pose[5]);
  if (quat.norm() == 0.0) {
    throw std::runtime_error("target pose quaternion must be non-zero");
  }
  quat.normalize();
  return quat;
}

Eigen::Vector3d pose_translation_from_vector(rust::Slice<const double> pose) {
  if (pose.size() != kPoseVectorLen) {
    throw std::runtime_error("target pose must contain 7 elements");
  }

  return Eigen::Vector3d(pose[0], pose[1], pose[2]);
}

Eigen::Matrix<double, kIkErrorDim, 1> pose_error(const pinocchio::SE3 &current,
                                                 const Eigen::Vector3d &target_translation,
                                                 const Eigen::Quaterniond &target_rotation) {
  Eigen::Matrix<double, kIkErrorDim, 1> error;
  error.template head<3>() = current.translation() - target_translation;

  const Eigen::Quaterniond current_rotation(current.rotation());
  Eigen::Quaterniond delta = target_rotation * current_rotation.conjugate();
  if (delta.w() < 0.0) {
    delta.coeffs() *= -1.0;
  }

  Eigen::AngleAxisd angle_axis(delta);
  error.template tail<3>() = angle_axis.axis() * angle_axis.angle();
  return error;
}

pinocchio::SE3 frame_pose(const pinocchio::Model &model, pinocchio::Data &data,
                          pinocchio::FrameIndex frame_id, const Eigen::VectorXd &q) {
  pinocchio::framesForwardKinematics(model, data, q);
  return data.oMf[frame_id];
}

rust::Vec<double> pose_to_rust_vec(const pinocchio::SE3 &pose) {
  rust::Vec<double> result;
  result.reserve(kPoseVectorLen);

  const Eigen::Quaterniond quat(pose.rotation());
  result.push_back(pose.translation().x());
  result.push_back(pose.translation().y());
  result.push_back(pose.translation().z());
  result.push_back(quat.x());
  result.push_back(quat.y());
  result.push_back(quat.z());
  result.push_back(quat.w());

  return result;
}

pinocchio::FrameIndex resolve_frame_id(const pinocchio::Model &model,
                                       const std::string &frame_name) {
  if (!frame_name.empty() && model.existFrame(frame_name)) {
    return model.getFrameId(frame_name);
  }

  if (model.existFrame("end_link")) {
    return model.getFrameId("end_link");
  }

  if (model.nframes == 0) {
    throw std::runtime_error("Pinocchio model has no frames");
  }

  return model.nframes - 1;
}

pinocchio::Model reduce_to_arm_model(const pinocchio::Model &full_model) {
  const auto arm_names = arm_joint_name_set();
  std::vector<pinocchio::JointIndex> joints_to_lock;
  joints_to_lock.reserve(full_model.njoints);

  for (pinocchio::JointIndex joint_id = 1;
       joint_id < static_cast<pinocchio::JointIndex>(full_model.njoints); ++joint_id) {
    const std::string &joint_name = full_model.names[joint_id];
    if (arm_names.count(joint_name) == 0 && full_model.joints[joint_id].nq() > 0) {
      joints_to_lock.push_back(joint_id);
    }
  }

  if (joints_to_lock.empty()) {
    return full_model;
  }

  return pinocchio::buildReducedModel(full_model, joints_to_lock, pinocchio::neutral(full_model));
}

}  // namespace

struct PinocchioModel::Impl {
  pinocchio::Model model;
  mutable pinocchio::Data data;
  pinocchio::FrameIndex frame_id;
  std::string frame_name;

  explicit Impl(pinocchio::Model reduced_model, pinocchio::FrameIndex resolved_frame_id,
                std::string resolved_frame_name)
      : model(std::move(reduced_model)),
        data(model),
        frame_id(resolved_frame_id),
        frame_name(std::move(resolved_frame_name)) {}
};

PinocchioModel::PinocchioModel(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

PinocchioModel::~PinocchioModel() = default;

std::unique_ptr<PinocchioModel> load_model(rust::Str urdf_xml, rust::Str frame_name) {
  pinocchio::Model full_model;
  pinocchio::urdf::buildModelFromXML(std::string(urdf_xml), full_model, false, false);

  pinocchio::Model reduced_model = reduce_to_arm_model(full_model);
  const std::string requested_frame(frame_name);
  const pinocchio::FrameIndex resolved_frame_id =
      resolve_frame_id(reduced_model, requested_frame);
  const std::string resolved_frame_name =
      requested_frame.empty() ? reduced_model.frames[resolved_frame_id].name : requested_frame;

  return std::unique_ptr<PinocchioModel>(new PinocchioModel(
      std::make_unique<PinocchioModel::Impl>(std::move(reduced_model), resolved_frame_id,
                                             resolved_frame_name)));
}

std::size_t PinocchioModel::dof() const noexcept {
  return static_cast<std::size_t>(impl_->model.nq);
}

rust::Vec<double> PinocchioModel::forward_kinematics(rust::Slice<const double> joints) const {
  const Eigen::VectorXd q = slice_to_vector(joints, dof(), "joint vector");
  const pinocchio::SE3 pose = frame_pose(impl_->model, impl_->data, impl_->frame_id, q);
  return pose_to_rust_vec(pose);
}

rust::Vec<double> PinocchioModel::inverse_kinematics(rust::Slice<const double> target_pose,
                                                     rust::Slice<const double> seed) const {
  const Eigen::Vector3d target_translation = pose_translation_from_vector(target_pose);
  const Eigen::Quaterniond target_rotation = pose_quaternion_from_vector(target_pose);

  Eigen::VectorXd q =
      seed.empty() ? pinocchio::neutral(impl_->model) : slice_to_vector(seed, dof(), "IK seed");

  constexpr std::size_t kMaxIterations = 200;
  constexpr double kTolerance = 1e-5;
  constexpr double kDamping = 1e-6;
  constexpr double kStepScale = 0.5;
  constexpr double kFiniteDelta = 1e-6;
  constexpr double kMaxStepNorm = 0.25;

  const Eigen::VectorXd lower = impl_->model.lowerPositionLimit;
  const Eigen::VectorXd upper = impl_->model.upperPositionLimit;

  for (std::size_t iteration = 0; iteration < kMaxIterations; ++iteration) {
    const pinocchio::SE3 current_pose = frame_pose(impl_->model, impl_->data, impl_->frame_id, q);
    const Eigen::Matrix<double, kIkErrorDim, 1> error =
        pose_error(current_pose, target_translation, target_rotation);

    if (error.norm() < kTolerance) {
      return eigen_to_rust_vec(q);
    }

    Eigen::Matrix<double, kIkErrorDim, Eigen::Dynamic> jacobian(kIkErrorDim, q.size());
    for (Eigen::Index column = 0; column < q.size(); ++column) {
      Eigen::VectorXd q_perturbed = q;
      q_perturbed[column] += kFiniteDelta;

      const pinocchio::SE3 perturbed_pose =
          frame_pose(impl_->model, impl_->data, impl_->frame_id, q_perturbed);
      const Eigen::Matrix<double, kIkErrorDim, 1> perturbed_error =
          pose_error(perturbed_pose, target_translation, target_rotation);

      jacobian.col(column) = (perturbed_error - error) / kFiniteDelta;
    }

    Eigen::MatrixXd normal = jacobian.transpose() * jacobian;
    normal.diagonal().array() += kDamping;
    Eigen::VectorXd delta =
        -normal.ldlt().solve(jacobian.transpose() * error) * kStepScale;

    const double step_norm = delta.norm();
    if (step_norm > kMaxStepNorm) {
      delta *= kMaxStepNorm / step_norm;
    }

    q += delta;
    if (lower.size() == q.size() && upper.size() == q.size()) {
      q = q.cwiseMax(lower).cwiseMin(upper);
    }
  }

  throw std::runtime_error("inverse kinematics failed to converge");
}

rust::Vec<double> PinocchioModel::forward_dynamics(rust::Slice<const double> joints,
                                                   rust::Slice<const double> vel,
                                                   rust::Slice<const double> torque) const {
  const Eigen::VectorXd q = slice_to_vector(joints, dof(), "joint vector");
  const Eigen::VectorXd v = slice_to_vector(vel, dof(), "velocity vector");
  const Eigen::VectorXd tau = slice_to_vector(torque, dof(), "torque vector");

  const Eigen::VectorXd ddq = pinocchio::aba(impl_->model, impl_->data, q, v, tau);
  return eigen_to_rust_vec(ddq);
}

rust::Vec<double> PinocchioModel::inverse_dynamics(rust::Slice<const double> joints,
                                                   rust::Slice<const double> vel,
                                                   rust::Slice<const double> acc) const {
  const Eigen::VectorXd q = slice_to_vector(joints, dof(), "joint vector");
  const Eigen::VectorXd v = slice_to_vector(vel, dof(), "velocity vector");
  const Eigen::VectorXd a = slice_to_vector(acc, dof(), "acceleration vector");

  const Eigen::VectorXd tau = pinocchio::rnea(impl_->model, impl_->data, q, v, a);
  return eigen_to_rust_vec(tau);
}

}  // namespace airbot::native
