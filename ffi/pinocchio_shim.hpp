#pragma once

#include "rust/cxx.h"

#include <cstddef>
#include <memory>

namespace airbot::native {

class PinocchioModel {
public:
  ~PinocchioModel();

  PinocchioModel(const PinocchioModel &) = delete;
  PinocchioModel &operator=(const PinocchioModel &) = delete;

  std::size_t dof() const noexcept;
  rust::Vec<double> forward_kinematics(rust::Slice<const double> joints) const;
  rust::Vec<double> inverse_kinematics(rust::Slice<const double> target_pose,
                                       rust::Slice<const double> seed) const;
  rust::Vec<double> forward_dynamics(rust::Slice<const double> joints,
                                     rust::Slice<const double> vel,
                                     rust::Slice<const double> torque) const;
  rust::Vec<double> inverse_dynamics(rust::Slice<const double> joints,
                                     rust::Slice<const double> vel,
                                     rust::Slice<const double> acc) const;

private:
  struct Impl;

  explicit PinocchioModel(std::unique_ptr<Impl> impl);

  std::unique_ptr<Impl> impl_;

  friend std::unique_ptr<PinocchioModel> load_model(rust::Str urdf_xml,
                                                    rust::Str frame_name);
};

std::unique_ptr<PinocchioModel> load_model(rust::Str urdf_xml, rust::Str frame_name);

}  // namespace airbot::native
