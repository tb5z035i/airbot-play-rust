use std::cmp::Ordering;
use std::f64::consts::{FRAC_PI_2, PI, TAU};

use nalgebra::{Matrix4, Quaternion, Rotation3, UnitQuaternion};

use super::backend::{KinematicsDynamicsBackend, ModelError, Pose};
use super::pinocchio::PinocchioBackend;
use super::urdf::{MountedEefType, bundled_urdf_xml, default_frame_name};

const ARM_DOF: usize = 6;

#[derive(Clone)]
struct JointLimits {
    lower: f64,
    upper: f64,
}

#[derive(Clone)]
struct AnalyticalParameters {
    alpha: [f64; ARM_DOF],
    a: [f64; ARM_DOF],
    d: [f64; ARM_DOF],
    theta_offset: [f64; ARM_DOF],
    joint_limits: [JointLimits; ARM_DOF],
    end_convert: Matrix4<f64>,
    limit_punish: [f64; ARM_DOF],
    bias_punish: [f64; ARM_DOF],
    shoulder_singularity_threshold: f64,
    wrist_singularity_threshold: f64,
}

pub struct PlayAnalyticalBackend {
    kinematics: AnalyticalParameters,
    dynamics: PinocchioBackend,
}

impl PlayAnalyticalBackend {
    pub fn load(mounted_eef: MountedEefType) -> Result<Self, ModelError> {
        let urdf_xml = bundled_urdf_xml(&mounted_eef)?;
        let dynamics = PinocchioBackend::load_from_urdf_xml(
            mounted_eef.clone(),
            urdf_xml,
            default_frame_name(&mounted_eef),
        )?;
        Ok(Self {
            kinematics: AnalyticalParameters::for_mounted_eef(&mounted_eef),
            dynamics,
        })
    }

    fn validate_len(&self, label: &'static str, values: &[f64]) -> Result<(), ModelError> {
        if values.len() != ARM_DOF {
            return Err(ModelError::VectorLengthMismatch {
                label,
                expected: ARM_DOF,
                got: values.len(),
            });
        }
        Ok(())
    }

    fn joints_from_slice(
        &self,
        label: &'static str,
        values: &[f64],
    ) -> Result<[f64; ARM_DOF], ModelError> {
        self.validate_len(label, values)?;
        let mut joints = [0.0; ARM_DOF];
        joints.copy_from_slice(values);
        Ok(joints)
    }

    fn forward_transform(&self, joints: &[f64; ARM_DOF]) -> Matrix4<f64> {
        let mut transform = Matrix4::<f64>::identity();
        for (index, joint) in joints.iter().enumerate() {
            transform *= self.kinematics.adjacent_transform(*joint, index);
        }
        transform * self.kinematics.end_convert
    }

    fn solve_inverse_kinematics(
        &self,
        target: &Pose,
        reference: Option<&[f64; ARM_DOF]>,
        force_calculate: bool,
        use_clip: bool,
    ) -> Result<Vec<[f64; ARM_DOF]>, ModelError> {
        let target_transform = pose_to_matrix(target);
        let end_inverse = self.kinematics.end_convert.try_inverse().ok_or_else(|| {
            ModelError::Backend("analytical end transform is singular".to_owned())
        })?;
        let arm_pose = target_transform * end_inverse;
        let wrist_position = arm_pose.fixed_view::<3, 1>(0, 3);
        let x = wrist_position[(0, 0)];
        let y = wrist_position[(1, 0)];
        let z = wrist_position[(2, 0)];
        let dz = z - self.kinematics.d[0];
        let numerator = -(x * x + y * y + dz * dz
            - self.kinematics.d[3] * self.kinematics.d[3]
            - self.kinematics.a[2] * self.kinematics.a[2]);
        let denominator = 2.0 * self.kinematics.d[3] * self.kinematics.a[2];

        let mut s3 = numerator / denominator;
        if !(-1.0..=1.0).contains(&s3) {
            if force_calculate {
                s3 = s3.clamp(-1.0, 1.0);
            } else {
                return Ok(Vec::new());
            }
        }

        let mut solutions = Vec::with_capacity(8);
        for shoulder_sign in [1.0, -1.0] {
            let mut theta1 =
                (shoulder_sign * y).atan2(shoulder_sign * x) - self.kinematics.theta_offset[0];
            if x.abs() < self.kinematics.shoulder_singularity_threshold
                && y.abs() < self.kinematics.shoulder_singularity_threshold
            {
                theta1 = reference.map(|seed| seed[0]).unwrap_or(0.0);
            }

            for elbow_sign in [1.0, -1.0] {
                let c3 = elbow_sign * (1.0 - s3 * s3).max(0.0).sqrt();
                let theta3 = s3.atan2(c3) - self.kinematics.theta_offset[2];

                let k1 = self.kinematics.a[2] - self.kinematics.d[3] * s3;
                let k2 = self.kinematics.d[3] * c3;
                let k3 = (x * x + y * y).sqrt();
                let k4 = z - self.kinematics.d[0];
                let theta2 = (-shoulder_sign * k2 * k3 + k1 * k4)
                    .atan2(shoulder_sign * k1 * k3 + k2 * k4)
                    - self.kinematics.theta_offset[1];

                let t30 = self.kinematics.adjacent_transform(theta1, 0)
                    * self.kinematics.adjacent_transform(theta2, 1)
                    * self.kinematics.adjacent_transform(theta3, 2);
                let t30_inverse = t30.try_inverse().ok_or_else(|| {
                    ModelError::Backend("analytical shoulder transform is singular".to_owned())
                })?;
                let t63 = t30_inverse * arm_pose;

                let tp15 = (t63[(1, 0)].powi(2) + t63[(1, 1)].powi(2)).sqrt();
                let q5 = tp15.atan2(t63[(1, 2)]);
                let mut q4 = 0.0;
                let mut q6 = 0.0;
                if q5.abs() > self.kinematics.wrist_singularity_threshold {
                    q4 = t63[(2, 2)].atan2(-t63[(0, 2)]);
                    q6 = (-t63[(1, 1)]).atan2(t63[(1, 0)]);
                }

                for wrist_sign in [1.0, -1.0] {
                    let theta5 = wrist_sign * q5 - self.kinematics.theta_offset[4];
                    let (mut theta4, mut theta6) =
                        if theta5.abs() > self.kinematics.wrist_singularity_threshold {
                            let mut theta4 = q4 - self.kinematics.theta_offset[3];
                            let mut theta6 = q6 - self.kinematics.theta_offset[5];
                            if wrist_sign < 0.0 {
                                theta4 += PI;
                                theta6 += PI;
                            }
                            (theta4, theta6)
                        } else {
                            let rotation_angle = (-t63[(2, 0)]).atan2(t63[(0, 0)]);
                            (
                                rotation_angle / 2.0 - self.kinematics.theta_offset[3],
                                rotation_angle / 2.0 - self.kinematics.theta_offset[5],
                            )
                        };

                    let candidate = [theta1, theta2, theta3, theta4, theta5, theta6];
                    if let Some(limited) = self.kinematics.limit_joints(candidate, force_calculate)
                    {
                        theta4 = limited[3];
                        theta6 = limited[5];
                        solutions.push([
                            limited[0], limited[1], limited[2], theta4, limited[4], theta6,
                        ]);
                    }
                }
            }
        }

        if solutions.is_empty() {
            return Ok(solutions);
        }

        if let Some(reference) = reference {
            for solution in &mut solutions {
                if solution[4].abs() <= 1e-3 {
                    let limit4 = &self.kinematics.joint_limits[3];
                    let limit6 = &self.kinematics.joint_limits[5];
                    let mut rotation_angle = solution[3] + solution[5];
                    let min_angle = limit4.lower - reference[3] + limit6.lower - reference[5];
                    let max_angle = limit4.upper - reference[3] + limit6.upper - reference[5];

                    while rotation_angle < min_angle {
                        rotation_angle += TAU;
                    }
                    while rotation_angle > max_angle {
                        rotation_angle -= TAU;
                    }
                    loop {
                        if rotation_angle + TAU < max_angle
                            && (rotation_angle + TAU).abs() < rotation_angle.abs()
                        {
                            rotation_angle += TAU;
                        } else if rotation_angle - TAU > min_angle
                            && (rotation_angle - TAU).abs() < rotation_angle.abs()
                        {
                            rotation_angle -= TAU;
                        } else {
                            break;
                        }
                    }

                    if rotation_angle / 2.0 + reference[3] > limit4.upper {
                        solution[3] = limit4.upper;
                        solution[5] = reference[3] + reference[5] + rotation_angle - limit4.upper;
                    } else if rotation_angle / 2.0 + reference[3] < limit4.lower {
                        solution[3] = limit4.lower;
                        solution[5] = reference[3] + reference[5] + rotation_angle - limit4.lower;
                    } else if rotation_angle / 2.0 + reference[5] > limit6.upper {
                        solution[5] = limit6.upper;
                        solution[3] = reference[3] + reference[5] + rotation_angle - limit6.upper;
                    } else if rotation_angle / 2.0 + reference[5] < limit6.lower {
                        solution[5] = limit6.lower;
                        solution[3] = reference[3] + reference[5] + rotation_angle - limit6.lower;
                    } else {
                        solution[3] = rotation_angle / 2.0 + reference[3];
                        solution[5] = rotation_angle / 2.0 + reference[5];
                    }
                }
            }

            solutions.sort_by(|left, right| {
                let left_limit = self.kinematics.calculate_limit_punish(left);
                let right_limit = self.kinematics.calculate_limit_punish(right);
                if (left_limit - right_limit).abs() < 1e-6 {
                    return self
                        .kinematics
                        .calculate_bias_punish(left, reference)
                        .partial_cmp(&self.kinematics.calculate_bias_punish(right, reference))
                        .unwrap_or(Ordering::Equal);
                }
                left_limit
                    .partial_cmp(&right_limit)
                    .unwrap_or(Ordering::Equal)
            });
        } else {
            solutions.sort_by(|left, right| {
                self.kinematics
                    .calculate_limit_punish(left)
                    .partial_cmp(&self.kinematics.calculate_limit_punish(right))
                    .unwrap_or(Ordering::Equal)
            });
        }

        if use_clip {
            for solution in &mut solutions {
                self.kinematics.clip_joints(solution);
            }
        }

        Ok(solutions)
    }
}

impl KinematicsDynamicsBackend for PlayAnalyticalBackend {
    fn backend_name(&self) -> &'static str {
        "play_analytical"
    }

    fn dof(&self) -> usize {
        ARM_DOF
    }

    fn forward_kinematics(&self, joints: &[f64]) -> Result<Pose, ModelError> {
        let joints = self.joints_from_slice("joint vector", joints)?;
        matrix_to_pose(&self.forward_transform(&joints))
    }

    fn inverse_kinematics(
        &self,
        target: &Pose,
        seed: Option<&[f64]>,
    ) -> Result<Vec<f64>, ModelError> {
        let seed = match seed {
            Some(values) => Some(self.joints_from_slice("IK seed", values)?),
            None => None,
        };
        let (force_calculate, use_clip) = if seed.is_some() {
            (true, true)
        } else {
            (true, false)
        };

        self.solve_inverse_kinematics(target, seed.as_ref(), force_calculate, use_clip)?
            .into_iter()
            .next()
            .map(|solution| solution.to_vec())
            .ok_or_else(|| ModelError::Backend("analytical IK found no solution".to_owned()))
    }

    fn forward_dynamics(
        &self,
        joints: &[f64],
        vel: &[f64],
        torque: &[f64],
    ) -> Result<Vec<f64>, ModelError> {
        self.dynamics.forward_dynamics(joints, vel, torque)
    }

    fn inverse_dynamics(
        &self,
        joints: &[f64],
        vel: &[f64],
        acc: &[f64],
    ) -> Result<Vec<f64>, ModelError> {
        self.dynamics.inverse_dynamics(joints, vel, acc)
    }
}

impl AnalyticalParameters {
    fn for_mounted_eef(mounted_eef: &MountedEefType) -> Self {
        Self {
            alpha: [0.0, FRAC_PI_2, 0.0, -FRAC_PI_2, FRAC_PI_2, -FRAC_PI_2],
            a: [0.0, 0.0, 0.27009, 0.0, 0.0, 0.0],
            d: [0.1127, 0.0, 0.0, 0.29015, 0.0, 0.0],
            theta_offset: [0.0, 2.754863, 1.957526, -FRAC_PI_2, 0.0, 0.0],
            joint_limits: [
                JointLimits {
                    lower: -PI,
                    upper: 2.0944,
                },
                JointLimits {
                    lower: -2.9671,
                    upper: 0.17453,
                },
                JointLimits {
                    lower: -0.087266,
                    upper: PI,
                },
                JointLimits {
                    lower: -3.0107,
                    upper: 3.0107,
                },
                JointLimits {
                    lower: -1.7628,
                    upper: 1.7628,
                },
                JointLimits {
                    lower: -3.0107,
                    upper: 3.0107,
                },
            ],
            end_convert: end_convert_for_mounted_eef(mounted_eef),
            limit_punish: [1.0; ARM_DOF],
            bias_punish: [1.0; ARM_DOF],
            shoulder_singularity_threshold: 5e-4,
            wrist_singularity_threshold: 1e-3,
        }
    }

    fn adjacent_transform(&self, joint: f64, index: usize) -> Matrix4<f64> {
        let theta = joint + self.theta_offset[index];
        let alpha = self.alpha[index];
        Matrix4::new(
            cos_norm(theta),
            -sin_norm(theta),
            0.0,
            self.a[index],
            sin_norm(theta) * cos_norm(alpha),
            cos_norm(theta) * cos_norm(alpha),
            -sin_norm(alpha),
            -self.d[index] * sin_norm(alpha),
            sin_norm(theta) * sin_norm(alpha),
            cos_norm(theta) * sin_norm(alpha),
            cos_norm(alpha),
            self.d[index] * cos_norm(alpha),
            0.0,
            0.0,
            0.0,
            1.0,
        )
    }

    fn limit_joints(
        &self,
        joints: [f64; ARM_DOF],
        force_calculate: bool,
    ) -> Option<[f64; ARM_DOF]> {
        let mut limited = joints;
        for (index, value) in limited.iter_mut().enumerate() {
            let limits = &self.joint_limits[index];
            if *value >= limits.lower && *value <= limits.upper {
                continue;
            }

            if *value < limits.lower {
                let t1 = ((limits.lower - *value) / TAU).floor();
                let q_t1 = t1 * TAU + *value;
                let t2 = ((limits.lower - *value) / TAU).ceil();
                let q_t2 = t2 * TAU + *value;

                if q_t2 <= limits.upper {
                    *value = q_t2;
                } else if force_calculate {
                    *value = if limits.lower - q_t1 < q_t2 - limits.upper {
                        q_t1
                    } else {
                        q_t2
                    };
                } else {
                    return None;
                }
            } else {
                let t1 = ((*value - limits.upper) / TAU).floor();
                let q_t1 = *value - t1 * TAU;
                let t2 = ((*value - limits.upper) / TAU).ceil();
                let q_t2 = *value - t2 * TAU;

                if q_t2 >= limits.lower {
                    *value = q_t2;
                } else if force_calculate {
                    *value = if q_t1 - limits.upper < limits.lower - q_t2 {
                        q_t1
                    } else {
                        q_t2
                    };
                } else {
                    return None;
                }
            }
        }

        Some(limited)
    }

    fn calculate_limit_punish(&self, joints: &[f64; ARM_DOF]) -> f64 {
        joints
            .iter()
            .zip(self.joint_limits.iter())
            .zip(self.limit_punish.iter())
            .fold(0.0, |punish, ((joint, limits), weight)| {
                if *joint < limits.lower {
                    punish + (limits.lower - *joint) * *weight
                } else if *joint > limits.upper {
                    punish + (*joint - limits.upper) * *weight
                } else {
                    punish
                }
            })
    }

    fn calculate_bias_punish(&self, joints: &[f64; ARM_DOF], reference: &[f64; ARM_DOF]) -> f64 {
        joints
            .iter()
            .zip(reference.iter())
            .zip(self.bias_punish.iter())
            .map(|((joint, reference), weight)| (joint - reference).abs() * *weight)
            .sum()
    }

    fn clip_joints(&self, joints: &mut [f64; ARM_DOF]) {
        for (joint, limits) in joints.iter_mut().zip(self.joint_limits.iter()) {
            *joint = joint.clamp(limits.lower, limits.upper);
        }
    }
}

fn end_convert_for_mounted_eef(mounted_eef: &MountedEefType) -> Matrix4<f64> {
    match mounted_eef {
        MountedEefType::None => xyz_rpy_transform([0.0, 0.0, 0.0865], [FRAC_PI_2, -FRAC_PI_2, 0.0]),
        MountedEefType::E2B => {
            xyz_rpy_transform([0.0, 0.0, 0.1488995], [FRAC_PI_2, -FRAC_PI_2, 0.0])
        }
        MountedEefType::G2 => xyz_rpy_transform([0.0, 0.0, 0.2466], [FRAC_PI_2, -FRAC_PI_2, 0.0]),
        MountedEefType::Other(_) => {
            xyz_rpy_transform([0.0, 0.0, 0.0865], [FRAC_PI_2, -FRAC_PI_2, 0.0])
        }
    }
}

fn xyz_rpy_transform(xyz: [f64; 3], rpy: [f64; 3]) -> Matrix4<f64> {
    let rotation = Rotation3::from_euler_angles(rpy[0], rpy[1], rpy[2]);
    let mut transform = Matrix4::<f64>::identity();
    transform
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(rotation.matrix());
    transform[(0, 3)] = xyz[0];
    transform[(1, 3)] = xyz[1];
    transform[(2, 3)] = xyz[2];
    transform
}

fn matrix_to_pose(transform: &Matrix4<f64>) -> Result<Pose, ModelError> {
    let rotation =
        Rotation3::from_matrix_unchecked(transform.fixed_view::<3, 3>(0, 0).into_owned());
    let quaternion = UnitQuaternion::from_rotation_matrix(&rotation);
    Pose::from_components(
        [transform[(0, 3)], transform[(1, 3)], transform[(2, 3)]],
        [quaternion.i, quaternion.j, quaternion.k, quaternion.w],
    )
}

fn pose_to_matrix(pose: &Pose) -> Matrix4<f64> {
    let quaternion = UnitQuaternion::from_quaternion(Quaternion::new(
        pose.rotation_xyzw[3],
        pose.rotation_xyzw[0],
        pose.rotation_xyzw[1],
        pose.rotation_xyzw[2],
    ));
    let mut transform = Matrix4::<f64>::identity();
    transform
        .fixed_view_mut::<3, 3>(0, 0)
        .copy_from(quaternion.to_rotation_matrix().matrix());
    transform[(0, 3)] = pose.translation[0];
    transform[(1, 3)] = pose.translation[1];
    transform[(2, 3)] = pose.translation[2];
    transform
}

fn cos_norm(mut angle: f64) -> f64 {
    while angle < -PI {
        angle += TAU;
    }
    while angle > PI {
        angle -= TAU;
    }
    if angle < 0.0 {
        return cos_norm(-angle);
    }
    if angle > FRAC_PI_2 {
        return -cos_norm(PI - angle);
    }
    if angle.abs() < 1e-6 {
        return 1.0;
    }
    if (angle - FRAC_PI_2).abs() < 1e-6 {
        return 0.0;
    }
    angle.cos()
}

fn sin_norm(mut angle: f64) -> f64 {
    while angle < -PI {
        angle += TAU;
    }
    while angle > PI {
        angle -= TAU;
    }
    if angle < 0.0 {
        return -sin_norm(-angle);
    }
    if angle > FRAC_PI_2 {
        return sin_norm(PI - angle);
    }
    if angle.abs() < 1e-6 {
        return 0.0;
    }
    if (angle - FRAC_PI_2).abs() < 1e-6 {
        return 1.0;
    }
    angle.sin()
}
