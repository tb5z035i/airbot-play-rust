#[cxx::bridge(namespace = "airbot::native")]
pub mod ffi {
    unsafe extern "C++" {
        include!("ffi/pinocchio_shim.hpp");

        type PinocchioModel;

        fn load_model(urdf_xml: &str, frame_name: &str) -> Result<UniquePtr<PinocchioModel>>;
        fn dof(self: &PinocchioModel) -> usize;
        fn forward_kinematics(self: &PinocchioModel, joints: &[f64]) -> Result<Vec<f64>>;
        fn inverse_kinematics(
            self: &PinocchioModel,
            target_pose: &[f64],
            seed: &[f64],
        ) -> Result<Vec<f64>>;
        fn forward_dynamics(
            self: &PinocchioModel,
            joints: &[f64],
            vel: &[f64],
            torque: &[f64],
        ) -> Result<Vec<f64>>;
        fn inverse_dynamics(
            self: &PinocchioModel,
            joints: &[f64],
            vel: &[f64],
            acc: &[f64],
        ) -> Result<Vec<f64>>;
    }
}
