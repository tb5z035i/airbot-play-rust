use airbot_play_rust::model::{
    ModelBackendKind, ModelRegistry, MountedEefType, bundled_urdf_xml, gravity_coefficients_for_eef,
};

fn zeros() -> [f64; 6] {
    [0.0; 6]
}

#[test]
fn e2_urdf_xml_is_bundled_and_self_contained() {
    let xml = bundled_urdf_xml(&MountedEefType::E2B).expect("expected E2 URDF");
    assert!(xml.contains("<robot name=\"play_e2\">"));
    assert!(xml.contains("<joint name=\"joint6\" type=\"revolute\">"));
    assert!(xml.contains("<joint name=\"connect_eef_joint\" type=\"fixed\">"));
    assert!(!xml.contains("file:///"));
}

#[test]
fn play_urdf_is_bundled() {
    let xml = bundled_urdf_xml(&MountedEefType::None).expect("expected plain play URDF");
    assert!(xml.contains("<robot name=\"play\">"));
    assert!(xml.contains("<joint name=\"joint6\" type=\"revolute\">"));
    assert!(!xml.contains("file:///"));
}

#[test]
fn g2_urdf_is_bundled() {
    let xml = bundled_urdf_xml(&MountedEefType::G2).expect("expected G2 URDF");
    assert!(xml.contains("<robot name=\"play_g2\">"));
    assert!(xml.contains("<joint name=\"g2_joint\" type=\"prismatic\">"));
    assert!(!xml.contains("file:///"));
}

#[test]
fn gravity_coefficients_follow_eef_defaults() {
    assert_eq!(
        gravity_coefficients_for_eef(&MountedEefType::E2B),
        [0.6, 0.6, 0.6, 1.338, 1.236, 0.893]
    );
    assert_eq!(
        gravity_coefficients_for_eef(&MountedEefType::G2),
        [0.6, 0.6, 0.6, 1.303, 1.181, 1.5]
    );
}

#[test]
fn pinocchio_backend_loads_and_exercises_core_model_ops() {
    for mounted_eef in [MountedEefType::None, MountedEefType::E2B, MountedEefType::G2] {
        let backend = ModelRegistry::load(ModelBackendKind::Pinocchio, mounted_eef.clone())
            .expect("expected Pinocchio backend to load");

        assert_eq!(backend.backend_name(), "pinocchio");
        assert_eq!(backend.dof(), 6);

        let q = zeros();
        let v = zeros();
        let tau = backend
            .inverse_dynamics(&q, &v, &zeros())
            .expect("inverse dynamics should succeed");
        assert_eq!(tau.len(), 6);

        let pose = backend
            .forward_kinematics(&q)
            .expect("forward kinematics should succeed");
        assert_eq!(pose.as_vec().len(), 7);

        let q_ik = backend
            .inverse_kinematics(&pose, Some(&q))
            .expect("inverse kinematics should converge on the current pose");
        assert_eq!(q_ik.len(), 6);

        let ddq = backend
            .forward_dynamics(&q, &v, &tau)
            .expect("forward dynamics should succeed");
        assert_eq!(ddq.len(), 6);
    }
}
