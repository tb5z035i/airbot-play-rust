use airbot_play_rust::model::{
    ModelBackendKind, ModelRegistry, MountedEefType, Pose, bundled_urdf_xml,
    gravity_coefficients_for_eef,
};

fn zeros() -> [f64; 6] {
    [0.0; 6]
}

fn sample_joint_sets() -> [[f64; 6]; 3] {
    [
        zeros(),
        [0.25, -0.8, 1.1, -0.45, 0.35, -0.9],
        [-1.0, -0.4, 0.7, 1.2, -0.5, 0.8],
    ]
}

fn roundtrip_joint_sets() -> [[f64; 6]; 2] {
    [
        [0.25, -0.8, 1.1, -0.45, 0.35, -0.9],
        [-1.0, -0.4, 0.7, 1.2, -0.5, 0.8],
    ]
}

fn backend_kinds() -> [ModelBackendKind; 2] {
    [
        ModelBackendKind::Pinocchio,
        ModelBackendKind::PlayAnalytical,
    ]
}

fn mounted_eef_variants() -> [MountedEefType; 3] {
    [
        MountedEefType::None,
        MountedEefType::E2B,
        MountedEefType::G2,
    ]
}

fn quaternion_distance(left: [f64; 4], right: [f64; 4]) -> f64 {
    let same_sign = left
        .iter()
        .zip(right.iter())
        .map(|(left, right)| (left - right).powi(2))
        .sum::<f64>()
        .sqrt();
    let opposite_sign = left
        .iter()
        .zip(right.iter())
        .map(|(left, right)| (left + right).powi(2))
        .sum::<f64>()
        .sqrt();
    same_sign.min(opposite_sign)
}

fn assert_pose_close(actual: &Pose, expected: &Pose, position_tol: f64, rotation_tol: f64) {
    for (actual, expected) in actual.translation.iter().zip(expected.translation.iter()) {
        assert!(
            (actual - expected).abs() <= position_tol,
            "expected translation {expected}, got {actual}"
        );
    }

    let rotation_delta = quaternion_distance(actual.rotation_xyzw, expected.rotation_xyzw);
    assert!(
        rotation_delta <= rotation_tol,
        "expected rotation {:?}, got {:?} (delta {rotation_delta})",
        expected.rotation_xyzw,
        actual.rotation_xyzw
    );
}

fn assert_seeded_roundtrip(
    backend_kind: ModelBackendKind,
    mounted_eef: MountedEefType,
    joints: [f64; 6],
    position_tol: f64,
    rotation_tol: f64,
) {
    let backend = ModelRegistry::load(backend_kind, mounted_eef).expect("expected backend to load");
    let target_pose = backend
        .forward_kinematics(&joints)
        .expect("FK should succeed for roundtrip target");
    let recovered = backend
        .inverse_kinematics(&target_pose, Some(&joints))
        .expect("IK should recover a seeded solution");
    let recovered_pose = backend
        .forward_kinematics(&recovered)
        .expect("FK should succeed for recovered joints");
    assert_pose_close(&recovered_pose, &target_pose, position_tol, rotation_tol);
}

fn assert_cross_solver_pose_transfer(
    source_kind: ModelBackendKind,
    target_kind: ModelBackendKind,
    mounted_eef: MountedEefType,
    joints: [f64; 6],
    position_tol: f64,
    rotation_tol: f64,
) {
    let source =
        ModelRegistry::load(source_kind, mounted_eef.clone()).expect("expected source backend");
    let target = ModelRegistry::load(target_kind, mounted_eef).expect("expected target backend");

    let source_pose = source
        .forward_kinematics(&joints)
        .expect("source FK should succeed");
    let recovered = target
        .inverse_kinematics(&source_pose, Some(&joints))
        .expect("target IK should recover a seeded cross-solver solution");
    let recovered_pose = target
        .forward_kinematics(&recovered)
        .expect("target FK should succeed for recovered joints");
    assert_pose_close(&recovered_pose, &source_pose, position_tol, rotation_tol);
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
fn both_model_backends_load_and_exercise_core_model_ops() {
    for backend_kind in backend_kinds() {
        for mounted_eef in mounted_eef_variants() {
            let backend = ModelRegistry::load(backend_kind, mounted_eef.clone())
                .expect("expected backend to load");

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
}

#[test]
fn analytical_fk_tracks_pinocchio_across_bundled_models() {
    for mounted_eef in mounted_eef_variants() {
        let pinocchio = ModelRegistry::load(ModelBackendKind::Pinocchio, mounted_eef.clone())
            .expect("expected Pinocchio backend to load");
        let analytical = ModelRegistry::load(ModelBackendKind::PlayAnalytical, mounted_eef.clone())
            .expect("expected analytical backend to load");

        for joints in sample_joint_sets() {
            let pinocchio_pose = pinocchio
                .forward_kinematics(&joints)
                .expect("Pinocchio FK should succeed");
            let analytical_pose = analytical
                .forward_kinematics(&joints)
                .expect("analytical FK should succeed");
            assert_pose_close(&analytical_pose, &pinocchio_pose, 2e-3, 2e-3);
        }
    }
}

#[test]
fn both_solvers_seeded_roundtrip_fk_pose() {
    for backend_kind in backend_kinds() {
        for mounted_eef in mounted_eef_variants() {
            for joints in roundtrip_joint_sets() {
                assert_seeded_roundtrip(backend_kind, mounted_eef.clone(), joints, 1e-4, 1e-4);
            }
        }
    }
}

#[test]
fn pinocchio_and_analytical_cross_solve_each_other_fk_pose() {
    for mounted_eef in mounted_eef_variants() {
        for joints in roundtrip_joint_sets() {
            assert_cross_solver_pose_transfer(
                ModelBackendKind::Pinocchio,
                ModelBackendKind::PlayAnalytical,
                mounted_eef.clone(),
                joints,
                2e-3,
                2e-3,
            );
            assert_cross_solver_pose_transfer(
                ModelBackendKind::PlayAnalytical,
                ModelBackendKind::Pinocchio,
                mounted_eef.clone(),
                joints,
                2e-3,
                2e-3,
            );
        }
    }
}

#[test]
fn model_backend_kind_roundtrips_over_serde() {
    let json =
        serde_json::to_string(&ModelBackendKind::PlayAnalytical).expect("enum should serialize");
    assert_eq!(json, "\"play_analytical\"");

    let parsed: ModelBackendKind =
        serde_json::from_str(&json).expect("enum should deserialize from public value");
    assert_eq!(parsed, ModelBackendKind::PlayAnalytical);
}

#[test]
fn model_backend_kind_defaults_to_analytical() {
    assert_eq!(
        ModelBackendKind::default(),
        ModelBackendKind::PlayAnalytical
    );
}
