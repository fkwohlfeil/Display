# intrinsic_toppra_wrapper -- eigener Wrapper um TOPP-RA aus Intrinsic Core.
#
# Eigener Code: intrinsic_toppra_wrapper.cc und json_klein.h (siehe
# Kopfkommentare dort).
# Aus Intrinsic Core 20260922.0 (Apache 2.0) nur als Abhaengigkeiten unten,
# unveraendert.
#
# Liegt im Repo unter ~/intrinsic-core/intrinsic_toppra_wrapper/.
#   bazel build //intrinsic_toppra_wrapper:intrinsic_toppra_wrapper
#   bazel-bin/intrinsic_toppra_wrapper/intrinsic_toppra_wrapper --pruefen

load("//bazel:cc_macros.bzl", "cc_binary")

cc_binary(
    name = "intrinsic_toppra_wrapper",
    srcs = [
        "intrinsic_toppra_wrapper.cc",
        "json_klein.h",  # eigener JSON-Leser/-Schreiber
    ],
    deps = [
        # Intrinsic Core (unveraendert)
        "//intrinsic_kinematics/intrinsic/kinematics/types:joint_limits",
        "//intrinsic_kinematics/intrinsic/kinematics/types:joint_state",
        "//intrinsic_kinematics/intrinsic/kinematics/types:joint_trajectories",
        "//intrinsic_motion_planning/intrinsic/motion_planning/path_planning:path_segment",
        "//intrinsic_motion_planning/intrinsic/motion_planning/path_planning/parametric_curves:cartesian_parametric_curve",
        "//intrinsic_motion_planning/intrinsic/motion_planning/path_planning/parametric_curves:cartesian_parametric_curve_utils",
        "//intrinsic_motion_planning/intrinsic/motion_planning/path_planning/parametric_curves:cartesian_polyline_bsplines_curve",
        "//intrinsic_motion_planning/intrinsic/motion_planning/path_planning/parametric_curves:parametric_curve_utils",
        "//intrinsic_motion_planning/intrinsic/motion_planning/path_planning/planners:joint_sampling_utils",
        "//intrinsic_motion_planning/intrinsic/motion_planning/trajectory_planning:interpolate_joint_trajectories",
        "//intrinsic_motion_planning/intrinsic/motion_planning/trajectory_planning:optimized_toppra",
        "//intrinsic_motion_planning/intrinsic/motion_planning/trajectory_planning/path_refinement:spline_based_path_refinement",
        "//intrinsic_motion_planning/intrinsic/motion_planning/trajectory_planning/topp:joint_optimization_options_cc_proto",
        "//intrinsic_motion_planning/intrinsic/motion_planning/trajectory_planning/topp:path_sample",
        "//intrinsic_motion_planning/intrinsic/motion_planning/trajectory_planning/topp:refine_path_and_compute_acceleration_limited_trajectory",
        "//intrinsic_motion_planning/intrinsic/motion_planning/trajectory_planning/topp:topp_options",
        "//intrinsic_sdk/intrinsic/eigenmath",
        "//intrinsic_sdk/intrinsic/math:pose3",
        # Fremdbibliotheken, die Intrinsic Core mitbringt
        "@abseil-cpp//absl/status",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
        "@abseil-cpp//absl/time",
        "@abseil-cpp//absl/types:span",
    ],
)
