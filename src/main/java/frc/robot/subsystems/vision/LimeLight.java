// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class LimeLight extends SubsystemBase {

  private final String limelightName = "limelight"; // Default name
  private static final double offset_side = -0.185;
  private static final double offset_forward = 0.42;
  private static final double offset_forward_level5 = 0.55;

  // private final AprilTagFieldLayout APRILTAGFIELDLAYOUT =
  public static final AprilTagFieldLayout TAG_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  // AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  // meter up from center.
  @AutoLog
  public static class LimeLightIOInputs {
    public int tagId = 0;

    public double current_r = 0;
    public double current_x = 0;
    public double current_y = 0;
    public double target_r = 0;
    public double target_x = 0;
    public double target_y = 0;
  }

  private LimeLightIOInputsAutoLogged limeLightInputs = new LimeLightIOInputsAutoLogged();

  public final HashMap<String, Pose2d> APRILTAG_TARGET_POSE = new HashMap<String, Pose2d>();

  private static final Transform2d transform_source =
      new Transform2d(0.22, 0, Rotation2d.fromDegrees(180));
  private static final Transform2d transform_algae =
      new Transform2d(0.55, 0, Rotation2d.fromDegrees(0));

  private static final Transform2d transform_left =
      new Transform2d(offset_forward, offset_side, Rotation2d.fromDegrees(0));
  private static final Transform2d transform_right =
      new Transform2d(offset_forward, -offset_side, Rotation2d.fromDegrees(0));

  private static final Transform2d transform_left_level5 =
      new Transform2d(offset_forward_level5, offset_side, Rotation2d.fromDegrees(0));
  private static final Transform2d transform_right_level5 =
      new Transform2d(offset_forward_level5, -offset_side, Rotation2d.fromDegrees(0));

  public LimeLight() {

    // source
    APRILTAG_TARGET_POSE.put(
        "1", TAG_LAYOUT.getTagPose(1).get().toPose2d().transformBy(transform_source));
    APRILTAG_TARGET_POSE.put(
        "2", TAG_LAYOUT.getTagPose(2).get().toPose2d().transformBy(transform_source));
    APRILTAG_TARGET_POSE.put(
        "12", TAG_LAYOUT.getTagPose(12).get().toPose2d().transformBy(transform_source));
    APRILTAG_TARGET_POSE.put(
        "13", TAG_LAYOUT.getTagPose(13).get().toPose2d().transformBy(transform_source));

    // algae level 2 3
    APRILTAG_TARGET_POSE.put(
        "6", TAG_LAYOUT.getTagPose(6).get().toPose2d().transformBy(transform_algae));
    APRILTAG_TARGET_POSE.put(
        "7", TAG_LAYOUT.getTagPose(7).get().toPose2d().transformBy(transform_algae));
    APRILTAG_TARGET_POSE.put(
        "8", TAG_LAYOUT.getTagPose(8).get().toPose2d().transformBy(transform_algae));
    APRILTAG_TARGET_POSE.put(
        "9", TAG_LAYOUT.getTagPose(9).get().toPose2d().transformBy(transform_algae));
    APRILTAG_TARGET_POSE.put(
        "10", TAG_LAYOUT.getTagPose(10).get().toPose2d().transformBy(transform_algae));
    APRILTAG_TARGET_POSE.put(
        "11", TAG_LAYOUT.getTagPose(11).get().toPose2d().transformBy(transform_algae));

    APRILTAG_TARGET_POSE.put(
        "17", TAG_LAYOUT.getTagPose(17).get().toPose2d().transformBy(transform_algae));
    APRILTAG_TARGET_POSE.put(
        "18", TAG_LAYOUT.getTagPose(18).get().toPose2d().transformBy(transform_algae));
    APRILTAG_TARGET_POSE.put(
        "19", TAG_LAYOUT.getTagPose(19).get().toPose2d().transformBy(transform_algae));
    APRILTAG_TARGET_POSE.put(
        "20", TAG_LAYOUT.getTagPose(20).get().toPose2d().transformBy(transform_algae));
    APRILTAG_TARGET_POSE.put(
        "21", TAG_LAYOUT.getTagPose(21).get().toPose2d().transformBy(transform_algae));
    APRILTAG_TARGET_POSE.put(
        "22", TAG_LAYOUT.getTagPose(22).get().toPose2d().transformBy(transform_algae));

    // level 2,3
    APRILTAG_TARGET_POSE.put(
        "6L", TAG_LAYOUT.getTagPose(6).get().toPose2d().transformBy(transform_left));
    APRILTAG_TARGET_POSE.put(
        "7L", TAG_LAYOUT.getTagPose(7).get().toPose2d().transformBy(transform_left));
    APRILTAG_TARGET_POSE.put(
        "8L", TAG_LAYOUT.getTagPose(8).get().toPose2d().transformBy(transform_left));
    APRILTAG_TARGET_POSE.put(
        "9L", TAG_LAYOUT.getTagPose(9).get().toPose2d().transformBy(transform_left));
    APRILTAG_TARGET_POSE.put(
        "10L", TAG_LAYOUT.getTagPose(10).get().toPose2d().transformBy(transform_left));
    APRILTAG_TARGET_POSE.put(
        "11L", TAG_LAYOUT.getTagPose(11).get().toPose2d().transformBy(transform_left));

    APRILTAG_TARGET_POSE.put(
        "17L", TAG_LAYOUT.getTagPose(17).get().toPose2d().transformBy(transform_left));
    APRILTAG_TARGET_POSE.put(
        "18L", TAG_LAYOUT.getTagPose(18).get().toPose2d().transformBy(transform_left));
    APRILTAG_TARGET_POSE.put(
        "19L", TAG_LAYOUT.getTagPose(19).get().toPose2d().transformBy(transform_left));
    APRILTAG_TARGET_POSE.put(
        "20L", TAG_LAYOUT.getTagPose(20).get().toPose2d().transformBy(transform_left));
    APRILTAG_TARGET_POSE.put(
        "21L", TAG_LAYOUT.getTagPose(21).get().toPose2d().transformBy(transform_left));
    APRILTAG_TARGET_POSE.put(
        "22L", TAG_LAYOUT.getTagPose(22).get().toPose2d().transformBy(transform_left));

    APRILTAG_TARGET_POSE.put(
        "6R", TAG_LAYOUT.getTagPose(6).get().toPose2d().transformBy(transform_right));
    APRILTAG_TARGET_POSE.put(
        "7R", TAG_LAYOUT.getTagPose(7).get().toPose2d().transformBy(transform_right));
    APRILTAG_TARGET_POSE.put(
        "8R", TAG_LAYOUT.getTagPose(8).get().toPose2d().transformBy(transform_right));
    APRILTAG_TARGET_POSE.put(
        "9R", TAG_LAYOUT.getTagPose(9).get().toPose2d().transformBy(transform_right));
    APRILTAG_TARGET_POSE.put(
        "10R", TAG_LAYOUT.getTagPose(10).get().toPose2d().transformBy(transform_right));
    APRILTAG_TARGET_POSE.put(
        "11R", TAG_LAYOUT.getTagPose(11).get().toPose2d().transformBy(transform_right));

    APRILTAG_TARGET_POSE.put(
        "17R", TAG_LAYOUT.getTagPose(17).get().toPose2d().transformBy(transform_right));
    APRILTAG_TARGET_POSE.put(
        "18R", TAG_LAYOUT.getTagPose(18).get().toPose2d().transformBy(transform_right));
    APRILTAG_TARGET_POSE.put(
        "19R", TAG_LAYOUT.getTagPose(19).get().toPose2d().transformBy(transform_right));
    APRILTAG_TARGET_POSE.put(
        "20R", TAG_LAYOUT.getTagPose(20).get().toPose2d().transformBy(transform_right));
    APRILTAG_TARGET_POSE.put(
        "21R", TAG_LAYOUT.getTagPose(21).get().toPose2d().transformBy(transform_right));
    APRILTAG_TARGET_POSE.put(
        "22R", TAG_LAYOUT.getTagPose(22).get().toPose2d().transformBy(transform_right));

    // Level 4
    APRILTAG_TARGET_POSE.put(
        "6L5", TAG_LAYOUT.getTagPose(6).get().toPose2d().transformBy(transform_left_level5));
    APRILTAG_TARGET_POSE.put(
        "7L5", TAG_LAYOUT.getTagPose(7).get().toPose2d().transformBy(transform_left_level5));
    APRILTAG_TARGET_POSE.put(
        "8L5", TAG_LAYOUT.getTagPose(8).get().toPose2d().transformBy(transform_left_level5));
    APRILTAG_TARGET_POSE.put(
        "9L5", TAG_LAYOUT.getTagPose(9).get().toPose2d().transformBy(transform_left_level5));
    APRILTAG_TARGET_POSE.put(
        "10L5", TAG_LAYOUT.getTagPose(10).get().toPose2d().transformBy(transform_left_level5));
    APRILTAG_TARGET_POSE.put(
        "11L5", TAG_LAYOUT.getTagPose(11).get().toPose2d().transformBy(transform_left_level5));

    APRILTAG_TARGET_POSE.put(
        "17L5", TAG_LAYOUT.getTagPose(17).get().toPose2d().transformBy(transform_left_level5));
    APRILTAG_TARGET_POSE.put(
        "18L5", TAG_LAYOUT.getTagPose(18).get().toPose2d().transformBy(transform_left_level5));
    APRILTAG_TARGET_POSE.put(
        "19L5", TAG_LAYOUT.getTagPose(19).get().toPose2d().transformBy(transform_left_level5));
    APRILTAG_TARGET_POSE.put(
        "20L5", TAG_LAYOUT.getTagPose(20).get().toPose2d().transformBy(transform_left_level5));
    APRILTAG_TARGET_POSE.put(
        "21L5", TAG_LAYOUT.getTagPose(21).get().toPose2d().transformBy(transform_left_level5));
    APRILTAG_TARGET_POSE.put(
        "22L5", TAG_LAYOUT.getTagPose(22).get().toPose2d().transformBy(transform_left_level5));

    APRILTAG_TARGET_POSE.put(
        "6R5", TAG_LAYOUT.getTagPose(6).get().toPose2d().transformBy(transform_right_level5));
    APRILTAG_TARGET_POSE.put(
        "7R5", TAG_LAYOUT.getTagPose(7).get().toPose2d().transformBy(transform_right_level5));
    APRILTAG_TARGET_POSE.put(
        "8R5", TAG_LAYOUT.getTagPose(8).get().toPose2d().transformBy(transform_right_level5));
    APRILTAG_TARGET_POSE.put(
        "9R5", TAG_LAYOUT.getTagPose(9).get().toPose2d().transformBy(transform_right_level5));
    APRILTAG_TARGET_POSE.put(
        "10R5", TAG_LAYOUT.getTagPose(10).get().toPose2d().transformBy(transform_right_level5));
    APRILTAG_TARGET_POSE.put(
        "11R5", TAG_LAYOUT.getTagPose(11).get().toPose2d().transformBy(transform_right_level5));

    APRILTAG_TARGET_POSE.put(
        "17R5", TAG_LAYOUT.getTagPose(17).get().toPose2d().transformBy(transform_right_level5));
    APRILTAG_TARGET_POSE.put(
        "18R5", TAG_LAYOUT.getTagPose(18).get().toPose2d().transformBy(transform_right_level5));
    APRILTAG_TARGET_POSE.put(
        "19R5", TAG_LAYOUT.getTagPose(19).get().toPose2d().transformBy(transform_right_level5));
    APRILTAG_TARGET_POSE.put(
        "20R5", TAG_LAYOUT.getTagPose(20).get().toPose2d().transformBy(transform_right_level5));
    APRILTAG_TARGET_POSE.put(
        "21R5", TAG_LAYOUT.getTagPose(21).get().toPose2d().transformBy(transform_right_level5));
    APRILTAG_TARGET_POSE.put(
        "22R5", TAG_LAYOUT.getTagPose(22).get().toPose2d().transformBy(transform_right_level5));

    LimelightHelpers.setCameraPose_RobotSpace(
        "",
        -0.30, // Forward offset (meters)
        0, // Side offset (meters)
        0.254, // Height offset (meters)
        0.0, // Roll (degrees)
        0.0, // Pitch (degrees)
        180 // Yaw (degrees)
        );
  }

  public void periodic() {

    Logger.processInputs("LimeLight", limeLightInputs);
  }

  /** Check if an AprilTag is detected */
  public boolean hasTarget() {
    return LimelightHelpers.getTV(limelightName);
  }

  /** Get horizontal offset (tx) from the crosshair */
  public double getTx() {
    return LimelightHelpers.getTX(limelightName);
  }

  /** Get vertical offset (ty) from the crosshair */
  public double getTy() {
    return LimelightHelpers.getTY(limelightName);
  }

  /** Get target area (ta) */
  public double getTa() {
    return LimelightHelpers.getTA(limelightName);
  }

  /** Get the AprilTag ID */
  public int getTagID() {
    int tagId = (int) LimelightHelpers.getFiducialID(limelightName);
    return tagId;
  }

  /** Get estimated robot pose from the Limelight */
  public Pose2d getRobotPose() {
    return LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
  }

  public Pose2d getTargetPose2D(boolean isLeft, boolean isLevel5, boolean isCoral) {
    Pose2d targetPose = null;
    limeLightInputs.tagId = getTagID();
    if (limeLightInputs.tagId != 0) {
      if (isCoral) {
        targetPose =
            APRILTAG_TARGET_POSE.get(
                limeLightInputs.tagId + (isLeft ? "L" : "R") + (isLevel5 ? "5" : ""));
      } else {
        targetPose = APRILTAG_TARGET_POSE.get(String.valueOf(limeLightInputs.tagId));
      }

      if (targetPose == null) return null;
      limeLightInputs.target_r = targetPose.getRotation().getDegrees();
      limeLightInputs.target_x = targetPose.getX();
      limeLightInputs.target_y = targetPose.getY();
      limeLightInputs.current_r = getRobotPose().getRotation().getDegrees();
      limeLightInputs.current_x = getRobotPose().getX();
      limeLightInputs.current_y = getRobotPose().getY();
    }

    return targetPose;
  }
}
