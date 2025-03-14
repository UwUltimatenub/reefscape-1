// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class LimeLight extends SubsystemBase {

  private final String limelightName = "limelight"; // Default name

  // private final AprilTagFieldLayout APRILTAGFIELDLAYOUT =
  // AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

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

  private final HashMap<Integer, Aprilposition> APRILTAG_RADIAN =
      new HashMap<Integer, Aprilposition>();

  public static class Aprilposition {
    public double rotation;
    public double leftx;
    public double lefty;
    public double rightx;
    public double righty;

    public Aprilposition(
        double rotation, double leftx, double lefty, double rightx, double righty) {
      this.rotation = rotation;
      this.leftx = leftx;
      this.lefty = lefty;
      this.rightx = rightx;
      this.righty = righty;
    }
  }

  public LimeLight() {
    APRILTAG_RADIAN.put(Integer.valueOf(7), new Aprilposition(0, 2.757, 4.283, 2.757, 4.283));
    APRILTAG_RADIAN.put(Integer.valueOf(8), new Aprilposition(0, 2.757, 4.283, 2.757, 4.283));
    APRILTAG_RADIAN.put(Integer.valueOf(9), new Aprilposition(0, 2.757, 4.283, 2.757, 4.283));
    APRILTAG_RADIAN.put(Integer.valueOf(10), new Aprilposition(0, 2.757, 4.283, 2.757, 4.283));
    APRILTAG_RADIAN.put(Integer.valueOf(11), new Aprilposition(0, 2.757, 4.283, 2.757, 4.283));
    APRILTAG_RADIAN.put(Integer.valueOf(6), new Aprilposition(0, 2.757, 4.283, 2.757, 4.283));

    APRILTAG_RADIAN.put(
        Integer.valueOf(18), new Aprilposition(Math.PI, 2.757, 4.283, 2.757, 4.283));
    APRILTAG_RADIAN.put(Integer.valueOf(17), new Aprilposition(0, 2.757, 4.283, 2.757, 4.283));
    APRILTAG_RADIAN.put(Integer.valueOf(22), new Aprilposition(0, 2.757, 4.283, 2.757, 4.283));
    APRILTAG_RADIAN.put(Integer.valueOf(21), new Aprilposition(0, 2.757, 4.283, 2.757, 4.283));
    APRILTAG_RADIAN.put(Integer.valueOf(20), new Aprilposition(0, 2.757, 4.283, 2.757, 4.283));
    APRILTAG_RADIAN.put(Integer.valueOf(19), new Aprilposition(0, 2.757, 4.283, 2.757, 4.283));

    LimelightHelpers.setCameraPose_RobotSpace(
        "",
        -0.33, // Forward offset (meters)
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
    return LimelightHelpers.getTV(limelightName)
        && APRILTAG_RADIAN.containsKey(Integer.valueOf(getTagID()));
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
    if (!APRILTAG_RADIAN.containsKey(Integer.valueOf(tagId))) {
      tagId = 0;
    }
    return tagId;
  }

  /** Get estimated robot pose from the Limelight */
  public Pose2d getRobotPose() {
    return LimelightHelpers.getBotPose2d_wpiBlue(limelightName);
  }

  public Pose2d getTargetPose2D(boolean isLeft) {
    Pose2d targetPose = null;
    limeLightInputs.tagId = getTagID();
    if (limeLightInputs.tagId != 0) {
      Aprilposition position = APRILTAG_RADIAN.get(limeLightInputs.tagId);
      if (isLeft) {
        targetPose =
            new Pose2d(position.leftx, position.lefty, Rotation2d.fromRadians(position.rotation));
      } else {
        targetPose =
            new Pose2d(position.rightx, position.righty, Rotation2d.fromRadians(position.rotation));
      }
      limeLightInputs.target_r = targetPose.getRotation().getDegrees();
      limeLightInputs.target_x = targetPose.getX();
      limeLightInputs.target_y = targetPose.getY();
    }
    limeLightInputs.current_r = getRobotPose().getRotation().getDegrees();
    limeLightInputs.current_x = getRobotPose().getX();
    limeLightInputs.current_y = getRobotPose().getY();

    return targetPose;
  }
}
