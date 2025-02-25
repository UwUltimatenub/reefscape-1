// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class LimeLight extends SubsystemBase {

  private final String limelightName = "limelight"; // Default name
  PIDController rotatePid = new PIDController(0.125, 0, 0);
  PIDController xPid = new PIDController(1, 0, 0.005);
  PIDController yPid = new PIDController(0.0605, 0, 0.0055);

  // private final AprilTagFieldLayout APRILTAGFIELDLAYOUT =
  //     AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

  //  AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

  // meter up from center.
  @AutoLog
  public static class LimeLightIOInputs {
    public double pitch = 0;
    public double yaw = 0;
    public double distance = 0;
  }

  private LimeLightIOInputsAutoLogged limeLightInputs = new LimeLightIOInputsAutoLogged();

  private final double CAMERA_HEIGHT = 0.28;
  private final double REEF_TARGET = 0.22;
  private final double CAMERA_PITCH = 0;

  public LimeLight() {}

  public void periodic() {
    limeLightInputs.pitch = getTy();
    limeLightInputs.yaw = getTx();
    limeLightInputs.distance = getDistance();
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
    return (int) LimelightHelpers.getFiducialID(limelightName);
  }

  /** Get estimated robot pose from the Limelight */
  public Pose3d getRobotPose() {
    return LimelightHelpers.getBotPose3d_wpiBlue(limelightName);
  }

  //
  public double getDistance() {
    double ty = getTy();
    double angleRadian = Units.degreesToRadians(CAMERA_PITCH + ty);

    if (Math.abs(angleRadian) < 1e-6) return Double.MAX_VALUE; // Prevent unreliable distances

    return (REEF_TARGET - CAMERA_HEIGHT) / Math.tan(angleRadian);
  }

  public double autoRotate() {

    return getTx();
  }

  public double autoTranslateX() {
    double correction =
        -xPid.calculate(getDistance() * Math.sin(Units.degreesToRadians(getTx())), 0);
    System.out.println("X: " + correction);
    if (hasTarget()) {
      return correction;
    } else {
      return 0;
    }
  }

  public double autoTranslateY() {
    double correction =
        -yPid.calculate(getDistance() * Math.cos(Units.degreesToRadians(getTx())), 10);
    System.out.println("Y: " + correction);
    if (hasTarget()) {
      return correction;
    } else {
      return 0;
    }
  }
}
