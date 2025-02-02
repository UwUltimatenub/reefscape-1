// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.AprilTagVisionIO.UnloggableAprilTagVisionIOInputs;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class AprilTagVision extends SubsystemBase {

  private AprilTagVisionIO io;
  private LoggableAprilTagVisionIOInputsAutoLogged loggableInputs =
      new LoggableAprilTagVisionIOInputsAutoLogged();
  private UnloggableAprilTagVisionIOInputs unloggableInputs =
      new UnloggableAprilTagVisionIOInputs();

  public AprilTagVision(AprilTagVisionIO io) {
    this.io = io;
  }

  public double autoRotate() {
    return io.autoRotate();
  }

  public double autoTranslateX() {
    return io.autoTranslateX();
  }

  public double autoTranslateY() {
    return io.autoTranslateY();
  }

  public boolean hasTarget() {
    return io.hasTarget();
  }

  public double getArea() {
    return io.getArea();
  }

  public void periodic() {
    updateInputs();
  }

  public void updateInputs() {
    io.updateInputs(loggableInputs, unloggableInputs);
  }

  public Optional<EstimatedRobotPose> getEstimatedPose() {
    return unloggableInputs.latestEstimatedPose;
  }

  public Transform3d getCamToTag() {
    return loggableInputs.latestCamToTagTranslation;
  }
}
