// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.Logger;

public class Gyro {
  private GyroIO io;
  private GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

  public Gyro(GyroIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Gyro", inputs);
  }

  public void reset() {
    io.resetGyro(inputs);
  }

  public boolean isConnected() {
    return inputs.isConnected;
  }

  public Rotation2d[] getOdometryPositions() {
    return inputs.odometryYawPositions;
  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryYawTimestamps;
  }
}
