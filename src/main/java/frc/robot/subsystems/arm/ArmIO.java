// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  class ArmIOInputs {
    public boolean leaderMotorConnected = true;
    public boolean followerMotorConnected = true;

    public double armPositionRads = 0.0;
    public double armEncoderPositionRads = 0.0;
    public double armAbsoluteEncoderPositionRads = 0.0;
    public double armVelocityRadsPerSec = 0.0;
    public double[] armAppliedVolts = new double[] {};
    public double[] armCurrentAmps = new double[] {};
    public double[] armTorqueCurrentAmps = new double[] {};
    public double[] armTempCelcius = new double[] {};
    public boolean absoluteEncoderConnected = true;
  }

  default void updateInputs(ArmIOInputs inputs) {}

  /** Set brake mode enabled */
  default void setBrakeMode(boolean enabled) {}

  /** Stops motors */
  default void stop() {}

  void setPositionControl(double positionRotations);

  void setMotionControl(double positionRotations);
}
