// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  class WristIOInputs {
    public boolean motorConnected = true;
    public boolean encoderConnected = false;

    public Rotation2d internalPosition = new Rotation2d();
    public Rotation2d encoderAbsolutePosition = new Rotation2d();
    public double encoderRelativePosition = 0.0;
    public double velocityRadPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  default void updateInputs(WristIOInputs inputs) {}

  default void runPosition(Rotation2d position) {}
}
