// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  IntakeIO io;
  IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  public void setCoralIntakeVoltage(double voltage) {
    io.setIntakeVoltage(voltage);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void setAlgaeVoltage(double voltage) {
    io.setIntakeVoltage(voltage);
  }

  public boolean isCoralLoaded() {
    return io.getCanRange() < 10;
  }
}
