// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOTalonFX implements IntakeIO {
  TalonFX coralIntake;
  CANrange canRange;

  public IntakeIOTalonFX() {
    // find actual motor IDs
    coralIntake = new TalonFX(0, "*");
    canRange = new CANrange(1, "*");
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.coralWristPosition = coralIntake.getPosition().getValueAsDouble();
    inputs.coralRange = canRange.getDistance().getValueAsDouble();
  }

  @Override
  public void setIntakeVoltage(double voltage) {
    coralIntake.setVoltage(voltage);
  }

  @Override
  public double getCanRange() {
    return canRange.getDistance().getValueAsDouble();
  }
}
