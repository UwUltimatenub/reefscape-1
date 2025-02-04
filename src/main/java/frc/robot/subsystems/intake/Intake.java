// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;

public class Intake extends SubsystemBase {
  TalonFX coralIntake;
  CANrange canRange;

  @AutoLog
  public static class IntakeIOInputs {
    public double coralRange = 0.0;
  }

  IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake() {
    // find actual motor IDs
    coralIntake = new TalonFX(0, "*");
    canRange = new CANrange(1, "*");
  }

  @Override
  public void periodic() {
    inputs.coralRange = canRange.getDistance().getValueAsDouble();
  }

  public void setCoralIntakeVoltage(double voltage) {
    coralIntake.setVoltage(voltage);
  }

  public void setAlgaeVoltage(double voltage) {
    coralIntake.setVoltage(voltage);
  }

  public boolean isCoralLoaded() {
    return canRange.getDistance().getValueAsDouble() < 10;
  }
}
