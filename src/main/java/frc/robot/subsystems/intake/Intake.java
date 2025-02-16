// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;

public class Intake extends SubsystemBase {
  TalonFX intake;
  CANrange canRange;

  @AutoLog
  public static class IntakeIOInputs {
    public double coralRange = 0.0;
  }

  IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake() {
    // find actual motor IDs
    intake = new TalonFX(0, "*");
    intake.setNeutralMode(NeutralModeValue.Brake); //stop motor quickly
    canRange = new CANrange(1, "*");
  }

  @Override
  public void periodic() {
    inputs.coralRange = canRange.getDistance().getValueAsDouble();
  }

  public void intakeCoral() {
    intake.setVoltage(10);
  }
  public void ejectCoral() {
    intake.setVoltage(-10);
  }
  public void intakeAlgae() {
    intake.setVoltage(10);
  }
  public void ejectAlgae() {
    intake.setVoltage(-10);
  }
  public void stop() {
    intake.stopMotor();
  }
  public boolean overloaded() {

    double currentDraw = intake.getStatorCurrent().getValueAsDouble(); // Stator current in amps
    double threshold = 30.0; // Adjust based on your setup
    return currentDraw > threshold;
  }
  public boolean isCoralLoaded() {
    return canRange.getDistance().getValueAsDouble() < 0.1;//meter
  }
}
