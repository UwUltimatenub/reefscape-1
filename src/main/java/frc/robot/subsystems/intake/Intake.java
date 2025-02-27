// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SuperStructureState;
import frc.robot.subsystems.elevator.Elevator;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  TalonFX intake;
  CANrange canRange;

  @AutoLog
  public static class IntakeIOInputs {
    public double coralRange = 0.0;
    public boolean isCoralLoaded = false;
  }

  private Elevator elevator = null;
  IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(Elevator elevator1) {
    this.elevator = elevator1;
    // find actual motor IDs
    intake = new TalonFX(7, "*");
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake; // or NeutralModeValue.Coast
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // or CounterClockwise_Positive
    intake.getConfigurator().apply(config);

    canRange = new CANrange(19, "*");
  }

  @Override
  public void periodic() {
    inputs.coralRange = canRange.getDistance().getValueAsDouble();
    inputs.isCoralLoaded = isCoralLoaded();
    Logger.processInputs("Intake Sensor", inputs);
  }

  public void intake(boolean isIntake) {
    double volts = 0;
    if (elevator.currentState == SuperStructureState.STATE_SOURCE
        || elevator.currentState == SuperStructureState.STATE_L2
        || elevator.currentState == SuperStructureState.STATE_L3
        || elevator.currentState == SuperStructureState.STATE_L4) {
      if (isIntake) {
        volts = 8;
      } else {
        volts = 4;
      }
    } else {
      if (isIntake) {
        volts = -6;
      } else {
        volts = 12;
      }
    }
    intake.setVoltage(volts);
  }

  public void stop() {
    intake.stopMotor();
  }

  public boolean overloaded() {

    double currentDraw = intake.getStatorCurrent().getValueAsDouble(); // Stator current in amps
    double threshold = 20.0; // Adjust based on your setup
    return currentDraw > threshold;
  }

  public boolean isCoralLoaded() {

    return canRange.getDistance().getValueAsDouble() < 0.1
        && canRange.getDistance().getValueAsDouble() != 0; // meter
  }

  public boolean isDone() {
    boolean flag = false;
    if (elevator.currentState == SuperStructureState.STATE_SOURCE
        || elevator.currentState == SuperStructureState.STATE_L2
        || elevator.currentState == SuperStructureState.STATE_L3
        || elevator.currentState == SuperStructureState.STATE_L4) {
      flag = isCoralLoaded();
    } else {
      flag = overloaded();
    }
    return flag;
  }
}
