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
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.IntakeCommand;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  TalonFX intake;
  CANrange canRange;
  public int intakeStatus = 0;

  Debouncer db = new Debouncer(0.01, DebounceType.kRising);

  @AutoLog
  public static class IntakeIOInputs {
    public double coralRange = 0.0;
    public boolean isCoralLoaded = false;
    public int intakeStatus = 0;
  }

  IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake() {

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
    inputs.intakeStatus = intakeStatus;
    Logger.processInputs("Intake Sensor", inputs);
  }

  public void intake(int intakeStatus) {
    double volts = 0;
    this.intakeStatus = intakeStatus;
    switch (intakeStatus) {
      case IntakeCommand.Intake_Stopped:
        volts = 0;
        break;
      case IntakeCommand.Intake_Coral:
        volts = 5;
        break;
      case IntakeCommand.Eject_Coral:
        volts = 8;
        break;
      case IntakeCommand.Intake_Algae:
        volts = -6;
        break;
      case IntakeCommand.Eject_Algae:
        volts = 12;
        break;
    }
    if (volts == 0) {
      intake.stopMotor();
    } else {
      intake.setVoltage(volts);
    }
  }

  public boolean isCoralLoaded() {
    return db.calculate(canRange.getIsDetected().getValue());
  }
}
