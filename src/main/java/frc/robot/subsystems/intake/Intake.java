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
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  TalonFX intake;
  CANrange canRange;

  Debouncer db = new Debouncer(0.03, DebounceType.kRising);

  @AutoLog
  public static class IntakeIOInputs {
    public double coralRange = 0.0;
    public boolean isCoralLoaded = false;
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
    Logger.processInputs("Intake Sensor", inputs);
  }

  public void intake(double volts) {

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
    return db.calculate(
        canRange.getDistance().getValueAsDouble() < 0.1
            && canRange.getDistance().getValueAsDouble() != 0);
  }
}
