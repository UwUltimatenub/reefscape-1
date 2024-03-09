package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class IntakeIOReal implements IntakeIO {

  private TalonFX motor;

  // rotations per second.
  VelocityVoltage velOut =
      new VelocityVoltage(Units.radiansToRotations(0.0), 0.0, true, 0, 0, false, false, false);

  public IntakeIOReal() {
    motor = new TalonFX(8);

    var config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 30;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;

    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {

    inputs.velocityRadPerSec = Units.rotationsToRadians(motor.getVelocity().getValueAsDouble());
    inputs.appliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.currentAmps = motor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    System.out.println("Vel : " + velocityRadPerSec);
    motor.setControl(velOut.withVelocity(velocityRadPerSec * 1.1)); // rotations per second.
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
