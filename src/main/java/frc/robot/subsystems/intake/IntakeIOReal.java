package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class IntakeIOReal implements IntakeIO {

  private TalonFX motor;

  // rotations per second.
  VelocityVoltage velOut =
      new VelocityVoltage(Units.radiansToRotations(0.0));

  public IntakeIOReal() {
    motor = new TalonFX(16);

    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 50.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.kP = 0.04;
    config.Slot0.kD = 0; // 0.05;
    config.Slot0.kV = .12; // 10;
    config.Slot0.kS = 0; // 0.33329;

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
    motor.setControl(velOut.withVelocity(velocityRadPerSec)); // rotations per second.
  }

  @Override
  public void stop() {
    motor.setVoltage(-1);
    Timer.delay(0.05);
    motor.stopMotor();
  }
}
