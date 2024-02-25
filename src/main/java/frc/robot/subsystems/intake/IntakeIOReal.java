package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;

public class IntakeIOReal implements IntakeIO {

  private TalonFX motor;

  private VoltageOut vout;

  public IntakeIOReal() {
    motor = new TalonFX(1);

    var config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 30;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;

    vout = new VoltageOut(0, false, false, false, false);

    motor.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {}

  @Override
  public void setVoltage(double volts) {
    double input = MathUtil.clamp(volts, -12.0, 12.0);
    motor.setControl(vout.withOutput(input));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
