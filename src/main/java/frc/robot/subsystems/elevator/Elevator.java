// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;

public class Elevator extends SubsystemBase {
  public static final double reduction = 5.0;

  // Hardware
  private final TalonFX talon;
  private final TalonFX followerTalon;

  @AutoLog
  public static class ElevatorIOInputs {
    public double motorCurrent = 0;
    public double motorVoltage = 0;
    public double motorAngle = 0;
  }

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  public static final double minHeight = 0;
  public static final double maxHeight = 60;

  public Elevator() {
    talon = new TalonFX(0, "*");
    followerTalon = new TalonFX(1, "*");
    followerTalon.setControl(new Follower(talon.getDeviceID(), false));

    // Configure motor
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
    config.Feedback.SensorToMechanismRatio = reduction;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 120.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -120.0;
    config.CurrentLimits.StatorCurrentLimit = 120.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    talon.getConfigurator().apply(config);

    ParentDevice.optimizeBusUtilizationForAll(talon);
  }

  public void setVoltage(double voltage) {
    // Set the power to the main motor
    talon.set(voltage);
  }

  // Periodic method called in every cycle (e.g., 20ms)
  @Override
  public void periodic() {
    //
  }

  public double getPosition() {
    // Get the position from the encoder
    return talon.getPosition().getValueAsDouble();
  }

  public double getVelocity() {
    // Get the velocity from the encoder
    return talon.getVelocity().getValueAsDouble();
  }

  public void resetPosition() {
    // Reset the encoder to the specified position
    talon.setPosition(0);
  }

  public void setPosition(double desireHeight) {
    // Get target position (in radians)
    double targetHeight = MathUtil.clamp(desireHeight, minHeight, maxHeight);
    // Feedforward Model (Tune These Values)
    ElevatorFeedforward feedforward = new ElevatorFeedforward(0.0, 0.8, 0, 0);
    talon.setControl(
        positionTorqueCurrentRequest
            .withPosition(targetHeight/4.7)//perimeter of pinion gear in centmeter
            .withFeedForward(feedforward.calculate(0)));
  }

  public void stop() {
    talon.stopMotor();
  }

  public void setBrakeMode(boolean enabled) {
    new Thread(
            () -> talon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast))
        .start();
  }
}
