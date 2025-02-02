// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

public class WristIOTalonFX implements WristIO {
  public static final double reduction = 3.0;
  private static final Rotation2d offset = new Rotation2d();
  private static final int encoderId = 0;

  // Hardware
  private final TalonFX talon;
  private final CANcoder encoder;

  // Config
  private final TalonFXConfiguration Config = new TalonFXConfiguration();

  private final PositionTorqueCurrentFOC positionTorqueCurrentFOC =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  public WristIOTalonFX() {
    talon = new TalonFX(0, "*");
    encoder = new CANcoder(encoderId, "*");

    // Configure  motor
    Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    Config.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
    Config.Feedback.RotorToSensorRatio = reduction;
    Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    Config.Feedback.FeedbackRemoteSensorID = encoderId;
    Config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    Config.Feedback.SensorToMechanismRatio = 1.0;
    Config.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    Config.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
    Config.CurrentLimits.StatorCurrentLimit = 40.0;
    Config.CurrentLimits.StatorCurrentLimitEnable = true;
    Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    talon.getConfigurator().apply(Config);

    // Configure encoder
    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = offset.getRotations();
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    encoder.getConfigurator().apply(cancoderConfig);

    ParentDevice.optimizeBusUtilizationForAll(talon, encoder);
  }

  public void setWristPositionDegrees(double position) {
    double targetPosition = Math.toRadians(position);
    ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.577, 0.0);
    talon.setControl(
        positionTorqueCurrentFOC
            .withPosition(position)
            .withFeedForward(feedforward.calculate(position, targetPosition)));
  }
}
