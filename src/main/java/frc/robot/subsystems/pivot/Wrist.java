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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;

public class Wrist extends SubsystemBase {
  public static final double reduction = 3.0;
  private static final Rotation2d offset = new Rotation2d();
  private static final int encoderId = 0;
  public static final Rotation2d minAngle = Rotation2d.fromDegrees(-140.0);
  public static final Rotation2d maxAngle = Rotation2d.fromDegrees(160.0);

  // Hardware
  private final TalonFX talon;
  private final CANcoder wristEncoder;
  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();
  private final PositionTorqueCurrentFOC positionTorqueCurrentFOC =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);

  @AutoLog
  public static class WristIOInputs {
    public boolean motorConnected = true;
    public boolean encoderConnected = false;
    public double wristAngle = 0.0;
  }

  private final WristIOInputsAutoLogged pivotInputs = new WristIOInputsAutoLogged();

  public Wrist() {
    talon = new TalonFX(0, "*");
    wristEncoder = new CANcoder(encoderId, "*");

    // Configure  motor
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
    config.Feedback.RotorToSensorRatio = reduction;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.FeedbackRemoteSensorID = encoderId;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    config.Feedback.SensorToMechanismRatio = 1.0;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    talon.getConfigurator().apply(config);

    // Configure encoder
    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = offset.getRotations();
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    wristEncoder.getConfigurator().apply(cancoderConfig);

    ParentDevice.optimizeBusUtilizationForAll(talon, wristEncoder);
  }

  public void periodic() {
    pivotInputs.wristAngle = wristEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public void wristAngle(double position) {
    double targetPosition = Math.toRadians(position);
    ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.577, 0.0);
    talon.setControl(
        positionTorqueCurrentFOC
            .withPosition(position)
            .withFeedForward(feedforward.calculate(position, targetPosition)));
  }
}
