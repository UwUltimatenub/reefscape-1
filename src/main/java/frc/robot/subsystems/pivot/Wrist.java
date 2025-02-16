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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;

public class Wrist extends SubsystemBase {
  public static final double reduction = 18.689; //wrist gearbox gear ration 58/10*58/18
  public static final Rotation2d WRIST_OFFSET = new Rotation2d(60);//default wrist angle; zero degree arm in horizontal
  private static final int encoderId = 0;
  public static final Rotation2d minAngle = Rotation2d.fromDegrees(60);
  public static final Rotation2d maxAngle = Rotation2d.fromDegrees(240.0);

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
    cancoderConfig.MagnetSensor.MagnetOffset = WRIST_OFFSET.getRotations();
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    wristEncoder.getConfigurator().apply(cancoderConfig);

    ParentDevice.optimizeBusUtilizationForAll(talon, wristEncoder);
  }

  public void periodic() {
    pivotInputs.wristAngle = wristEncoder.getAbsolutePosition().getValueAsDouble();
  }

  public void wristAngle(double targetDegrees) {
    // Get target position (in radians)
    double targetAngle =
        MathUtil.clamp(Math.toRadians(targetDegrees), minAngle.getRadians(), maxAngle.getRadians());
    // double targetPosition = Math.toRadians(position);
    ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.577, 0.0);
    talon.setControl(
        positionTorqueCurrentFOC
            .withPosition(targetAngle)
            .withFeedForward(feedforward.calculate(targetAngle, 0, 0)));
  }
}
