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
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  public static final double reduction = 18.689; // wrist gearbox gear ration 58/10*58/18
  public static final Rotation2d WRIST_OFFSET =
      Rotation2d.fromRotations(-0.0325); // default wrist angle; zero degree arm in horizontal
  private static final int encoderId = 13;
  public static final double minAngle = 0;
  public static final double maxAngle = 200;

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
      talon = new TalonFX(3, "*");
      wristEncoder = new CANcoder(encoderId, "*");

      // Configure  motor
      config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      config.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
      config.Feedback.RotorToSensorRatio = reduction;
      config.Feedback.FeedbackRemoteSensorID = encoderId;
      config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
      config.Feedback.RotorToSensorRatio = 60.0 * 60.0 * 30.0 / (10.0 * 18.0 * 12.0);
      config.TorqueCurrent.PeakForwardTorqueCurrent = 40.0;
      config.TorqueCurrent.PeakReverseTorqueCurrent = -40.0;
      config.CurrentLimits.StatorCurrentLimit = 40.0;
      config.CurrentLimits.StatorCurrentLimitEnable = true;
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
      talon.getConfigurator().apply(config);

      // Configure encoder
      var cancoderConfig = new CANcoderConfiguration();
      cancoderConfig.MagnetSensor.MagnetOffset = WRIST_OFFSET.getRotations();
      cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
      cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
      wristEncoder.getConfigurator().apply(cancoderConfig);

      // ParentDevice.optimizeBusUtilizationForAll(talon, wristEncoder);
    }

    public void periodic() {
      pivotInputs.encoderConnected = wristEncoder.isConnected();
      pivotInputs.motorConnected = talon.isConnected();
      pivotInputs.wristAngle = 360 * wristEncoder.getAbsolutePosition().getValueAsDouble();
      Logger.processInputs("Wrist", pivotInputs);
    }

    public void setVoltage(double voltage) {
      // Set the power to the main motor
       talon.setControl(new VoltageOut(voltage));
    }

    public void wristAngle(double targetDegrees) {
      // Get target position (in radians)
      targetDegrees =
          MathUtil.clamp(targetDegrees, minAngle, maxAngle);
      // double targetPosition = Math.toRadians(position);
      ArmFeedforward feedforward = new ArmFeedforward(0.0, 0.577, 0.0);
      talon.setControl(
          positionTorqueCurrentFOC
              .withPosition(targetDegrees/360)
              .withFeedForward(feedforward.calculate(Units.degreesToRadians(targetDegrees), 0)));
    }
  }
