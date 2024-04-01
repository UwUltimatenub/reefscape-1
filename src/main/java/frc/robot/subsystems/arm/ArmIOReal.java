// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.*;
import edu.wpi.first.math.util.Units;
import java.util.List;

public class ArmIOReal implements ArmIO {

  public static final int ARM_LEADER_ID = 15;
  public static final int ARM_FOLLOWER_ID = 21;
  public static final int ARM_ENCODER_ID = 14;
  // Hardware
  private final TalonFX leaderTalon;
  private final TalonFX followerTalon;
  private final CANcoder armCANEncoder;

  // Status Signals
  private final StatusSignal<Double> armInternalPositionRotations;
  private final StatusSignal<Double> armEncoderPositionRotations;
  private final StatusSignal<Double> armAbsolutePositionRotations;
  private final StatusSignal<Double> armVelocityRps;
  private final List<StatusSignal<Double>> armAppliedVoltage;
  private final List<StatusSignal<Double>> armOutputCurrent;
  private final List<StatusSignal<Double>> armTorqueCurrent;
  private final List<StatusSignal<Double>> armTempCelsius;

  PositionVoltage pPos = new PositionVoltage(0, 0, false, 0, 0, false, false, false);
  MotionMagicVoltage pMmPos = new MotionMagicVoltage(0, false, 0, 1, false, false, false);
  /** The offset of the arm encoder in rotations. */
  // public static double armEncoderOffsetRads = -2.538 + Arm.INITIAL_ARM_RADS; // -2.854;
  public static double armEncoderOffsetRads = 2.605;

  public static double armEncoderOffsetRotations = Units.radiansToRotations(armEncoderOffsetRads);

  public ArmIOReal() {
    leaderTalon = new TalonFX(ARM_LEADER_ID);
    followerTalon = new TalonFX(ARM_FOLLOWER_ID);
    followerTalon.setControl(new Follower(ARM_LEADER_ID, true));
    armCANEncoder = new CANcoder(ARM_ENCODER_ID);
    //

    // Arm Encoder Configs
    CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
    armEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    armEncoderConfig.MagnetSensor.MagnetOffset = -armEncoderOffsetRotations;
    armCANEncoder.getConfigurator().apply(armEncoderConfig, 1);

    // Arm motor configs
    TalonFXConfiguration armTalonConfig = new TalonFXConfiguration();
    armTalonConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
    armTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armTalonConfig.Feedback.FeedbackRemoteSensorID = ARM_ENCODER_ID;
    armTalonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    armTalonConfig.Feedback.SensorToMechanismRatio = 1.0;
    armTalonConfig.Feedback.RotorToSensorRatio = reduction;

    armTalonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;
    // Hold the ARM
    armTalonConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    armTalonConfig.Slot0.kG = 0.35; // to hold the arm weight
    armTalonConfig.Slot0.kP = 100; // 100; // adjust PID
    armTalonConfig.Slot0.kI = 0;
    armTalonConfig.Slot0.kD = 0.015;
    armTalonConfig.Slot0.kS = 0;
    armTalonConfig.Slot0.kV = 0;
    armTalonConfig.Slot0.kA = 0;

    // Move the arm
    armTalonConfig.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
    armTalonConfig.Slot1.kG = 0.35; // to hold the arm weight
    armTalonConfig.Slot1.kP = 100; // 100; // adjust PID
    armTalonConfig.Slot1.kI = 0;
    armTalonConfig.Slot1.kD = 0;
    armTalonConfig.Slot1.kS = 0;
    armTalonConfig.Slot1.kV = 8; // 8.3; // move velocity
    armTalonConfig.Slot1.kA = 0.2; // 0.2; // move accerleration

    armTalonConfig.MotionMagic.MotionMagicCruiseVelocity = 1.0; // 0.5;
    armTalonConfig.MotionMagic.MotionMagicAcceleration = 2; // 1.0;
    armTalonConfig.MotionMagic.MotionMagicJerk = 10; // 10;

    // Set up armTalonConfig
    leaderTalon.getConfigurator().apply(armTalonConfig);

    // Follower configs
    armTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    followerTalon.getConfigurator().apply(armTalonConfig);

    // Status signals
    armInternalPositionRotations = leaderTalon.getPosition();
    armEncoderPositionRotations = armCANEncoder.getPosition();
    armAbsolutePositionRotations = armCANEncoder.getAbsolutePosition();
    armVelocityRps = leaderTalon.getVelocity();
    armAppliedVoltage = List.of(leaderTalon.getMotorVoltage(), followerTalon.getMotorVoltage());
    armOutputCurrent = List.of(leaderTalon.getSupplyCurrent(), followerTalon.getSupplyCurrent());
    armTorqueCurrent = List.of(leaderTalon.getTorqueCurrent(), followerTalon.getTorqueCurrent());
    armTempCelsius = List.of(leaderTalon.getDeviceTemp(), followerTalon.getDeviceTemp());

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        armInternalPositionRotations,
        armEncoderPositionRotations,
        armAbsolutePositionRotations,
        armVelocityRps,
        armAppliedVoltage.get(0),
        armAppliedVoltage.get(1),
        armOutputCurrent.get(0),
        armOutputCurrent.get(1),
        armTorqueCurrent.get(0),
        armTorqueCurrent.get(1),
        armTempCelsius.get(0),
        armTempCelsius.get(1));

    // Optimize bus utilization
    leaderTalon.optimizeBusUtilization(1.0);
    followerTalon.optimizeBusUtilization(1.0);
  }

  public void updateInputs(ArmIOInputs inputs) {
    inputs.leaderMotorConnected =
        BaseStatusSignal.refreshAll(
                armInternalPositionRotations,
                armVelocityRps,
                armAppliedVoltage.get(0),
                armOutputCurrent.get(0),
                armTorqueCurrent.get(0),
                armTempCelsius.get(0))
            .isOK();
    inputs.followerMotorConnected =
        BaseStatusSignal.refreshAll(
                armAppliedVoltage.get(1),
                armOutputCurrent.get(1),
                armTorqueCurrent.get(1),
                armTempCelsius.get(1))
            .isOK();
    inputs.absoluteEncoderConnected =
        BaseStatusSignal.refreshAll(armEncoderPositionRotations, armAbsolutePositionRotations)
            .isOK();

    inputs.armPositionRads = Units.rotationsToRadians(armInternalPositionRotations.getValue());
    inputs.armEncoderPositionRads =
        Units.rotationsToRadians(armEncoderPositionRotations.getValue());
    inputs.armAbsoluteEncoderPositionRads =
        Units.rotationsToRadians(armAbsolutePositionRotations.getValue());
    inputs.armVelocityRadsPerSec = Units.rotationsToRadians(armVelocityRps.getValue());
    inputs.armAppliedVolts =
        armAppliedVoltage.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.armCurrentAmps =
        armOutputCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.armTorqueCurrentAmps =
        armTorqueCurrent.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    inputs.armTempCelcius =
        armTempCelsius.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
  }

  // @Override
  // public void setPosition(double positionRads) {
  //   leaderTalon.setPosition(Units.radiansToRotations(positionRads));
  // }

  @Override
  public void setBrakeMode(boolean enabled) {
    leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    followerTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void stop() {
    leaderTalon.setControl(new NeutralOut());
    followerTalon.setControl(new NeutralOut());
  }

  @Override
  public void setPositionControl(double positionRotations) {
    leaderTalon.setControl(pPos.withPosition(positionRotations));
    followerTalon.setControl(pPos.withPosition(positionRotations));
  }

  @Override
  public void setMotionControl(double positionRotations) {
    leaderTalon.setControl(pMmPos.withPosition(positionRotations));
    followerTalon.setControl(pMmPos.withPosition(positionRotations));
  }
}
