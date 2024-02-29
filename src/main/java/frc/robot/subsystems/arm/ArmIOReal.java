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

  private static final double PIVOT_POS_SWITCH_THRESHOLD = 0.01;
  // Hardware
  private final TalonFX leaderTalon;
  private final TalonFX followerTalon;
  private final CANcoder absoluteEncoder;

  // Status Signals
  private final StatusSignal<Double> armInternalPositionRotations;
  private final StatusSignal<Double> armEncoderPositionRotations;
  private final StatusSignal<Double> armAbsolutePositionRotations;
  private final StatusSignal<Double> armVelocityRps;
  private final List<StatusSignal<Double>> armAppliedVoltage;
  private final List<StatusSignal<Double>> armOutputCurrent;
  private final List<StatusSignal<Double>> armTorqueCurrent;
  private final List<StatusSignal<Double>> armTempCelsius;

  PositionVoltage pPos = new PositionVoltage(0, 0, true, 0, 0, false, false, false);
  MotionMagicVoltage pMmPos = new MotionMagicVoltage(0, true, 0, 1, false, false, false);

  public ArmIOReal() {
    leaderTalon = new TalonFX(leaderID);
    followerTalon = new TalonFX(followerID);
    followerTalon.setControl(new Follower(leaderID, true));
    absoluteEncoder = new CANcoder(armEncoderID);
    //

    // Arm Encoder Configs
    CANcoderConfiguration armEncoderConfig = new CANcoderConfiguration();
    armEncoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    armEncoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    armEncoderConfig.MagnetSensor.MagnetOffset = armEncoderOffsetRotations;
    absoluteEncoder.getConfigurator().apply(armEncoderConfig, 1);

    // Leader motor configs
    TalonFXConfiguration armTalonConfig = new TalonFXConfiguration();
    armTalonConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    armTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    armTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armTalonConfig.Feedback.FeedbackRemoteSensorID = armEncoderID;
    armTalonConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    armTalonConfig.Feedback.SensorToMechanismRatio = 1.0;
    armTalonConfig.Feedback.RotorToSensorRatio = reduction;

    armTalonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;
    // posHold
    armTalonConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    armTalonConfig.Slot0.kG = 0.35;
    armTalonConfig.Slot0.kP = 254;
    armTalonConfig.Slot0.kI = 0;
    armTalonConfig.Slot0.kD = 0;
    armTalonConfig.Slot0.kS = 0;
    armTalonConfig.Slot0.kV = 0;
    armTalonConfig.Slot0.kA = 0;

    // mmPosMove
    armTalonConfig.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
    armTalonConfig.Slot1.kG = 0.35;
    armTalonConfig.Slot1.kP = 176;
    armTalonConfig.Slot1.kI = 0;
    armTalonConfig.Slot1.kD = 0;
    armTalonConfig.Slot1.kS = 0;
    armTalonConfig.Slot1.kV = 15;
    armTalonConfig.Slot1.kA = 0;

    // Set up armTalonConfig
    leaderTalon.getConfigurator().apply(armTalonConfig);

    // Follower configs
    armTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    followerTalon.getConfigurator().apply(armTalonConfig);

    // Status signals
    armInternalPositionRotations = leaderTalon.getPosition();
    armEncoderPositionRotations = absoluteEncoder.getPosition();
    armAbsolutePositionRotations = absoluteEncoder.getAbsolutePosition();
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

  public void setPosition(double positionRads) {
    if (Math.abs(positionRads - armEncoderPositionRotations.getValueAsDouble())
        < PIVOT_POS_SWITCH_THRESHOLD) {
      leaderTalon.setControl(pPos.withPosition(positionRads));
      followerTalon.setControl(pPos.withPosition(positionRads));
    } else {
      leaderTalon.setControl(pMmPos.withPosition(positionRads));
      followerTalon.setControl(pMmPos.withPosition(positionRads));
    }
  }

  @Override
  public void setBrakeMode(boolean enabled) {
    leaderTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    followerTalon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void stop() {
    leaderTalon.setControl(new NeutralOut());
    leaderTalon.setControl(new NeutralOut());
  }
}
