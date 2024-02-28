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
    TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
    leaderConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    leaderConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leaderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leaderConfig.Feedback.FeedbackRemoteSensorID = armEncoderID;
    leaderConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.SyncCANcoder;
    leaderConfig.Feedback.SensorToMechanismRatio = 1.0;
    leaderConfig.Feedback.RotorToSensorRatio = reduction;

    leaderConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;
    // posHold
    leaderConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    leaderConfig.Slot0.kG = 0.35;
    leaderConfig.Slot0.kP = 254;
    leaderConfig.Slot0.kI = 0;
    leaderConfig.Slot0.kD = 0;
    leaderConfig.Slot0.kS = 0;
    leaderConfig.Slot0.kV = 0;
    leaderConfig.Slot0.kA = 0;

    // mmPosMove
    leaderConfig.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
    leaderConfig.Slot1.kG = 0.35;
    leaderConfig.Slot1.kP = 176;
    leaderConfig.Slot1.kI = 0;
    leaderConfig.Slot1.kD = 0;
    leaderConfig.Slot1.kS = 0;
    leaderConfig.Slot1.kV = 15;
    leaderConfig.Slot1.kA = 0;

    // Set up leaderConfig
    leaderTalon.getConfigurator().apply(leaderConfig);

    // Follower configs
    leaderConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    followerTalon.getConfigurator().apply(leaderConfig);

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
      leaderTalon.setControl(pMmPos.withPosition(positionRads));
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
  }
}
