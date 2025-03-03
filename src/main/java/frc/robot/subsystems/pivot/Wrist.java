// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivot;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SuperStructureState;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {

  // Hardware
  private final TalonFX talon;
  private final CANcoder wristEncoder;

  PositionVoltage pPos = new PositionVoltage(0);
  MotionMagicVoltage pMmPos = new MotionMagicVoltage(0);

  public static final double reduction =
      50; // wrist gearbox gear ration 60.0 * 60.0 * 30.0 / (10.0 * 18.0 * 12.0)
  public static final double WRIST_OFFSET =
      0.098; // -0.0325 default wrist angle; zero degree arm in
  // horizontal
  private static final int encoderId = 13;
  public static final double minAngle = 50;
  public static final double maxAngle = 240;
  private static final double PIVOT_POS_SWITCH_THRESHOLD = 2;

  double targetDegrees = SuperStructureState.SOURCE_ANGLE;

  @AutoLog
  public static class WristIOInputs {
    public boolean motorConnected = true;
    public boolean encoderConnected = false;
    public double targetAngle = 0.0;
    public double currentAngle = 0.0;
  }

  private final WristIOInputsAutoLogged pivotInputs = new WristIOInputsAutoLogged();

  public Wrist() {
    talon = new TalonFX(3, "*");
    wristEncoder = new CANcoder(encoderId, "*");

    // Configure encoder
    var cancoderConfig = new CANcoderConfiguration();
    cancoderConfig.MagnetSensor.MagnetOffset = WRIST_OFFSET;
    cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    wristEncoder.getConfigurator().apply(cancoderConfig, 1);

    // Configure motor
    TalonFXConfiguration armTalonConfig = new TalonFXConfiguration();
    armTalonConfig.CurrentLimits.SupplyCurrentLimit = 50.0;
    armTalonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    armTalonConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    armTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    armTalonConfig.Feedback.FeedbackRemoteSensorID = encoderId;
    armTalonConfig.Feedback.FeedbackSensorSource =
        FeedbackSensorSourceValue.FusedCANcoder; // FusedCCANcoder
    armTalonConfig.Feedback.SensorToMechanismRatio = 1.0;
    armTalonConfig.Feedback.RotorToSensorRatio = reduction;

    armTalonConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;
    // Hold the ARM
    armTalonConfig.Slot1.GravityType = GravityTypeValue.Arm_Cosine;
    armTalonConfig.Slot1.kG = 0.35; // 0.35; // to hold the arm weight
    armTalonConfig.Slot1.kP = 20; // 60; // 60; // 100; // adjust PID
    armTalonConfig.Slot1.kI = 0;
    armTalonConfig.Slot1.kD = 0; // 0.02;
    armTalonConfig.Slot1.kS = 0;
    armTalonConfig.Slot1.kV = 0;
    armTalonConfig.Slot1.kA = 0;

    // Move the arm
    armTalonConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    armTalonConfig.Slot0.kG = 0.35; // 0.35; // to hold the arm weight
    armTalonConfig.Slot0.kP = 20; // 60; // 100; // adjust PID
    armTalonConfig.Slot0.kI = 0;
    armTalonConfig.Slot0.kD = 0.01;
    armTalonConfig.Slot0.kS = 0;
    armTalonConfig.Slot0.kV = 5; // 8.3; // move velocity
    armTalonConfig.Slot0.kA = 0.15; // 0.2; // move accerleration

    armTalonConfig.MotionMagic.MotionMagicCruiseVelocity = 5; // 1.0; // 0.5;
    armTalonConfig.MotionMagic.MotionMagicAcceleration = 0.8; // 2; // 1.0;
    armTalonConfig.MotionMagic.MotionMagicJerk = 0; // 10; // 10;

    pMmPos.Slot = 0;
    pMmPos.EnableFOC = true;
    pPos.Slot = 1;
    pPos.EnableFOC = true;

    // Set up armTalonConfig
    tryUntilOk(5, () -> talon.getConfigurator().apply(armTalonConfig, 0.25));

    // ParentDevice.optimizeBusUtilizationForAll(talon, wristEncoder);
  }

  public void periodic() {
    pivotInputs.encoderConnected = wristEncoder.isConnected();
    pivotInputs.motorConnected = talon.isConnected();
    pivotInputs.targetAngle = targetDegrees;
    pivotInputs.currentAngle = 360 * wristEncoder.getAbsolutePosition().getValueAsDouble();
    Logger.processInputs("Wrist", pivotInputs);

    // if (Math.abs(targetDegrees - pivotInputs.currentAngle) < PIVOT_POS_SWITCH_THRESHOLD) {
    //   if (targetDegrees == SuperStructureState.SOURCE_ANGLE
    //       && Math.abs(targetDegrees - pivotInputs.currentAngle) < 1) { // degree
    //     talon.setControl(new NeutralOut());
    //   } else {
    //     talon.setControl(pPos.withPosition(Units.degreesToRotations(targetDegrees)));
    //   }
    // } else {
    talon.setControl(pMmPos.withPosition(Units.degreesToRotations(targetDegrees)));
    // }

    if (DriverStation.isDisabled()) {
      talon.setControl(new NeutralOut());
    }
  }

  public void setVoltage(double voltage) {
    // Set the power to the main motor
    talon.setControl(new VoltageOut(voltage));
  }

  public void setWristAngle(double setPointAngle) {
    targetDegrees = MathUtil.clamp(setPointAngle, minAngle, maxAngle);
  }

  public BooleanSupplier isDone() {
    boolean flag = Math.abs(targetDegrees - pivotInputs.currentAngle) < 3;
    return () -> flag;
  }
}
