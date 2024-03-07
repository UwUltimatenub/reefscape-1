// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class FlywheelIOTalonFX implements FlywheelIO {
  private static final double GEAR_RATIO = 1;

  private final TalonFX leader = new TalonFX(7);
  private final TalonFX follower = new TalonFX(6);
  private final TalonFX follower2 = new TalonFX(5);
  private final TalonFX follower3 = new TalonFX(2);

  private final StatusSignal<Double> leaderPosition = leader.getPosition();
  private final StatusSignal<Double> leaderVelocity = leader.getVelocity();
  private final StatusSignal<Double> leaderAppliedVolts = leader.getMotorVoltage();
  private final StatusSignal<Double> leaderCurrent = leader.getSupplyCurrent();
  private final StatusSignal<Double> followerCurrent = follower.getSupplyCurrent();

  // rotations per second.
  VelocityVoltage velOut =
      new VelocityVoltage(Units.radiansToRotations(0.0), 0.0, true, 0, 0, false, false, false);

  public FlywheelIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.SupplyCurrentLimit = 50.0;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Voltage.PeakForwardVoltage = 12.0;
    config.Voltage.PeakReverseVoltage = -12.0;
    config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.2;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.Slot0.kP = 0.04;
    config.Slot0.kD = 0; // 0.05;
    config.Slot0.kV = .12; // 10;
    config.Slot0.kS = 0; // 0.33329;

    leader.getConfigurator().apply(config);
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    follower.getConfigurator().apply(config);
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    follower2.getConfigurator().apply(config);
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    follower3.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0, leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    BaseStatusSignal.refreshAll(
        leaderPosition, leaderVelocity, leaderAppliedVolts, leaderCurrent, followerCurrent);
    inputs.positionRad = Units.rotationsToRadians(leaderPosition.getValueAsDouble()) / GEAR_RATIO;
    inputs.velocityRadPerSec =
        Units.rotationsToRadians(leaderVelocity.getValueAsDouble()) / GEAR_RATIO;
    inputs.appliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.currentAmps =
        new double[] {leaderCurrent.getValueAsDouble(), followerCurrent.getValueAsDouble()};
  }

  // @Override
  // public void setVoltage(double volts) {
  //   System.out.println("Volts : " + volts);
  //   leader.setControl(new VoltageOut(volts));
  //   follower.setControl(new VoltageOut(volts));
  //   follower2.setControl(new VoltageOut(volts));
  //   follower3.setControl(new VoltageOut(volts));
  // }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    System.out.println("Vel : " + velocityRadPerSec);
    leader.setControl(velOut.withVelocity(velocityRadPerSec * 1.1)); // rotations per second.
    follower.setControl(velOut.withVelocity(velocityRadPerSec * 0.95));
    follower2.setControl(velOut.withVelocity(velocityRadPerSec * 1.1));
    follower3.setControl(velOut.withVelocity(velocityRadPerSec * 0.95));
  }

  @Override
  public void stop() {

    leader.stopMotor();
    follower.stopMotor();
    follower2.stopMotor();
    follower3.stopMotor();
  }
}
