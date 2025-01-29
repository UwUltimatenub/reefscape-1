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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelIOTalonFX implements FlywheelIO {
  private static final double GEAR_RATIO = 1;

  private final TalonFX lf = new TalonFX(18);
  private final TalonFX rf = new TalonFX(20);
  private final TalonFX lb = new TalonFX(17);
  private final TalonFX rb = new TalonFX(19);

  private final StatusSignal<Angle> leaderPosition = lf.getPosition();
  private final StatusSignal<AngularVelocity> leaderVelocity = lf.getVelocity();
  private final StatusSignal<Voltage> leaderAppliedVolts = lf.getMotorVoltage();
  private final StatusSignal<Current> leaderCurrent = lf.getSupplyCurrent();
  private final StatusSignal<Current> followerCurrent = rf.getSupplyCurrent();

  // rotations per second.
  VelocityVoltage velOut =
      new VelocityVoltage(Units.radiansToRotations(0.0));

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

    lf.getConfigurator().apply(config);
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rf.getConfigurator().apply(config);
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    lb.getConfigurator().apply(config);
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rb.getConfigurator().apply(config);

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

  @Override
  public void setVelocity(double velocityRadPerSec) {
    System.out.println("Vel : " + velocityRadPerSec);
    lf.setControl(velOut.withVelocity(velocityRadPerSec)); // rotations per second.
    rf.setControl(velOut.withVelocity(velocityRadPerSec));
    lb.setControl(velOut.withVelocity(velocityRadPerSec));
    rb.setControl(velOut.withVelocity(velocityRadPerSec));
  }

  @Override
  public void stop() {

    lf.stopMotor();
    rf.stopMotor();
    lb.stopMotor();
    rb.stopMotor();
  }
}
