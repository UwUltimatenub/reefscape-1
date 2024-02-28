// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(2),
          reduction,
          1.06328,
          armLength,
          minAngle.getRadians(),
          maxAngle.getRadians(),
          false,
          Units.degreesToRadians(0.0));

  private double appliedVoltage = 0.0;
  private double positionOffset = 0.0;



  private boolean wasNotAuto = true;

  public ArmIOSim() { 
    sim.setState(0.0, 0.0);
    setPosition(0.0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
 
    // Assume starting at ~80 degrees
    if (wasNotAuto && DriverStation.isAutonomousEnabled()) {
      sim.setState(Units.degreesToRadians(80.0), 0.0);
      wasNotAuto = false;
    }
    if (!DriverStation.isAutonomousEnabled()) {
      wasNotAuto = true;
    }

    sim.update(0.02);

    inputs.armPositionRads = sim.getAngleRads() + positionOffset;
    inputs.armVelocityRadsPerSec = sim.getVelocityRadPerSec();
    inputs.armAppliedVolts = new double[] {appliedVoltage};
    inputs.armCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.armTorqueCurrentAmps = new double[] {sim.getCurrentDrawAmps()};
    inputs.armTempCelcius = new double[] {0.0};

    // Reset input
    sim.setInputVoltage(0.0);
  }

  @Override
  public void stop() {
    appliedVoltage = 0.0;
    sim.setInputVoltage(appliedVoltage);
  }

  @Override
  public void setPosition(double position) {
    positionOffset = position - sim.getAngleRads();
  }
}
