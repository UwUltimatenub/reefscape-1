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

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;

/** IO implementation for Pigeon2 */
public class GyroIONavx implements GyroIO {
  private AHRS ahrs;
  private double prevAngle;
  private double initial_degree;

  public GyroIONavx() {

    try {
      /* Communicate w/navX-MXP via the MXP SPI Bus.                                     */
      /* Alternatively:  I2C.Port.kMXP, SerialPort.Port.kMXP or SerialPort.Port.kUSB     */
      /* See http://navx-mxp.kauailabs.com/guidance/selecting-an-interface/ for details. */
      ahrs = new AHRS(SPI.Port.kMXP);
      initial_degree = -ahrs.getAngle();
      prevAngle = initial_degree;

    } catch (RuntimeException ex) {
      DriverStation.reportError("Error instantiating navX-MXP:  " + ex.getMessage(), true);
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    double current_angle = -ahrs.getAngle() - initial_degree;
    inputs.yawPosition = Rotation2d.fromDegrees(current_angle);
    inputs.yawVelocityRadPerSec = Units.degreesToRadians((current_angle - prevAngle) / 0.02);
    prevAngle = current_angle;
  }

  @Override
  public void resetFO() {
    initial_degree = -ahrs.getAngle();
    System.out.println("resetting Field Orientation...");
  }
}
