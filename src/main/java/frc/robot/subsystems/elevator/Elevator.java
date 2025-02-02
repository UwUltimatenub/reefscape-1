// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  // Constructor
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  // Method to set power for the elevator
  public void setVoltage(double voltage) {
    System.out.println("Elevator position: " + getPosition());
    io.set(voltage);
  }

  // Method to stop the elevator
  public void stop() {
    io.stop();
  }

  // Set the elevator to a specific position
  public void setPosition(double position) {
    // System.out.println("Elevator position: " + getPosition());
    io.setPosition(position);
  }

  // Periodic method called in every cycle (e.g., 20ms)
  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public double getPosition() {
    return io.getPosition();
  }

  public double getVelocity() {
    return io.getVelocity();
  }

  public void resetPosition() {
    io.resetPosition();
  }
}
