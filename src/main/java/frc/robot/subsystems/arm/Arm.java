// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;


import frc.robot.util.Alert;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;

public class Arm {
 
//     public enum ARM_LEVEL {
//        STATION_INTAKE(104), 
//        AIM(203);
//     }
 
//     STATION_INTAKE(  35),
//     AIM(new LoggedTunableNumber("Arm/StationIntakeDegrees", 30.0)),
//     STOW(new LoggedTunableNumber("Arm/StowDegrees", -30.0)),
//     AMP(new LoggedTunableNumber("Arm/AmpDegrees", 20.0)),
//     SUBWOOFER(new LoggedTunableNumber("Arm/SubwooferDegrees", 30.0)),
//     CUSTOM(new LoggedTunableNumber("Arm/CustomSetpoint", 20.0));
// } 
 
 
 

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  

  private final Alert leaderMotorDisconnected =
      new Alert("Arm leader motor disconnected!", Alert.AlertType.WARNING);
  private final Alert followerMotorDisconnected =
      new Alert("Arm follower motor disconnected!", Alert.AlertType.WARNING);
  private final Alert absoluteEncoderDisconnected =
      new Alert("Arm absolute encoder disconnected!", Alert.AlertType.WARNING);

  public Arm(ArmIO io) {
    this.io = io;
    io.setBrakeMode(true);

  }

  public void periodic() {
    // Process inputs
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // Set alerts
    leaderMotorDisconnected.set(!inputs.leaderMotorConnected);
    followerMotorDisconnected.set(!inputs.followerMotorConnected);
    absoluteEncoderDisconnected.set(!inputs.absoluteEncoderConnected);



    if (DriverStation.isDisabled()) {
      io.stop(); 
    }
 
 
  }

  public void stop() {
    io.stop();
  }
 
}
