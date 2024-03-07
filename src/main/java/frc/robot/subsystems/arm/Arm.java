// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Alert;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  public static final int ARM_LEVEL_STOW = 0;
  public static final int ARM_LEVEL_DIAGNOSE = 1;
  public static final int ARM_LEVEL_AMP = 2;
  public static final int ARM_LEVEL_SPEAKER = 3;
  public static final int ARM_LEVEL_FAR_SPEAKER = 4;
  public static final int ARM_LEVEL_STATION_INTAKE = 5;
  public static final int ARM_LEVEL_HANG = 6;

  //     STATION_INTAKE(  35),
  //     AIM(new LoggedTunableNumber("Arm/StationIntakeDegrees", 30.0)),
  //     STOW(new LoggedTunableNumber("Arm/StowDegrees", -30.0)),
  //     AMP(new LoggedTunableNumber("Arm/AmpDegrees", 20.0)),
  //     SUBWOOFER(new LoggedTunableNumber("Arm/SubwooferDegrees", 30.0)),
  //     CUSTOM(new LoggedTunableNumber("Arm/CustomSetpoint", 20.0));
  // }
  private static final double PIVOT_POS_SWITCH_THRESHOLD = 0.01;

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private final Alert leaderMotorDisconnected =
      new Alert("Arm leader motor disconnected!", Alert.AlertType.WARNING);
  private final Alert followerMotorDisconnected =
      new Alert("Arm follower motor disconnected!", Alert.AlertType.WARNING);
  private final Alert absoluteEncoderDisconnected =
      new Alert("Arm absolute encoder disconnected!", Alert.AlertType.WARNING);
  private static final double INITIAL_ARM_RADS = -0.43;

  public int armPosition = ARM_LEVEL_STOW;

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
    double positionRads = 0;
    switch (armPosition) {
      case Arm.ARM_LEVEL_STOW:
        positionRads = INITIAL_ARM_RADS;
        break;
      case Arm.ARM_LEVEL_DIAGNOSE:
        positionRads = INITIAL_ARM_RADS + Units.degreesToRadians(20);
        break;
      case Arm.ARM_LEVEL_AMP:
        positionRads = INITIAL_ARM_RADS + Units.degreesToRadians(60);
        break;
      case Arm.ARM_LEVEL_SPEAKER:
        positionRads = INITIAL_ARM_RADS + Units.degreesToRadians(83);
        break;
      case Arm.ARM_LEVEL_FAR_SPEAKER:
        positionRads = INITIAL_ARM_RADS + Units.degreesToRadians(62);
        break;
      case Arm.ARM_LEVEL_STATION_INTAKE:
        positionRads = INITIAL_ARM_RADS + Units.degreesToRadians(50);
        break;
      case Arm.ARM_LEVEL_HANG:
        positionRads = INITIAL_ARM_RADS + Units.degreesToRadians(115);
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    if (Math.abs(positionRads - inputs.armAbsoluteEncoderPositionRads)
        < PIVOT_POS_SWITCH_THRESHOLD) {
      io.setPositionControl(Units.radiansToRotations(positionRads));
    } else {
      io.setMotionControl(Units.radiansToRotations(positionRads));
    }

    if (DriverStation.isDisabled()) {
      io.setBrakeMode(true);
      io.stop();
    }
  }

  public void setArmLevel(int level) {
    armPosition = level;
    System.out.println("setting arm position" + armPosition);
  }

  public void stop() {
    io.stop();
  }
}
