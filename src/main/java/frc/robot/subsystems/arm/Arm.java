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
  public static final int ARM_LEVEL_AMP = 1;
  public static final int ARM_LEVEL_SPEAKER = 2;
  public static final int ARM_LEVEL_FAR_SPEAKER = 3;
  public static final int ARM_LEVEL_STATION_INTAKE = 4;
  public static final int ARM_LEVEL_HANG = 5;

  private static final double INITIAL_ARM_RADS = -0.38; // -29
  // public static final double INITIAL_ARM_RADS = -0.316; // -22 degree
  public static final double ARM_DEGREES[] = {
    INITIAL_ARM_RADS, // STOW
    INITIAL_ARM_RADS + Units.degreesToRadians(57), // AMP used to be 55 WAS 59
    INITIAL_ARM_RADS + Units.degreesToRadians(74), // SPEAKER
    INITIAL_ARM_RADS + Units.degreesToRadians(70), // FAR SPEAKER aka feeding
    INITIAL_ARM_RADS + Units.degreesToRadians(45), // INTAKE
    INITIAL_ARM_RADS + Units.degreesToRadians(113) // HANG
  };

  //     STATION_INTAKE(  35),
  //     AIM(new LoggedTunableNumber("Arm/StationIntakeDegrees", 30.0)),
  //     STOW(new LoggedTunableNumber("Arm/StowDegrees", -30.0)),
  //     AMP(new LoggedTunableNumber("Arm/AmpDegrees", 20.0)),
  //     SUBWOOFER(new LoggedTunableNumber("Arm/SubwooferDegrees", 30.0)),
  //     CUSTOM(new LoggedTunableNumber("Arm/CustomSetpoint", 20.0));
  // }
  public static final double PIVOT_POS_SWITCH_THRESHOLD = 0.1;

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public ArmIOInputsAutoLogged getInputs() {
    return inputs;
  }

  private final Alert leaderMotorDisconnected =
      new Alert("Arm leader motor disconnected!", Alert.AlertType.WARNING);
  private final Alert followerMotorDisconnected =
      new Alert("Arm follower motor disconnected!", Alert.AlertType.WARNING);
  private final Alert absoluteEncoderDisconnected =
      new Alert("Arm absolute encoder disconnected!", Alert.AlertType.WARNING);

  public int armPosition = ARM_LEVEL_FAR_SPEAKER;

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
    double positionRads = ARM_DEGREES[armPosition];

    if (Math.abs(positionRads - inputs.armAbsoluteEncoderPositionRads)
        < PIVOT_POS_SWITCH_THRESHOLD) {
      if (armPosition == ARM_LEVEL_STOW
          && Math.abs(positionRads - inputs.armAbsoluteEncoderPositionRads) < 0.01) {
        io.stop();
      } else {
        io.setPositionControl(Units.radiansToRotations(positionRads));
      }
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
    // Timer.delay(0.1);
    System.out.println("setting arm position" + armPosition);
  }

  public void stop() {
    io.stop();
  }
}
