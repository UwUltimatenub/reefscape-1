// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.arm.Arm;

public class SpeakerShoot extends Command {
  private boolean isFinished = false;

  private RobotContainer robot;
  /** Creates a new MoveArm. */
  public SpeakerShoot(RobotContainer myRobot) {
    robot = myRobot;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robot.arm.setArmLevel(Arm.ARM_LEVEL_SPEAKER);
    Timer.delay(1000);
    robot.flywheel.runVelocity(robot.getFlywheelRPM());
    robot.intake.runVelocity(RobotContainer.INTAKE_ROLLER_SPEED);
    Timer.delay(500);

    robot.arm.setArmLevel(Arm.ARM_LEVEL_FAR_SPEAKER);
    robot.flywheel.stop();
    robot.intake.stop();
    Timer.delay(1000);
    isFinished = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
