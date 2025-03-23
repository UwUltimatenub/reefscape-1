// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.SuperStructureState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommand extends Command {

  public static final int Intake_Stopped = 0;
  public static final int Intake_Coral = 1;
  public static final int Eject_Coral = 2;
  public static final int Eject_Coral_Level1 = 5;
  public static final int Intake_Algae = 3;
  public static final int Eject_Algae = 4;
  public int intakeStatus = Intake_Stopped;

  RobotContainer robot;
  boolean isCoral = true;
  boolean isIntake = false;
  SuperStructureState currentState;

  /** Creates a new SetWristAndElevator. */
  public IntakeCommand(RobotContainer robot, boolean isIntake) {
    addRequirements(robot.intake);
    this.isIntake = isIntake;
    this.robot = robot;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if (robot.currentState == SuperStructureState.STATE_SOURCE
        || robot.currentState == SuperStructureState.STATE_L1
        || robot.currentState == SuperStructureState.STATE_L2
        || robot.currentState == SuperStructureState.STATE_L3
        || robot.currentState == SuperStructureState.STATE_L4) {
      isCoral = true;
      if (isIntake) {
        intakeStatus = Intake_Coral;

      } else {
        if (robot.currentState == SuperStructureState.STATE_L1) {
          intakeStatus = Eject_Coral_Level1;
        } else {
          intakeStatus = Eject_Coral;
        }
      }
    } else {
      isCoral = false;
      if (isIntake) {
        intakeStatus = Intake_Algae;
      } else {
        intakeStatus = Eject_Algae;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    robot.intake.intake(intakeStatus);
    // double targetPosition = Math.toRadians(position);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if(!isIntake){
    robot.intake.intake(Intake_Stopped);
    // }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isCoral && isIntake && robot.intake.isCoralLoaded()) {
      robot.controller.setRumble(RumbleType.kLeftRumble, 0.5);
    }
    return isCoral && isIntake && robot.intake.isCoralLoaded()
        || isCoral && !isIntake && !robot.intake.isCoralLoaded();
    // return isFinished;

  }
}
