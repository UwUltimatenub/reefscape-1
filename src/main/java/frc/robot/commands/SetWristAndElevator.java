// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.SuperStructureState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetWristAndElevator extends Command {

  RobotContainer robot;
  double safeAngle = 0;
  boolean isSafe = false;
  boolean isFinished = false;
  boolean isNoCoralToSource = false;

  int level = 0;

  /** Creates a new SetWristAndElevator. */
  public SetWristAndElevator(RobotContainer robot, int level) {
    addRequirements(robot.wrist, robot.elevator, robot.intake);

    this.robot = robot;
    this.level = level;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isSafe = false;
    isFinished = false;
    isNoCoralToSource = false;

    if (robot.intake.isCoralLoaded()) {
      switch (level) {
        case 1:
          robot.targetState = SuperStructureState.STATE_L1;
          break;
        case 2:
          robot.targetState = SuperStructureState.STATE_L2;
          break;
        case 3:
          robot.targetState = SuperStructureState.STATE_L3;
          break;
        case 4:
          robot.targetState = SuperStructureState.STATE_L4;
          break;
        default:
          robot.targetState = SuperStructureState.STATE_SOURCE;
          break;
      }
    } else {
      switch (level) {
        case 1:
          robot.targetState = SuperStructureState.STATE_PROCESSOR;
          break;
        case 2:
          robot.targetState = SuperStructureState.STATE_ALGAE_LOW;
          break;
        case 3:
          robot.targetState = SuperStructureState.STATE_ALGAE_MID;
          break;
        case 4:
          robot.targetState = SuperStructureState.STATE_ALGAE_TOP;
          break;
        default:
          robot.targetState = SuperStructureState.STATE_SOURCE;
          isNoCoralToSource = true;
          break;
      }
    }
    if (robot.currentState == robot.targetState) {
      isSafe = true;
    } else if (robot.currentState == SuperStructureState.STATE_SOURCE
        || robot.currentState == SuperStructureState.STATE_L1
        || robot.currentState == SuperStructureState.STATE_L2
        || robot.currentState == SuperStructureState.STATE_L3
        || robot.currentState == SuperStructureState.STATE_L4) {
      // doing coral, set safty angle for coral
      safeAngle = SuperStructureState.STATE_SAFTY.angle;
    } else {
      // doing algae, set safty angle for algae
      if (robot.currentState == SuperStructureState.STATE_ALGAE_TOP) {
        safeAngle = SuperStructureState.STATE_SAFTY.angle; // safty angle for coral
      } else if (robot.targetState == SuperStructureState.STATE_SOURCE) {
        safeAngle = SuperStructureState.STATE_SAFTY.angle; // safty angle for algae
      } else {
        safeAngle = SuperStructureState.STATE_SAFTY.height; // safty angle for algae
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // double targetPosition = Math.toRadians(position);
    if (!isSafe) {
      // no coral to source, eject algae
      if (isNoCoralToSource) {
        robot.intake.intake(IntakeCommand.Eject_Algae);
      }
      // calculate safty angle
      robot.wrist.setWristAngle(safeAngle);
      if (robot.wrist.isDone().getAsBoolean()) {
        isSafe = true;
      }
    } else {
      robot.elevator.setElevatorHeight(robot.targetState.height);
      if (robot.elevator.isDone().getAsBoolean()) {
        // no coral to source, eject algae, end
        if (isNoCoralToSource) {
          robot.intake.intake(0);
        }
        robot.wrist.setWristAngle(robot.targetState.angle);
        if (robot.wrist.isDone().getAsBoolean()) {
          robot.currentState = robot.targetState;
          System.out.println("current state=" + robot.targetState.name);
          isFinished = true;
        }
      }
    }
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
