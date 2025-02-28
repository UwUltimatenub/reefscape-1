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
  SuperStructureState state;
  boolean isSafe = false;
  double safeAngle = 0;
  boolean isFinished = false;

  /** Creates a new SetWristAndElevator. */
  public SetWristAndElevator(RobotContainer robot, int level) {
    addRequirements(robot.wrist, robot.elevator);

    this.robot = robot;
    if (robot.intake.isCoralLoaded()) {
      switch (level) {
        case 1:
          state = SuperStructureState.STATE_L1;
          break;
        case 2:
          state = SuperStructureState.STATE_L2;
          break;
        case 3:
          state = SuperStructureState.STATE_L3;
          break;
        case 4:
          state = SuperStructureState.STATE_L4;
          break;
        default:
          state = SuperStructureState.STATE_SOURCE;
          break;
      }
    } else {
      switch (level) {
        case 1:
          state = SuperStructureState.STATE_PROCESSOR;
          break;
        case 2:
          state = SuperStructureState.STATE_ALGAE_LOW;
          break;
        case 3:
          state = SuperStructureState.STATE_ALGAE_MID;
          break;
        case 4:
          state = SuperStructureState.STATE_ALGAE_TOP;
          break;
        default:
          state = SuperStructureState.STATE_SOURCE;
          break;
      }
    }
    if (robot.currentState == SuperStructureState.STATE_SOURCE
        || robot.currentState == SuperStructureState.STATE_L1
        || robot.currentState == SuperStructureState.STATE_L2
        || robot.currentState == SuperStructureState.STATE_L3
        || robot.currentState == SuperStructureState.STATE_L4) {
      safeAngle = SuperStructureState.STATE_SAFTY.angle; // safty angle for coral
    } else {
      safeAngle = SuperStructureState.STATE_SAFTY.height; // safty angle for algae
    }

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // double targetPosition = Math.toRadians(position);
    if (!isSafe) {
      // calculate safty angle
      robot.wrist.setWristAngle(safeAngle);
      if (robot.wrist.isDone().getAsBoolean()) {
        isSafe = true;
      }
    } else {
      robot.elevator.setElevatorHeight(state.height);
      if (robot.elevator.isDone().getAsBoolean()) {
        robot.wrist.setWristAngle(state.angle);
        if (robot.wrist.isDone().getAsBoolean()) {
          robot.currentState = state;
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
    return false;
    // return isFinished;

  }
}
