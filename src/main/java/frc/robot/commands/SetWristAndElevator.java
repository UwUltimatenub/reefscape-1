// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.SuperStructureState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetWristAndElevator extends Command {

  RobotContainer robot;
  int level = 0;
  // reset at scheduling
  boolean isSafe = false;
  boolean isFinished = false;

  public SetWristAndElevator(RobotContainer robot, int level) {
    addRequirements(robot.wrist, robot.elevator);
    this.robot = robot;
    this.level = level;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isSafe = false;
    isFinished = false;

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
          break;
      }
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!isValidTargetState()) {
      // if not valid target state, do nothing;
      isFinished = true;
      return;

    } else if (robot.targetState == SuperStructureState.STATE_SOURCE) {
      if (robot.currentState.name.startsWith("Coral")) {
        // from coral level to source
        // set elevator
        // then set wrist
        robot.elevator.setElevatorHeight(robot.targetState.height);
        if (robot.elevator.isDone().getAsBoolean()) {
          robot.wrist.setWristAngle(robot.targetState.angle);
          if (robot.wrist.isDone().getAsBoolean()) {
            robot.currentState = robot.targetState;
            isFinished = true;
          }
        }
      } else if (robot.currentState == SuperStructureState.STATE_ALGAE_TOP
          || robot.currentState == SuperStructureState.STATE_ALGAE_MID
          || robot.currentState == SuperStructureState.STATE_ALGAE_LOW) {
        // from top/mid/low algae to source, eject algae
        // robot.intake.intake(IntakeCommand.Eject_Algae);
        // switch to safty angle
        robot.wrist.setWristAngle(SuperStructureState.STATE_SAFTY.angle);
        // along with set elevator height
        robot.elevator.setElevatorHeight(robot.targetState.height);
        if (robot.elevator.isDone().getAsBoolean()) {
          // eject algae, end
          // robot.intake.intake(0);
          // reset wrist angle
          robot.wrist.setWristAngle(robot.targetState.angle);
          if (robot.wrist.isDone().getAsBoolean()) {
            robot.currentState = robot.targetState;
            isFinished = true;
          }
        }
      } else if (robot.currentState == SuperStructureState.STATE_PROCESSOR) {
        // from processor to source
        robot.elevator.setElevatorHeight(robot.targetState.height);
        robot.wrist.setWristAngle(robot.targetState.angle);
        if (robot.wrist.isDone().getAsBoolean()) {
          robot.currentState = robot.targetState;
          isFinished = true;
        }
      }
    } else if (robot.targetState == SuperStructureState.STATE_L2
        || robot.targetState == SuperStructureState.STATE_L3
        || robot.targetState == SuperStructureState.STATE_L4) {
      // set wrist
      // along with set elevator
      robot.wrist.setWristAngle(robot.targetState.angle);
      robot.elevator.setElevatorHeight(robot.targetState.height);
      if (robot.elevator.isDone().getAsBoolean()) {
        robot.currentState = robot.targetState;
        isFinished = true;
      }
    } else if (robot.targetState == SuperStructureState.STATE_L1) {
      // set wrist
      // along with set elevator
      robot.elevator.setElevatorHeight(robot.targetState.height);
      if (robot.elevator.isDone().getAsBoolean()) {
        robot.wrist.setWristAngle(robot.targetState.angle);
        if (robot.wrist.isDone().getAsBoolean()) {
          robot.currentState = robot.targetState;
          isFinished = true;
        }
      }
    } else if (robot.targetState == SuperStructureState.STATE_PROCESSOR
        || robot.targetState == SuperStructureState.STATE_ALGAE_LOW
        || robot.targetState == SuperStructureState.STATE_ALGAE_MID
        || robot.targetState == SuperStructureState.STATE_ALGAE_TOP) {
      // set wrist
      // along with set elevator
      robot.wrist.setWristAngle(robot.targetState.angle);
      robot.elevator.setElevatorHeight(robot.targetState.height);
      if (robot.elevator.isDone().getAsBoolean()) {
        robot.currentState = robot.targetState;
        isFinished = true;
      }
    } else {
      isFinished = true;
    }
    SmartDashboard.putString("state", robot.currentState.name);
    SmartDashboard.putString("desired state", robot.targetState.name);
  }

  private boolean isValidTargetState() {
    boolean valid = false;
    if (robot.currentState == robot.targetState) {
      // the same level, do nothing
      valid = false;
    } else if (robot.currentState == SuperStructureState.STATE_SOURCE
        && robot.intake.isCoralLoaded()
        && robot.targetState.name.startsWith("Coral")) {
      // source after intake coral, go to coral level
      valid = true;
    } else if (robot.currentState == SuperStructureState.STATE_SOURCE
        && !robot.intake.isCoralLoaded()
        && (robot.targetState.name.startsWith("Algae")
            || robot.targetState.name.startsWith("Processor"))) {
      // source without coral, go to algae level/processor
      valid = true;
    } else if (robot.currentState.name.startsWith("Coral")
        && robot.intake.isCoralLoaded()
        && robot.targetState.name.startsWith("Coral")) {
      // with coral, coral level can swith to another coral level
      valid = true;
    } else if (robot.currentState.name.startsWith("Coral")
        && !robot.intake.isCoralLoaded()
        && (robot.targetState.name.startsWith("Algae")
            || robot.targetState.name.startsWith("Source"))) {
      // without coral, coral level can swith to algae intake level or back to source
      valid = true;
    } else if (robot.currentState.name.startsWith("Algae")
        && (robot.targetState.name.startsWith("Barge")
            || robot.targetState.name.startsWith("Processor")
            || robot.targetState.name.startsWith("Source")
            || robot.currentState.name.startsWith("Algae"))) {
      // algae level can swith to algae/barge/processor
      valid = true;
    } else if (robot.currentState.name.startsWith("Barge")
        && robot.targetState.name.startsWith("Source")) {
      // barge to source
      valid = true;
    } else if (robot.currentState.name.startsWith("Processor")
        && (robot.targetState.name.startsWith("Source")
            || robot.targetState.name.startsWith("Algae"))) {
      // processor to source/algae
      valid = true;
    }
    return valid;
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
