// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SuperStructureState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.pivot.Wrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetWristAndElevator extends Command {

  Elevator elevator;
  Wrist wrist;
  SuperStructureState state;

  /** Creates a new SetWristAndElevator. */
  public SetWristAndElevator(Wrist wrist, Elevator elevator, SuperStructureState state) {
    addRequirements(wrist, elevator);

    this.wrist = wrist;
    this.elevator = elevator;
    this.state = state;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.wristAngle(state);
    if (wrist.isDone().getAsBoolean()) {
      elevator.setElevatorHeight(state);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
