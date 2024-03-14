// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

public class MoveArm extends Command {

  private Arm arm;
  /** Creates a new MoveArm. */
  public MoveArm(Arm arm) {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setArmLevel(Arm.ARM_LEVEL_SPEAKER);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(Arm.ARM_DEGREES[arm.armPosition] - arm.getInputs().armAbsoluteEncoderPositionRads)
        < Arm.PIVOT_POS_SWITCH_THRESHOLD) {

      return true;
    } else {
      return false;
    }
  }
}
