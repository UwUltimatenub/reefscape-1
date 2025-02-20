// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Wrist;
import frc.robot.subsystems.vision.LimeLight;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final LimeLight vision;
  private final Intake intake;
  private final Wrist wrist;
  private final Elevator elevator;

  private SuperStructureState currentState = SuperStructureState.STATE_SOURCE;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = driverController;
  // private final CommandXboxController operatorController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, IO devices, and commands. */
  public RobotContainer() {

    // Real robot, instantiate hardware IO implementations
    vision = new LimeLight();
    intake = new Intake();
    wrist = new Wrist();
    elevator = new Elevator();
    drive =
        new Drive(
            new GyroIOPigeon2(),
            new LimeLight(),
            new ModuleIOTalonFX(0),
            new ModuleIOTalonFX(1),
            new ModuleIOTalonFX(2),
            new ModuleIOTalonFX(3));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Field centric swerve drive
    drive.setDefaultCommand(
        Drive.drive(
            drive,
            () -> driverController.getLeftY() * 0.6,
            () -> driverController.getLeftX() * 0.6,
            () -> -driverController.getRightX() * 0.65));

    // Slowed field centric swerve drive
    // driverController
    //     .leftBumper()
    //     .whileTrue(
    //         Drive.drive(
    //             drive,
    //             () -> driverController.getLeftY() * 0.5,
    //             () -> driverController.getLeftX() * 0.5,
    //             () -> -driverController.getRightX() * 0.5));

    // Point wheels in x formation to stop
    driverController.rightTrigger().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Point robot to april tag
    // driverController
    //     .a()
    //     .whileTrue(
    //         Drive.drive(
    //             drive,
    //             () -> driverController.getLeftY(),
    //             () -> driverController.getLeftX(),
    //             () -> -vision.autoRotate()));

    // // Align robot to april tag
    // driverController
    //     .y()
    //     .whileTrue(
    //         Drive.drive(
    //             drive,
    //             () -> vision.autoTranslateY(),
    //             () -> vision.autoTranslateX(),
    //             () -> -vision.autoRotate()));

    // Reset gyro
    driverController
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Intake coral/algae
    driverController.leftBumper().whileTrue(getIntakeCommand(true));

    // Eject coral/algae
    operatorController.leftTrigger().whileTrue(getIntakeCommand(false));

    // Source state, safty wrist angle to prevent collision
    operatorController.a().onTrue(getStateCommand(SuperStructureState.STATE_SOURCE));

    // L2 state
    operatorController.x().onTrue(getStateCommand(SuperStructureState.STATE_L2));

    // L3 state
    operatorController.y().onTrue(getStateCommand(SuperStructureState.STATE_L3));

    // L4 state
    operatorController.b().onTrue(getStateCommand(SuperStructureState.STATE_L4));

    // Processor state
    operatorController.povDown().onTrue(getStateCommand(SuperStructureState.STATE_PROCESSOR));

    // Low Algae state
    operatorController.povLeft().onTrue(getStateCommand(SuperStructureState.STATE_ALGAE_LOW));

    // Mid Algae state
    operatorController.povUp().onTrue(getStateCommand(SuperStructureState.STATE_ALGAE_MID));

    // Top algae state
    operatorController.povRight().onTrue(getStateCommand(SuperStructureState.STATE_ALGAE_TOP));

    // Manual lift
    Command manualLift =
        new RunCommand(() -> elevator.setVoltage(-operatorController.getLeftY() * 0.5), elevator);
    Command manualWrist =
        new RunCommand(() -> wrist.setVoltage(operatorController.getRightY() * 0.25), wrist);
    ParallelCommandGroup manualCommandGroup = new ParallelCommandGroup(manualLift, manualWrist);
    operatorController.rightBumper().whileTrue(manualCommandGroup);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Example Auto");
  }

  public Command getIntakeCommand(boolean isIntake) {
    Command command = null;
    // Intake coral/algae
    if(currentState==SuperStructureState.STATE_SOURCE||currentState==SuperStructureState.STATE_L2
        ||currentState==SuperStructureState.STATE_L3||currentState==SuperStructureState.STATE_L4){
            if(isIntake){
                command = new StartEndCommand(() -> intake.forward(8), () -> intake.stop(), intake)
                .until((() -> intake.isCoralLoaded() || intake.overloaded()));        
            }else{
                command = new StartEndCommand(() -> intake.forward(8), () -> intake.stop(), intake);        
            }
        }else  if(currentState==SuperStructureState.STATE_PROCESSOR||currentState==SuperStructureState.STATE_ALGAE_LOW
        ||currentState==SuperStructureState.STATE_ALGAE_MID||currentState==SuperStructureState.STATE_ALGAE_TOP){
            if(isIntake){
                command = new StartEndCommand(() -> intake.backward(8), () -> intake.stop(), intake)
                .until((() ->  intake.overloaded()));        
            }else{
                command = new StartEndCommand(() -> intake.forward(12), () -> intake.stop(), intake);        
            }
        }
    return command;
  }

  public Command getStateCommand(SuperStructureState toState) {

    Command command = null;
    //if (currentState == toState) return command;

    if (currentState == SuperStructureState.STATE_SOURCE
        || toState == SuperStructureState.STATE_SOURCE) {
      // To/from Source state, set safty wrist angle to prevent collision
      Command wristToSafteyCommand =
          new RunCommand(() -> wrist.wristAngle(SuperStructureState.L2_ANGLE), wrist);
      Command liftCommand =
          new RunCommand(() -> elevator.setElevatorHeight(toState.height), elevator);
      Command wristCommand = new RunCommand(() -> wrist.wristAngle(toState.angle), wrist);

      command = wristToSafteyCommand.andThen(liftCommand).andThen(wristCommand);
    } else {

      // General State, run height and angle concurrently
      Command liftCommand =
          new RunCommand(() -> elevator.setElevatorHeight(toState.height), elevator);
      Command wristCommand = new RunCommand(() -> wrist.wristAngle(toState.angle), wrist);
      command = new ParallelCommandGroup(liftCommand, wristCommand);
    }

    currentState = toState;

    return command;
  }
}
