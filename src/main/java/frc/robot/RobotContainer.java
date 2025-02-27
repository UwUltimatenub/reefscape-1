// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
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
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SetWristAndElevator;
import frc.robot.generated.TunerConstants;
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
  public final Intake intake;
  public final Wrist wrist;
  public final Elevator elevator;

  private SuperStructureState currentState = SuperStructureState.STATE_SOURCE;

  // Controller
  public final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Real robot, instantiate hardware IO implementations
    vision = new LimeLight();
    wrist = new Wrist();
    elevator = new Elevator();
    intake = new Intake(elevator);
    drive =
        new Drive(
            new GyroIOPigeon2(),
            new ModuleIOTalonFX(TunerConstants.FrontLeft),
            new ModuleIOTalonFX(TunerConstants.FrontRight),
            new ModuleIOTalonFX(TunerConstants.BackLeft),
            new ModuleIOTalonFX(TunerConstants.BackRight));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
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
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d()));

    // Point robot to april tag
    // controller
    //     .leftStick()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> controller.getLeftY(),
    //             () -> controller.getLeftX(),
    //             () -> new Rotation2d(Units.degreesToRadians(vision.autoRotate()))));

    // // Align robot to april tag
    // controller
    //     .back()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> vision.autoTranslateY(),
    //             () -> vision.autoTranslateX(),
    //             () -> new Rotation2d(Units.degreesToRadians(vision.autoRotate()))));

    // Reset gyro
    controller
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // Intake coral/Algae
    controller.leftBumper().whileTrue(getIntakeCommand(true));
    // Eject coral
    controller.leftTrigger().whileTrue(getIntakeCommand(false));

    // Source state, safty wrist angle to prevent collision
    // controller.a().onTrue(getStateCommand(SuperStructureState.STATE_SOURCE));

    // L2 state
    elevator.setDefaultCommand(Commands.run(() -> {}, elevator));
    wrist.setDefaultCommand(Commands.run(() -> {}, wrist));

    controller.x().onTrue(new SetWristAndElevator(wrist, elevator, SuperStructureState.STATE_L2));

    controller
        .a()
        .onTrue(new SetWristAndElevator(wrist, elevator, SuperStructureState.STATE_SOURCE));
    // .until(wrist.isDone())
    // .andThen(
    //     Commands.run(() -> elevator.setElevatorHeight(SuperStructureState.STATE_L2)))
    // .until(elevator.isDone()));

    // L3 state
    // controller.y().onTrue(getStateCommand(SuperStructureState.STATE_L2));
    controller.y().onTrue(new SetWristAndElevator(wrist, elevator, SuperStructureState.STATE_L3));

    // L4 state
    controller.b().onTrue(getStateCommand(SuperStructureState.STATE_L4));

    // Processor state
    controller.povDown().onTrue(getStateCommand(SuperStructureState.STATE_PROCESSOR));

    // Low Algae state
    controller.povLeft().onTrue(getStateCommand(SuperStructureState.STATE_ALGAE_LOW));

    // Mid Algae state
    controller.povUp().onTrue(getStateCommand(SuperStructureState.STATE_ALGAE_MID));

    // Top algae state
    controller.povRight().onTrue(getStateCommand(SuperStructureState.STATE_ALGAE_TOP));

    // Manual lift
    Command manualLift =
        new RunCommand(() -> elevator.setVoltage(-controller.getLeftY() * 2), elevator);
    Command manualWrist = new RunCommand(() -> wrist.setVoltage(controller.getRightY() * 2), wrist);
    ParallelCommandGroup manualCommandGroup = new ParallelCommandGroup(manualLift, manualWrist);
    controller.rightBumper().whileTrue(manualCommandGroup);

    // Point wheels in x formation to stop
    controller.rightTrigger().onTrue(Commands.runOnce(drive::stopWithX, drive));
  }

  public Command getIntakeCommand(boolean isIntake) {
    Command command = null;
    // Intake coral/algae

    command =
        new StartEndCommand(() -> intake.intake(isIntake), () -> intake.stop(), intake)
            .until((() -> intake.isDone())); // || intake.overloaded()

    return command;
  }

  public Command getStateCommand(SuperStructureState toState) {

    Command command = null;
    // if (currentState == toState) return command;

    // if (currentState == SuperStructureState.STATE_SOURCE
    //     || toState == SuperStructureState.STATE_SOURCE) {
    // To/from Source state, set safty wrist angle to prevent collision
    Command wristToSafteyCommand =
        new RunCommand(() -> wrist.wristAngle(SuperStructureState.STATE_SAFTY), wrist);
    Command liftCommand = new RunCommand(() -> elevator.setElevatorHeight(toState), elevator);
    Command wristCommand = new RunCommand(() -> wrist.wristAngle(toState), wrist);

    command =
        wristToSafteyCommand
            .until(wrist.isDone())
            .andThen(liftCommand)
            .until(elevator.isDone())
            .andThen(wristCommand)
            .until(wrist.isDone());
    // } else {

    //   // General State, run height and angle concurrently
    //   Command liftCommand =
    //       new RunCommand(() -> elevator.setElevatorHeight(toState), elevator);
    //   Command wristCommand = new RunCommand(() -> wrist.wristAngle(toState), wrist);
    //   command = new ParallelCommandGroup(liftCommand, wristCommand);
    // }

    // currentState = toState;

    return command;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
