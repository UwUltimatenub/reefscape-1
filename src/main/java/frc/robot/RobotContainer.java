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

  private final double PROCESSOR_HEIGHT = 0;
  private final double SOURCE_HEIGHT = 8.75;
  private final double L1_HEIGHT = 3;
  private final double L2_HEIGHT = 5.5;
  private final double L3_HEIGHT = 21.5;
  private final double L4_HEIGHT = 52.5;
  private final double TOP_ALGAE_HEIGHT = 40;

  private final double PROCESSOR_ANGLE = Wrist.WRIST_OFFSET.getDegrees() + 0;
  private final double SOURCE_ANGLE = Wrist.WRIST_OFFSET.getDegrees() + 9;
  private final double L1_ANGLE = Wrist.WRIST_OFFSET.getDegrees() + 17;
  private final double L2_ANGLE = Wrist.WRIST_OFFSET.getDegrees() + 13;
  private final double L3_ANGLE = Wrist.WRIST_OFFSET.getDegrees() + 13;
  private final double L4_ANGLE = Wrist.WRIST_OFFSET.getDegrees() + 15;
  private final double TOP_ALGAE_ANGLE = Wrist.WRIST_OFFSET.getDegrees() + 0;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = driverController;
  //private final CommandXboxController operatorController = new CommandXboxController(1);

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
    driverController
        .leftBumper()
        .whileTrue(
            Drive.drive(
                drive,
                () -> driverController.getLeftY() * 0.5,
                () -> driverController.getLeftX() * 0.5,
                () -> -driverController.getRightX() * 0.5));

    // Point wheels in x formation to stop
    driverController.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Point robot to april tag
    driverController
        .a()
        .whileTrue(
            Drive.drive(
                drive,
                () -> driverController.getLeftY(),
                () -> driverController.getLeftX(),
                () -> -vision.autoRotate()));

    // Align robot to april tag
    driverController
        .y()
        .whileTrue(
            Drive.drive(
                drive,
                () -> vision.autoTranslateY(),
                () -> vision.autoTranslateX(),
                () -> -vision.autoRotate()));

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

    // Eject algae
    Command ejectAlgaeCommand =
        new StartEndCommand(
            () -> intake.setAlgaeVoltage(12), () -> intake.setAlgaeVoltage(0), intake);
    driverController.rightBumper().whileTrue(ejectAlgaeCommand);

    Command intakeAlgaeCommand =
        new StartEndCommand(
            () -> intake.setAlgaeVoltage(-12), () -> intake.setAlgaeVoltage(0), intake);
    driverController.rightTrigger().whileTrue(intakeAlgaeCommand);

    // Intake coral
    Command intakeCoralCommand =
        new StartEndCommand(
                () -> intake.setCoralIntakeVoltage(-6),
                () -> intake.setCoralIntakeVoltage(0),
                intake)
            .until((() -> intake.isCoralLoaded()));
    driverController.leftTrigger().whileTrue(intakeCoralCommand);

    Command ejectCoralCommand =
        new StartEndCommand(
            () -> intake.setCoralIntakeVoltage(6), () -> intake.setCoralIntakeVoltage(0), intake);
    operatorController.leftBumper().whileTrue(ejectCoralCommand);

    // Processor state
    Command liftToProcessorCommand =
        new RunCommand(() -> elevator.setPosition(PROCESSOR_HEIGHT), elevator);
    Command wristToProcessorCommand =
        new RunCommand(() -> wrist.wristAngle(PROCESSOR_ANGLE), wrist);
    ParallelCommandGroup processorCommandGroup =
        new ParallelCommandGroup(liftToProcessorCommand, wristToProcessorCommand);
    operatorController.povDown().onTrue(processorCommandGroup);

    // Source state
    Command liftToSourceCommand =
        new RunCommand(() -> elevator.setPosition(SOURCE_HEIGHT), elevator);
    Command wristToSourceCommand = new RunCommand(() -> wrist.wristAngle(SOURCE_ANGLE), wrist);
    ParallelCommandGroup sourceCommandGroup =
        new ParallelCommandGroup(liftToSourceCommand, wristToSourceCommand);
    operatorController.povLeft().onTrue(sourceCommandGroup);

    // L1 state
    Command liftToL1Command = new RunCommand(() -> elevator.setPosition(L1_HEIGHT), elevator);
    Command wristToL1Command = new RunCommand(() -> wrist.wristAngle(L1_ANGLE), wrist);
    ParallelCommandGroup l1CommandGroup =
        new ParallelCommandGroup(liftToL1Command, wristToL1Command);
    operatorController.a().onTrue(l1CommandGroup);

    // L2 state
    Command liftToL2Command = new RunCommand(() -> elevator.setPosition(L2_HEIGHT), elevator);
    Command wristToL2Command = new RunCommand(() -> wrist.wristAngle(L2_ANGLE), wrist);
    ParallelCommandGroup l2CommandGroup =
        new ParallelCommandGroup(liftToL2Command, wristToL2Command);
    operatorController.b().onTrue(l2CommandGroup);

    // L3 state
    Command liftToL3Command = new RunCommand(() -> elevator.setPosition(L3_HEIGHT), elevator);
    Command wristToL3Command = new RunCommand(() -> wrist.wristAngle(L3_ANGLE), wrist);
    ParallelCommandGroup l3CommandGroup =
        new ParallelCommandGroup(liftToL3Command, wristToL3Command);
    operatorController.y().onTrue(l3CommandGroup);

    // L4 state
    Command liftToL4Command = new RunCommand(() -> elevator.setPosition(L4_HEIGHT), elevator);
    Command wristToL4Command = new RunCommand(() -> wrist.wristAngle(L4_ANGLE), wrist);
    ParallelCommandGroup l4CommandGroup =
        new ParallelCommandGroup(liftToL4Command, wristToL4Command);
    operatorController.x().onTrue(l4CommandGroup);

    // Top algae state
    Command liftToTopAlgaeCommand =
        new RunCommand(() -> elevator.setPosition(TOP_ALGAE_HEIGHT), elevator);
    Command wristToTopAlgaeCommand = new RunCommand(() -> wrist.wristAngle(TOP_ALGAE_ANGLE), wrist);
    ParallelCommandGroup topAlgaeCommandGroup =
        new ParallelCommandGroup(liftToTopAlgaeCommand, wristToTopAlgaeCommand);
    operatorController.povUp().onTrue(topAlgaeCommandGroup);

    // Manual lift
    Command manualLift =
        new RunCommand(() -> elevator.setVoltage(-operatorController.getLeftY() * 0.5), elevator);
    // Command manualWrist =
    //     new RunCommand(() -> intake.setWristVoltage(operatorController.getRightY() * 0.25),
    // intake);
    // ParallelCommandGroup manualCommandGroup = new ParallelCommandGroup(manualLift, manualWrist);
    operatorController.start().whileTrue(manualLift);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Example Auto");
  }
}
