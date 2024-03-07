// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmIOReal;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReal;
import frc.robot.subsystems.intake.IntakeIOSim;
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
  private final Flywheel flywheel;
  private final Intake intake;
  private final Arm arm;

  public Arm getArm() {
    return arm;
  }

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                // new GyroIONavx(),
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        flywheel = new Flywheel(new FlywheelIOTalonFX());

        intake = new Intake(new IntakeIOReal());
        arm = new Arm(new ArmIOReal());
        // drive = new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFX(0),
        // new ModuleIOTalonFX(1),
        // new ModuleIOTalonFX(2),
        // new ModuleIOTalonFX(3));
        // flywheel = new Flywheel(new FlywheelIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                // new GyroIONavx(),
                new GyroIOPigeon2(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        flywheel = new Flywheel(new FlywheelIOSim());
        intake = new Intake(new IntakeIOSim());
        arm = new Arm(new ArmIOSim());

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                // new GyroIONavx(),
                new GyroIOPigeon2(),
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        intake = new Intake(new IntakeIOReal());
        arm = new Arm(new ArmIOReal());
        break;
    }

    // Set up auto routines
    NamedCommands.registerCommand(
        "Run Flywheel",
        Commands.startEnd(() -> flywheel.runVelocity(getFlywheelRPM()), flywheel::stop, flywheel)
            .withTimeout(5.0));
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

  public double getFlywheelRPM() {
    double flywheelRPM = 0;
    switch (arm.armPosition) {
      case Arm.ARM_LEVEL_AMP:
        flywheelRPM = 500;
        break;
      case Arm.ARM_LEVEL_SPEAKER:
        flywheelRPM = 2000;
        break;
      case Arm.ARM_LEVEL_FAR_SPEAKER:
        flywheelRPM = 3000;
        break;
    }
    return flywheelRPM;
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller.y().onTrue(Commands.runOnce(drive::resetFieldOrientation, drive));

    // drive
    // drive.setDefaultCommand(
    //     DriveCommands.joystickDrive(
    //         drive,
    //         () ->
    //             arm.armPosition == Arm.ARM_LEVEL_STOW
    //                 ? -controller.getLeftY() / 1.25
    //                 : -controller.getLeftY() / 2,
    //         () ->
    //             arm.armPosition == Arm.ARM_LEVEL_STOW
    //                 ? -controller.getLeftX() / 1.25
    //                 : -controller.getLeftX() / 2,
    //         () -> controller.getRightX() / 1.4));

    // drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () ->
                arm.armPosition == Arm.ARM_LEVEL_STOW
                    ? -controller.getLeftY() / (controller.back().getAsBoolean() ? 2 : 1)
                    : -controller.getLeftY() / (controller.back().getAsBoolean() ? 2 : 2),
            () ->
                arm.armPosition == Arm.ARM_LEVEL_STOW
                    ? -controller.getLeftX() / (controller.back().getAsBoolean() ? 2 : 1)
                    : -controller.getLeftX() / (controller.back().getAsBoolean() ? 2 : 2),
            () -> controller.getRightX() / (controller.back().getAsBoolean() ? 2 : 1.25)));

    // controller
    //     .start()
    //     .onTrue(
    //         Commands.runOnce(
    //                 () ->
    //                     drive.setPose(
    //                         new Pose2d(drive.getPose().getTranslation(), new
    // Rotation2d(3.14/2))),
    //                 drive)
    //             .ignoringDisable(true));

    // high hang
    controller
        .b()
        .whileTrue(
            Commands.runOnce(() -> arm.setArmLevel(Arm.ARM_LEVEL_HANG))
                .alongWith(Commands.runOnce(() -> intake.stop()))
                .alongWith(Commands.runOnce(() -> flywheel.stop())))
        .whileFalse(
            Commands.runOnce(() -> arm.setArmLevel(Arm.ARM_LEVEL_STOW)).andThen(() -> arm.stop()));

    // intake
    controller
        .a()
        .whileTrue(
            Commands.startEnd(() -> intake.runVolts(4), () -> intake.stop(), intake)
                .alongWith(Commands.runOnce(() -> flywheel.runVelocity(getFlywheelRPM()))));

    // .alongWith(
    //     Commands.run(
    //         () ->
    //             controller
    //                 .getHID()
    //                 .setRumble(GenericHID.RumbleType.kBothRumble, 0.20))));
    // .toggleOnFalse(
    //     Commands.runOnce(() -> intake.stop())
    //         .alongWith(
    //             Commands.run(
    //                 () ->
    //                     controller.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0))));

    // Left Bumper, Arm to Feeder
    controller
        .leftTrigger()
        .whileTrue(
            Commands.runOnce(() -> arm.setArmLevel(Arm.ARM_LEVEL_STATION_INTAKE))
                .alongWith(Commands.runOnce(() -> flywheel.stop()))
                .alongWith(Commands.runOnce(() -> intake.runVolts(4))))
        .whileFalse(
            Commands.runOnce(() -> arm.setArmLevel(Arm.ARM_LEVEL_STOW))
                .alongWith(Commands.runOnce(() -> flywheel.stop()))
                .alongWith(Commands.runOnce(() -> intake.stop()))
                .andThen(() -> arm.stop()));
    // .alongWith(Commands.runOnce(() -> intake.runVolts(12))));

    // Right Trigger, Arm to AMP
    controller
        .leftBumper()
        .whileTrue(
            Commands.runOnce(() -> arm.setArmLevel(Arm.ARM_LEVEL_AMP))
                .alongWith(Commands.runOnce(() -> flywheel.runVelocity(getFlywheelRPM())))
                .alongWith(Commands.runOnce(() -> intake.stop())))
        .whileFalse(
            Commands.runOnce(() -> arm.setArmLevel(Arm.ARM_LEVEL_STOW))
                .alongWith(Commands.runOnce(() -> flywheel.stop()))
                .alongWith(Commands.runOnce(() -> intake.stop()))
                .andThen(() -> arm.stop()));

    // Right trigger, Arm to Speaker
    controller
        .rightTrigger()
        .whileTrue(
            Commands.runOnce(() -> arm.setArmLevel(Arm.ARM_LEVEL_SPEAKER))
                .andThen(Commands.runOnce(() -> flywheel.runVelocity(getFlywheelRPM())))
                .alongWith(Commands.runOnce(() -> intake.stop())))
        .whileFalse(
            Commands.runOnce(() -> arm.setArmLevel(Arm.ARM_LEVEL_STOW))
                .alongWith(Commands.runOnce(() -> flywheel.stop()))
                .alongWith(Commands.runOnce(() -> intake.stop()))
                .andThen(() -> arm.stop()));
    // Right Bumper, Arm to far Speaker
    controller
        .rightBumper()
        .whileTrue(
            Commands.runOnce(() -> arm.setArmLevel(Arm.ARM_LEVEL_FAR_SPEAKER))
                .andThen(Commands.runOnce(() -> flywheel.runVelocity(getFlywheelRPM())))
                .alongWith(Commands.runOnce(() -> intake.stop())))
        .whileFalse(
            Commands.runOnce(() -> arm.setArmLevel(Arm.ARM_LEVEL_STOW))
                .alongWith(Commands.runOnce(() -> flywheel.stop()))
                .alongWith(Commands.runOnce(() -> intake.stop()))
                .andThen(() -> arm.stop()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //   public Command getAutonomousCommand() {
  //     return autoChooser.get();
  //   }
  public Command getAutonomousCommand() {

    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

    // Create a path following command using AutoBuilder. This will also trigger event markers.
    return AutoBuilder.followPath(path);
  }
}
