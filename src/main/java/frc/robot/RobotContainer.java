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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
  public static final double INTAKE_ROLLER_SPEED = 300; // RPM
  // joystick threshold
  // joystick value 0 under the threshold
  static final double Joystick_Threshold = 0.05;

  // Subsystems
  public final Drive drive;
  public final Flywheel flywheel;
  public final Intake intake;
  public final Arm arm;

  //   PIDController turnPIDController = new PIDController(0.001, 0, 0.00);
  //   boolean isHeadingLock = false;
  //   double heading;

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
        flywheelRPM = 400;
        break;
      case Arm.ARM_LEVEL_SPEAKER:
        flywheelRPM = 2000;
        break;
      case Arm.ARM_LEVEL_FAR_SPEAKER:
        flywheelRPM = 2000;
        break;
    }
    return flywheelRPM;
  }

  /***************************************************************************************************************************************
   * double getClosestAngle(double angle_lib)
   * take Normalized Angle and return value -180 to 180.
   *
   * for example: getClosestAngle(350) will return -20
   * getNormalizedAngle(20) will return 20
   ***************************************************************************************************************************************/
  double getClosestAngle(double input) {
    return (input > 180 ? -(360 - input) : input);
  }

  // map joystick input to curved output
  // less sensitive at the low range, very sensentive at high range
  private double regulate(double input) {
    double output = input;
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      output = -input;
    }
    // double output =
    //     (1 / (1 - Joystick_Threshold)) * (input - Math.copySign(1, input))
    //         + Math.copySign(1, input);
    // output = output * Math.abs(output) / 1;
    return output;
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
    controller
        .b()
        .onTrue(Commands.runOnce(() -> drive.setPose(new Pose2d(0.0, 0.0, new Rotation2d(0)))));

    // drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () ->
                arm.armPosition == Arm.ARM_LEVEL_STOW
                    ? regulate(-controller.getLeftY()) / (controller.back().getAsBoolean() ? 2 : 1)
                    : regulate(-controller.getLeftY())
                        / (controller.back().getAsBoolean() ? 2 : 1.5),
            () ->
                arm.armPosition == Arm.ARM_LEVEL_STOW
                    ? regulate(-controller.getLeftX()) / (controller.back().getAsBoolean() ? 2 : 1)
                    : regulate(-controller.getLeftX())
                        / (controller.back().getAsBoolean() ? 2 : 1.5),
            () ->
                // isHeadingLock
                //     ? regulate(
                //             turnPIDController.calculate(
                //                 drive.getGyroIO().getRobotHeading(),
                //                 drive.getGyroIO().getRobotHeading() //
                //                     + getClosestAngle(
                //                         heading - drive.getGyroIO().getRobotHeading())))
                //         / (controller.back().getAsBoolean() ? 2 : 1.5)
                //     :
                controller.getRightX() / (controller.back().getAsBoolean() ? 2 : 1.5)));

    // controller
    // .start()
    // .onTrue(
    // Commands.runOnce(
    // () ->
    // drive.setPose(
    // new Pose2d(drive.getPose().getTranslation(), new
    // Rotation2d(3.14/2))),
    // drive)
    // .ignoringDisable(true));

    // high hang
    controller
        .back()
        .and(controller.start())
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
            Commands.startEnd(
                    () ->
                        intake.runVelocity(
                            arm.armPosition == Arm.ARM_LEVEL_STATION_INTAKE
                                ? INTAKE_ROLLER_SPEED
                                : getFlywheelRPM() * 2),
                    () -> intake.stop(),
                    intake)
                .alongWith(
                    Commands.startEnd(
                        () -> flywheel.runVelocity(getFlywheelRPM()),
                        () -> flywheel.stop(),
                        flywheel)));

    // set robot heading
    // controller
    //     .start()
    //     .and(controller.leftTrigger())
    //     .onTrue(Commands.runOnce(() -> setRobotHeading(45)));
    // controller
    //     .start()
    //     .and(controller.leftBumper())
    //     .onTrue(Commands.runOnce(() -> setRobotHeading(-90)));
    // controller
    //     .back()
    //     .and(controller.rightTrigger())
    //     .onTrue(Commands.runOnce(() -> setRobotHeading(180)));
    // controller
    //     .back()
    //     .and(controller.rightBumper())
    //     .onTrue(Commands.runOnce(() -> setRobotHeading(180)));

    // Left Bumper, Arm to Feeder
    controller
        .leftTrigger()
        .whileTrue(
            Commands.runOnce(() -> arm.setArmLevel(Arm.ARM_LEVEL_STATION_INTAKE))
                .alongWith(Commands.runOnce(() -> flywheel.stop()))
                .alongWith(Commands.runOnce(() -> intake.runVelocity(INTAKE_ROLLER_SPEED))))
        .whileFalse(
            Commands.runOnce(() -> arm.setArmLevel(Arm.ARM_LEVEL_STOW))
                .alongWith(Commands.runOnce(() -> flywheel.stop()))
                .alongWith(Commands.runOnce(() -> intake.stop()))
                .andThen(() -> arm.stop()));

    // Right Trigger, Arm to AMP
    controller
        .leftBumper()
        .whileTrue(
            Commands.runOnce(() -> arm.setArmLevel(Arm.ARM_LEVEL_AMP))
                .alongWith(Commands.runOnce(() -> flywheel.runVelocity(getFlywheelRPM()))))
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
                .andThen(Commands.runOnce(() -> flywheel.runVelocity(getFlywheelRPM()))))
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
                .andThen(Commands.runOnce(() -> flywheel.runVelocity(getFlywheelRPM()))))
        .whileFalse(
            Commands.runOnce(() -> arm.setArmLevel(Arm.ARM_LEVEL_STOW))
                .alongWith(Commands.runOnce(() -> flywheel.stop()))
                .alongWith(Commands.runOnce(() -> intake.stop()))
                .andThen(() -> arm.stop()));
  }

  //   private void setRobotHeading(double target) {
  //     // isHeadingLock = true;
  //     heading = target;
  //   }
  public void speakerNote() {
    arm.setArmLevel(Arm.ARM_LEVEL_SPEAKER);
    // Timer.delay(1);
    flywheel.runVelocity(getFlywheelRPM());
    intake.runVelocity(RobotContainer.INTAKE_ROLLER_SPEED);
    // Timer.delay(500);

    // arm.setArmLevel(Arm.ARM_LEVEL_STOW);
    // flywheel.stop();
    // intake.stop();
    // // Timer.delay(1000);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // return autoChooser.get();
  // }
  public Command getAutonomousCommand() {
    // Translation2d m_translation = this.drive.getPose().getTranslation();

    Rotation2d m_rotation = new Rotation2d();
    // Load the path you want to follow using its name in the GUI
    Pose2d initialPose2D = PathPlannerPath.fromPathFile("Path1").getPreviewStartingHolonomicPose();
    drive.setPose(initialPose2D);

    PathPlannerPath path = PathPlannerPath.fromPathFile("Path1");
    // PathPlannerPath path = PathPlannerAuto. .getPathGroupFromAutoFile("MyAuto").get(0);

    // return Commands.runOnce(() -> flywheel.stop())
    //     .andThen(() -> intake.stop())
    //     .andThen(() -> arm.setArmLevel(Arm.ARM_LEVEL_SPEAKER))
    //     .andThen(new WaitCommand(1))
    //     .andThen(() -> flywheel.runVelocity(getFlywheelRPM()))
    //     .andThen(() -> intake.runVelocity(INTAKE_ROLLER_SPEED))
    //     .andThen(new WaitCommand(0.8))
    //     .andThen(() -> arm.setArmLevel(Arm.ARM_LEVEL_STOW))
    //     .andThen(new WaitCommand(1))
    //     .andThen(() -> flywheel.stop())
    //     .andThen(() -> intake.stop());
    // .andThen(AutoBuilder.followPath(path));

    return AutoBuilder.followPath(path);

    // return Commands.runOnce(() -> speakerNote()).andThen(new WaitCommand(1));
  }
}
