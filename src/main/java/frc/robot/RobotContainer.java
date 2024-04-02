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
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  public static final double INTAKE_ROLLER_SPEED = 500; // RPM
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
        flywheelRPM = 300; // used to be 400
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
  double calculateTurnpower(double targetAngle) {
    double power = 0;
    System.out.println("asd=" + targetAngle + "   " + drive.getPose().getRotation().getDegrees());
    targetAngle = targetAngle - drive.getPose().getRotation().getDegrees();
    targetAngle = (targetAngle + 3600) % 360;
    targetAngle = targetAngle > 180 ? -(360 - targetAngle) : targetAngle;
    power = targetAngle / 15;
    power = Math.abs(power) > 0.5 ? Math.signum(power) * 0.5 : power;
    return power;
  }

  // map joystick input to curved output
  // less sensitive at the low range, very sensentive at high range
  private double regulate(double input) {
    double output = input;
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      output = -input;
    }
    return output;
  }

  private double getTurnPower() {
    double targetAngle = 0;
    double power = 0;
    if (controller.start().getAsBoolean() && controller.leftTrigger().getAsBoolean()) {
      // start + leftTrigger -> face to Source
      targetAngle = DriverStation.getAlliance().get() == Alliance.Blue ? -60 : -120;
      power = calculateTurnpower(targetAngle);
    } else if (controller.start().getAsBoolean() && controller.leftBumper().getAsBoolean()) {
      // start + leftBumper -> face to Amp
      targetAngle = 90;
      power = calculateTurnpower(targetAngle);
    } else if (controller.start().getAsBoolean() && controller.y().getAsBoolean()) {
      // start + y -> face to back
      targetAngle = DriverStation.getAlliance().get() == Alliance.Blue ? 180 : 0;
      power = calculateTurnpower(targetAngle);
    } else if (controller.start().getAsBoolean() && controller.x().getAsBoolean()) {
      // start + x -> face to left hang
      targetAngle = DriverStation.getAlliance().get() == Alliance.Blue ? -60 : 120;
      power = calculateTurnpower(targetAngle);
    } else if (controller.start().getAsBoolean() && controller.b().getAsBoolean()) {
      // start + b -> face to right hang
      targetAngle = DriverStation.getAlliance().get() == Alliance.Blue ? 60 : -120;
      power = calculateTurnpower(targetAngle);
    } else {
      power = -controller.getRightX() / 1.35;
    }
    return power;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // a + start -> resetFieldOrientation
    controller
        .a()
        .and(controller.start())
        .onTrue(
            Commands.runOnce(
                () ->
                    drive.setPose(
                        new Pose2d(
                            2.0,
                            5.0,
                            DriverStation.getAlliance().get() == Alliance.Blue
                                ? new Rotation2d(0)
                                : new Rotation2d(Math.PI)))));

    // drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () ->
                arm.armPosition == Arm.ARM_LEVEL_STOW
                    ? regulate(-controller.getLeftY())
                    : regulate(-controller.getLeftY()) / 1.5,
            () ->
                arm.armPosition == Arm.ARM_LEVEL_STOW
                    ? regulate(-controller.getLeftX())
                    : regulate(-controller.getLeftX()) / 1.5,
            () -> getTurnPower()));

    // back + start -> high hang
    controller
        .back()
        .and(controller.start())
        .whileTrue(
            Commands.runOnce(() -> arm.setArmLevel(Arm.ARM_LEVEL_HANG))
                .alongWith(Commands.runOnce(() -> intake.stop()))
                .alongWith(Commands.runOnce(() -> flywheel.stop())))
        .whileFalse(
            Commands.runOnce(() -> arm.setArmLevel(Arm.ARM_LEVEL_STOW)).andThen(() -> arm.stop()));

    // a -> intake
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

    // Left Bumper, Arm to Feeder
    controller
        .leftTrigger()
        .and(controller.start().negate())
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
        .and(controller.start().negate())
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  // public Command getAutonomousCommand() {
  // return autoChooser.get();
  // }
  public Command getAutonomousCommand() {

    // Command command = getPath();
    Command command = getAuto();
    return command;
  }

  public static final String Path_File = "ampside begin";
  // public static final String Path_File = "ampside push";
  public Command getPath() {
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile(Path_File);

    // get preview initial pose
    Pose2d initialPose2D = path.getPreviewStartingHolonomicPose();

    // // the path is design from Blue Alliance prospect, filp path and initial start point when
    // from
    // // Red.
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      path.flipPath();
      initialPose2D = GeometryUtil.flipFieldPose(initialPose2D);
    }
    drive.setPose(initialPose2D);

    return Commands.runOnce(() -> flywheel.stop())
        .andThen(() -> intake.stop())
        .andThen(() -> arm.setArmLevel(Arm.ARM_LEVEL_SPEAKER + 2))
        .andThen(new WaitCommand(0.8))
        .andThen(() -> flywheel.runVelocity(getFlywheelRPM()))
        .andThen(() -> intake.runVelocity(INTAKE_ROLLER_SPEED))
        .andThen(new WaitCommand(0.6))
        .andThen(() -> arm.setArmLevel(Arm.ARM_LEVEL_STOW))
        .andThen(() -> flywheel.stop())
        .andThen(() -> intake.stop())
        .andThen(AutoBuilder.followPath(path));
  }

  //   public static final String Auto_File = "AmpAmp";
  public static final String Auto_File = "Ampside 12";
  // public static final String Auto_File = "Ampside 21";
  // public static final String Auto_File = "Sourceside 12";
  // public static final String Auto_File = "Sourceside 21";
  // public static final String Auto_File = "Speaker 3";

  public Command getAuto() {
    // Load the path you want to follow using its name in the GUI
    // PathPlannerPath path = PathPlannerPath.fromPathFile("ampside push");
    // get preview initial pose
    Pose2d initialPose2D = PathPlannerAuto.getStaringPoseFromAutoFile(Auto_File);

    // // the path is design from Blue Alliance prospect, filp path and initial start point when
    // from Red.
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      initialPose2D = GeometryUtil.flipFieldPose(initialPose2D);
    }
    drive.setPose(initialPose2D);

    return Commands.runOnce(() -> flywheel.stop())
        .andThen(() -> intake.stop())
        .andThen(() -> arm.setArmLevel(Arm.ARM_LEVEL_SPEAKER))
        .andThen(new WaitCommand(0.8))
        .andThen(() -> flywheel.runVelocity(getFlywheelRPM()))
        .andThen(() -> intake.runVelocity(INTAKE_ROLLER_SPEED))
        .andThen(new WaitCommand(0.4))
        .andThen(() -> arm.setArmLevel(Arm.ARM_LEVEL_STOW))
        .andThen(() -> flywheel.stop())
        .andThen(() -> intake.stop())
        .andThen(AutoBuilder.buildAuto(Auto_File));
  }
}
