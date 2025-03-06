// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommand;
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
  public final Drive drive;
  private final LimeLight vision;
  public final Intake intake;
  public final Wrist wrist;
  public final Elevator elevator;

  public SuperStructureState currentState = SuperStructureState.STATE_SOURCE;
  public SuperStructureState targetState = SuperStructureState.STATE_SOURCE;

  // Controller
  public final CommandXboxController controller = new CommandXboxController(0);
  public final CommandXboxController controller2 = controller; // new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Real robot, instantiate hardware IO implementations
    vision = new LimeLight();
    wrist = new Wrist();
    elevator = new Elevator();
    intake = new Intake();
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

    NamedCommands.registerCommand("setSource", new SetWristAndElevator(this, 0));
    NamedCommands.registerCommand("setL1", new SetWristAndElevator(this, 1));
    NamedCommands.registerCommand("setL2", new SetWristAndElevator(this, 2));
    NamedCommands.registerCommand("setL3", new SetWristAndElevator(this, 3));
    NamedCommands.registerCommand("setL4", new SetWristAndElevator(this, 4));

    NamedCommands.registerCommand("intake", new IntakeCommand(this, true));
    NamedCommands.registerCommand("eject", new IntakeCommand(this, false));
  }

  // map joystick input to curved output
  // less sensitive at the low range, very sensentive at high range
  private double regulate(double input) {
    double output = Math.signum(input) * input * input;
    if (targetState == SuperStructureState.STATE_ALGAE_TOP
        || currentState == SuperStructureState.STATE_ALGAE_TOP
        || targetState == SuperStructureState.STATE_L4
        || currentState == SuperStructureState.STATE_L4) {
      output = output / 1.75;
    } else if (targetState == SuperStructureState.STATE_ALGAE_MID
        || targetState == SuperStructureState.STATE_ALGAE_LOW
        || targetState == SuperStructureState.STATE_L2
        || targetState == SuperStructureState.STATE_L3) {
      output = output / 1.4;
    } else {
      // processor and source, move normal speed
      if (intake.intakeStatus == IntakeCommand.Intake_Coral) {
        output = output / 1.5;
      }
    }
    // if (DriverStation.getAlliance().get() == Alliance.Red) {
    // output = -input;
    // }
    return output;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Reset gyro
    controller
        .y()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> regulate(-controller.getLeftY()),
            () -> regulate(-controller.getLeftX()),
            () -> regulate(-controller.getRightX() / 1.2)));

    // Intake coral/algae
    controller2.leftBumper().whileTrue(new IntakeCommand(this, true));
    // Eject coral/algae
    controller2.rightBumper().whileTrue(new IntakeCommand(this, false));

    // Source State
    controller2.a().onTrue(new SetWristAndElevator(this, 0));
    // level 1 state, depend on is coral loaded
    controller2.povDown().onTrue(new SetWristAndElevator(this, 1));
    // level 2 state, depend on is coral loaded
    controller2.povLeft().onTrue(new SetWristAndElevator(this, 2));
    // level 3 state, depend on is coral loaded
    controller2.povUp().onTrue(new SetWristAndElevator(this, 3));
    // level 4 state, depend on is coral loaded
    controller2.povRight().onTrue(new SetWristAndElevator(this, 4));

    // testing
    // controller2.x().onTrue(new RunCommand(() ->
    // wrist.setWristAngle(SuperStructureState.L2_ANGLE)));
    // controller2
    // .b()
    // .onTrue(new RunCommand(() ->
    // wrist.setWristAngle(SuperStructureState.SOURCE_ANGLE)));
    elevator.setDefaultCommand(Commands.run(() -> {}, elevator));
    wrist.setDefaultCommand(Commands.run(() -> {}, wrist));

    setCameraCommand();
  }

  private void setCameraCommand() {
    // Point robot to april tag
    controller
        .leftTrigger()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> regulate(-controller.getLeftY()),
                () -> regulate(-controller.getLeftX()),
                () -> new Rotation2d(Units.degreesToRadians(vision.autoRotate()))));

    // Align robot to april tag
    controller
        .rightTrigger()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> vision.autoTranslateY(),
                () -> vision.autoTranslateX(),
                () -> new Rotation2d(Units.degreesToRadians(vision.autoRotate()))));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String autoName = autoChooser.get().getName();
    Command autoCommand = null;
    switch (autoName) {
      case "left_auto":
        autoCommand =
            autoChooser
                .get()
                .andThen(new SetWristAndElevator(this, 4))
                .andThen(new IntakeCommand(this, false))
                .andThen(new SetWristAndElevator(this, 0));
        break;

      default:
        break;
    }
    return autoCommand;
  }
}
