// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PointTowardsZone;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import frc.robot.subsystems.vision.LimelightHelpers;
import java.util.Arrays;
import java.util.List;
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

    NamedCommands.registerCommand("setSource", new SetWristAndElevator(this, 0).withTimeout(5));
    NamedCommands.registerCommand("setL1", new SetWristAndElevator(this, 1).withTimeout(5));
    NamedCommands.registerCommand("setL2", new SetWristAndElevator(this, 2).withTimeout(5));
    NamedCommands.registerCommand("setL3", new SetWristAndElevator(this, 3).withTimeout(5));
    NamedCommands.registerCommand("setL4", new SetWristAndElevator(this, 4).withTimeout(8));

    NamedCommands.registerCommand("intake", new IntakeCommand(this, true).withTimeout(5));
    NamedCommands.registerCommand("eject", new IntakeCommand(this, false).withTimeout(5));

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();
  }

  // map joystick input to curved output
  // less sensitive at the low range, very sensentive at high range
  private double regulate(double input) {
    double output = Math.signum(input) * input * input;
    if (targetState == SuperStructureState.STATE_ALGAE_TOP
        || currentState == SuperStructureState.STATE_ALGAE_TOP
        || targetState == SuperStructureState.STATE_L4) {
      output = output / 1.9;
    } else if (targetState == SuperStructureState.STATE_ALGAE_MID
        || targetState == SuperStructureState.STATE_ALGAE_LOW
        || targetState == SuperStructureState.STATE_L2
        || targetState == SuperStructureState.STATE_L3
        || currentState == SuperStructureState.STATE_L4) {
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
        .start()
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
            () -> regulate(-controller.getRightX() / 1.1)));

    // Intake coral/algae
    controller2.leftBumper().whileTrue(new IntakeCommand(this, true));
    // Eject coral/algae; coral go to source after ejection
    controller2.rightBumper().whileTrue(new IntakeCommand(this, false));
    // .whileFalse(new SetWristAndElevator(this, 0)).and(() ->
    // !intake.isCoralLoaded()).and(()->currentState.name.startsWith("Coral"));

    // Source State
    controller2
        .a()
        .onTrue(
            new SetWristAndElevator(this, 0)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // level 1 state, depend on is coral loaded
    controller2
        .povDown()
        .onTrue(
            new SetWristAndElevator(this, 1)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // level 2 state, depend on is coral loaded
    controller2
        .povLeft()
        .onTrue(
            new SetWristAndElevator(this, 2)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // level 3 state, depend on is coral loaded
    controller2
        .povUp()
        .onTrue(
            new SetWristAndElevator(this, 3)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    // level 4 state, depend on is coral loaded
    controller2
        .povRight()
        .onTrue(
            new SetWristAndElevator(this, 4)
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    elevator.setDefaultCommand(Commands.run(() -> {}, elevator));
    wrist.setDefaultCommand(Commands.run(() -> {}, wrist));

    setCameraCommand();
  }

  private void setCameraCommand() {

    // Align robot to april tag
    controller
        .leftTrigger()
        .onTrue(
            new InstantCommand(
                    () -> {
                      var path = GoReefTarget(true, true);
                      if (path != null) {
                        var cmd = AutoBuilder.followPath(path);
                        drive.isTracking = true;
                        cmd.schedule();
                      }
                    })
                .alongWith(
                    new SetWristAndElevator(this, 4)
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
                .andThen(
                    new InstantCommand(
                        () -> {
                          drive.isTracking = false;
                        })));

    // Align robot to april tag
    controller
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                    () -> {
                      var path = GoReefTarget(false, true);
                      if (path != null) {
                        var cmd = AutoBuilder.followPath(path);
                        drive.isTracking = true;
                        cmd.schedule();
                      }
                    })
                    .alongWith(
                      new SetWristAndElevator(this, 4)
                          .withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
                  .andThen(
                    new InstantCommand(
                        () -> {
                          drive.isTracking = false;
                        })));

    // Align robot to april tag
    controller
        .x()
        .onTrue(
            new InstantCommand(
                    () -> {
                      var path = GoReefTarget(true, false);
                      if (path != null) {
                        var cmd = AutoBuilder.followPath(path);
                        drive.isTracking = true;
                        cmd.schedule();
                      }
                    })
                .andThen(
                    new InstantCommand(
                        () -> {
                          drive.isTracking = false;
                        })));

    // Align robot to april tag
    controller
        .b()
        .onTrue(
            new InstantCommand(
                    () -> {
                      var path = GoReefTarget(false, false);
                      if (path != null) {
                        var cmd = AutoBuilder.followPath(path);
                        drive.isTracking = true;
                        cmd.schedule();
                      }
                    })
                .andThen(
                    new InstantCommand(
                        () -> {
                          drive.isTracking = false;
                        })));
    // Align robot to april tag
    // controller
    //     .y()
    //     .onTrue(
    //         new InstantCommand(
    //                 () -> {
    //                   var path = GoReefTarget(false, false);
    //                   if (path != null) {
    //                     var cmd = AutoBuilder.followPath(path);
    //                     drive.isTracking = true;
    //                     cmd.schedule();
    //                   }
    //                 })
    //             .andThen(
    //                 new InstantCommand(
    //                     () -> {
    //                       drive.isTracking = false;
    //                     })));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    String autoName = "middle1";
    // autoChooser.get().getName();
    Command autoCommand = null;
    switch (autoName) {
      case "middle1":
        autoCommand =
            AutoBuilder.followPath(GoTarget1())
                .andThen(new SetWristAndElevator(this, 4))
                .andThen(new IntakeCommand(this, false))
                .andThen(new SetWristAndElevator(this, 0))
                .andThen(AutoBuilder.followPath(GoSource()))
                .andThen(new IntakeCommand(this, true))
                // .until(() -> intake.isCoralLoaded())
                .andThen(AutoBuilder.followPath(GoReefTarget(true, true)))
                .andThen(new SetWristAndElevator(this, 4))
                .andThen(new IntakeCommand(this, false));
        // .alongWith(new SetWristAndElevator(this, 0))
        // .alongWith(AutoBuilder.followPath(GoSource()))
        // .andThen(new IntakeCommand(this, true))
        // .until(() -> intake.isCoralLoaded())
        // .andThen(AutoBuilder.followPath(GoReefTarget(false, true)))
        // .alongWith(new SetWristAndElevator(this, 4))
        // .andThen(new IntakeCommand(this, false))
        // .alongWith(new SetWristAndElevator(this, 0));

        break;

      default:
        break;
    }
    return autoCommand;
  }

  private PathPlannerPath GoTarget1() {

    Pose2d currentPose2d = drive.getPose();
    Pose2d targetPose2d = null;
    if (currentPose2d.getX() < 8.7) {
      // blue
      if (currentPose2d.getY() < 4) {
        // lower id=12
        targetPose2d = vision.APRILTAG_TARGET_POSE.get("22R5");
      } else {
        // upper id=13
        targetPose2d = vision.APRILTAG_TARGET_POSE.get("20L5");
      }

    } else {
      // red
      if (currentPose2d.getY() < 4) {
        // lower id=1
        targetPose2d = vision.APRILTAG_TARGET_POSE.get("11R5");
      } else {
        // upper id=2
        targetPose2d = vision.APRILTAG_TARGET_POSE.get("9L5");
      }
    }
    return GoToPoint(drive.getPose(), targetPose2d);
  }

  private PathPlannerPath GoTarget2() {

    Pose2d currentPose2d = drive.getPose();
    Pose2d targetPose2d = null;
    if (currentPose2d.getX() < 8.7) {
      // blue
      if (currentPose2d.getY() < 4) {
        // lower id=12
        targetPose2d = vision.APRILTAG_TARGET_POSE.get("17L5");
      } else {
        // upper id=13
        targetPose2d = vision.APRILTAG_TARGET_POSE.get("19R5");
      }

    } else {
      // red
      if (currentPose2d.getY() < 4) {
        // lower id=1
        targetPose2d = vision.APRILTAG_TARGET_POSE.get("6L5");
      } else {
        // upper id=2
        targetPose2d = vision.APRILTAG_TARGET_POSE.get("8R5");
      }
    }
    return GoToPoint(drive.getPose(), targetPose2d);
  }

  // 7m 1.9m -30
  private PathPlannerPath GoSource() {

    Pose2d currentPose2d = drive.getPose();
    Pose2d targetPose2d = null;
    if (currentPose2d.getX() < 8.7) {
      // blue
      if (currentPose2d.getY() < 4) {
        // lower id=12
        targetPose2d = vision.APRILTAG_TARGET_POSE.get("12");
      } else {
        // upper id=13
        targetPose2d = vision.APRILTAG_TARGET_POSE.get("13");
      }

    } else {
      // red
      if (currentPose2d.getY() < 4) {
        // lower id=1
        targetPose2d = vision.APRILTAG_TARGET_POSE.get("1");
      } else {
        // upper id=2
        targetPose2d = vision.APRILTAG_TARGET_POSE.get("2");
      }
    }
    return GoToPoint(drive.getPose(), targetPose2d);
  }

  public PathPlannerPath GoReefTarget(boolean isLeft, boolean isLevel5) {

    if (LimelightHelpers.getTV("limelight")) {
      drive.setPose(LimelightHelpers.getBotPose2d_wpiBlue("limelight"));
    }

    Pose2d targetPose2d = vision.getTargetPose2D(isLeft, isLevel5);
    if (targetPose2d == null) {
      return null;
    }

    Pose2d fromPose2d =
        new Pose2d(drive.getPose().getX(), drive.getPose().getY(), drive.getPose().getRotation());
    return GoToPoint(fromPose2d, targetPose2d);
  }

  public PathPlannerPath GoToPoint(Pose2d fromPose2d, Pose2d targetPose2d) {

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(fromPose2d, targetPose2d);

    PathConstraints constraints =
        new PathConstraints(1.5, 2.2, 2 * Math.PI, 4 * Math.PI); // The constraints for this
    // path.
    // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); //
    // You can also use unlimited constraints, only limited by motor torque and
    // nominal battery voltage

    List<EventMarker> ListEM = Arrays.asList();
    List<RotationTarget> ListRT = Arrays.asList();
    List<ConstraintsZone> ListCZ = Arrays.asList();
    List<PointTowardsZone> ListPTZ = Arrays.asList();

    // Create the path using the waypoints created above
    PathPlannerPath path =
        new PathPlannerPath(
            waypoints,
            ListRT,
            ListPTZ,
            ListCZ,
            ListEM,
            constraints,
            null, // The ideal starting state, this is only relevant for pre-planned paths, so can
            // be null for on-the-fly paths.
            new GoalEndState(
                0.0,
                targetPose2d
                    .getRotation()), // Goal end state. You can set a holonomic rotation here. If
            // using a differential drivetrain, the rotation will have no
            // effect.
            false);

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping = true;

    return path;
  }
}
