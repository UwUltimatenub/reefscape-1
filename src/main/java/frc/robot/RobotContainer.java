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
import java.util.HashMap;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public static final boolean USE_MAGTAG_II = true;
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

  Command cmd = null;

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
    double output = input; // Math.signum(input) * input * input;
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
                .withInterruptBehavior(InterruptionBehavior.kCancelSelf));

    // // level 1 state, depend on is coral loaded
    // controller2
    //     .b()
    //     .onTrue(
    //         new SetWristAndElevator(this, 1)
    //             .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    elevator.setDefaultCommand(Commands.run(() -> {}, elevator));
    wrist.setDefaultCommand(Commands.run(() -> {}, wrist));

    setCameraCommand();
  }

  private void setCameraCommand() {

    controller
        .start()
        .onTrue(
            Commands.runOnce(
                () -> {
                  if (cmd != null) {
                    cmd.cancel();
                  }
                },
                drive));

    // 1. Align robot to Level 4 left
    controller
        .leftTrigger()
        .onTrue(
            new InstantCommand(
                    () -> {
                      var path = GoReefTarget(true, 4, 1);
                      if (path != null) {
                        cmd = AutoBuilder.followPath(path);
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

    // 2. Align robot to Level 4 Right
    controller
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                    () -> {
                      var path = GoReefTarget(false, 4, 1);
                      if (path != null) {
                        cmd = AutoBuilder.followPath(path);
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

    // 3. Align robot to Level 3 left
    controller
        .povLeft()
        .onTrue(
            new InstantCommand(
                    () -> {
                      var path = GoReefTarget(true, 3, 1);
                      if (path != null) {
                        cmd = AutoBuilder.followPath(path);
                        drive.isTracking = true;
                        cmd.schedule();
                      }
                    })
                .alongWith(
                    new SetWristAndElevator(this, 3)
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
                .andThen(
                    new InstantCommand(
                        () -> {
                          drive.isTracking = false;
                        })));

    // 4. Align robot to Level 3 Right
    controller
        .povUp()
        .onTrue(
            new InstantCommand(
                    () -> {
                      var path = GoReefTarget(false, 3, 1);
                      if (path != null) {
                        cmd = AutoBuilder.followPath(path);
                        drive.isTracking = true;
                        cmd.schedule();
                      }
                    })
                .alongWith(
                    new SetWristAndElevator(this, 3)
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
                .andThen(
                    new InstantCommand(
                        () -> {
                          drive.isTracking = false;
                        })));
    // 5. Align robot to Level 2 left
    controller
        .povDown()
        .onTrue(
            new InstantCommand(
                    () -> {
                      var path = GoReefTarget(true, 2, 1);
                      if (path != null) {
                        cmd = AutoBuilder.followPath(path);
                        drive.isTracking = true;
                        cmd.schedule();
                      }
                    })
                .alongWith(
                    new SetWristAndElevator(this, 2)
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
                .andThen(
                    new InstantCommand(
                        () -> {
                          drive.isTracking = false;
                        })));

    // 6. Align robot to Level 2 Right
    controller
        .povRight()
        .onTrue(
            new InstantCommand(
                    () -> {
                      var path = GoReefTarget(false, 2, 1);
                      if (path != null) {
                        cmd = AutoBuilder.followPath(path);
                        drive.isTracking = true;
                        cmd.schedule();
                      }
                    })
                .alongWith(
                    new SetWristAndElevator(this, 2)
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
                .andThen(
                    new InstantCommand(
                        () -> {
                          drive.isTracking = false;
                        })));

    // 7. Align robot to Level 1 left
    controller
        .x()
        .onTrue(
            new InstantCommand(
                    () -> {
                      var path = GoReefTarget(true, 1, 1);
                      if (path != null) {
                        cmd = AutoBuilder.followPath(path);
                        drive.isTracking = true;
                        cmd.schedule();
                      }
                    })
                .alongWith(
                    new SetWristAndElevator(this, 1)
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
                .andThen(
                    new InstantCommand(
                        () -> {
                          drive.isTracking = false;
                        })));

    // 8. Align robot to Level 1 right
    controller
        .b()
        .onTrue(
            new InstantCommand(
                    () -> {
                      var path = GoReefTarget(false, 1, 1);
                      if (path != null) {
                        cmd = AutoBuilder.followPath(path);
                        drive.isTracking = true;
                        cmd.schedule();
                      }
                    })
                .alongWith(
                    new SetWristAndElevator(this, 1)
                        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming))
                .andThen(
                    new InstantCommand(
                        () -> {
                          drive.isTracking = false;
                        })));
  }

  /////
  ///
  public static Rotation2d getForwardRotation(Pose2d pose1, Pose2d pose2) {
    // Calculate the angle between the two positions (deltaY / deltaX)
    double deltaX = pose2.getX() - pose1.getX();
    double deltaY = pose2.getY() - pose1.getY();
    double angleBetween = Math.atan2(deltaY, deltaX);

    // Calculate the relative rotation by subtracting pose1's rotation from the angle between the
    // two poses
    Rotation2d forwardRotation = new Rotation2d(angleBetween);

    return forwardRotation;
  }

  ///
  public PathPlannerPath createPath(Pose2d fromPose2d, Pose2d targetPose2d, int route) {
    Pose2d startWayPoint = fromPose2d;
    Pose2d endWayPoint = targetPose2d;
    // if (route != 2) { // make all route starighter, except route 2
    startWayPoint =
        new Pose2d(fromPose2d.getTranslation(), getForwardRotation(fromPose2d, targetPose2d));
    endWayPoint =
        new Pose2d(targetPose2d.getTranslation(), getForwardRotation(fromPose2d, targetPose2d));
    // } else {
    //   startWayPoint =
    //       new Pose2d(
    //           fromPose2d.getTranslation(), // smoother curve to the source
    //           Rotation2d.fromDegrees(
    //               (getForwardRotation(fromPose2d, targetPose2d).getDegrees()
    //                       + targetPose2d.getRotation().getDegrees())
    //                   / 2));
    //   endWayPoint = targetPose2d;
    // }
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startWayPoint, endWayPoint);

    PathConstraints constraints =
        new PathConstraints(
            route == 1 || route == 3 || route == 5 ? 2 : 4.5,
            route == 1 || route == 3 || route == 5 ? 2.2 : 4,
            2 * Math.PI,
            4 * Math.PI); // The constraints for this
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

  // auto Alignment for driver
  public PathPlannerPath GoReefTarget(boolean isLeft, int level, int route) {

    if ((level == 4 || level == 1) && !intake.isCoralLoaded()) {
      return null;
    }
    if (LimelightHelpers.getTV("limelight")) {

      drive.estimatePose(drive, USE_MAGTAG_II);
    }

    Pose2d targetPose2d = vision.getTargetPose2D(isLeft, level, intake.isCoralLoaded());
    if (targetPose2d == null) {
      return null;
    }

    Pose2d fromPose2d =
        new Pose2d(drive.getPose().getX(), drive.getPose().getY(), drive.getPose().getRotation());
    return createPath(fromPose2d, targetPose2d, route);
  }

  public final HashMap<String, PathPlannerPath> AUTO_PATH = new HashMap<String, PathPlannerPath>();

  void setAutoPath() {

    // blue_middle
    AUTO_PATH.put("blue_middle1", getPath("blue_middle", 1));
    // red_middle
    AUTO_PATH.put("red_middle1", getPath("red_middle", 1));

    // blue_left
    AUTO_PATH.put("blue_left1", getPath("blue_left", 1));
    AUTO_PATH.put("blue_left2", getPath("blue_left", 2));
    AUTO_PATH.put("blue_left3", getPath("blue_left", 3));
    AUTO_PATH.put("blue_left4", getPath("blue_left", 4));
    AUTO_PATH.put("blue_left5", getPath("blue_left", 5));
    AUTO_PATH.put("blue_left6", getPath("blue_left", 6));
    AUTO_PATH.put("blue_left7", getPath("blue_left", 7));
    AUTO_PATH.put("blue_left8", getPath("blue_left", 8));

    // blue_right
    AUTO_PATH.put("blue_right1", getPath("blue_right", 1));
    AUTO_PATH.put("blue_right2", getPath("blue_right", 2));
    AUTO_PATH.put("blue_right3", getPath("blue_right", 3));
    AUTO_PATH.put("blue_right4", getPath("blue_right", 4));
    AUTO_PATH.put("blue_right5", getPath("blue_right", 5));
    AUTO_PATH.put("blue_right6", getPath("blue_right", 6));
    AUTO_PATH.put("blue_right7", getPath("blue_right", 7));
    AUTO_PATH.put("blue_right8", getPath("blue_right", 8));

    // red_left
    AUTO_PATH.put("red_left1", getPath("red_left", 1));
    AUTO_PATH.put("red_left2", getPath("red_left", 2));
    AUTO_PATH.put("red_left3", getPath("red_left", 3));
    AUTO_PATH.put("red_left4", getPath("red_left", 4));
    AUTO_PATH.put("red_left5", getPath("red_left", 5));
    AUTO_PATH.put("red_left6", getPath("red_left", 6));
    AUTO_PATH.put("red_left7", getPath("red_left", 7));
    AUTO_PATH.put("red_left8", getPath("red_left", 8));

    // red_right
    AUTO_PATH.put("red_right1", getPath("red_right", 1));
    AUTO_PATH.put("red_right2", getPath("red_right", 2));
    AUTO_PATH.put("red_right3", getPath("red_right", 3));
    AUTO_PATH.put("red_right4", getPath("red_right", 4));
    AUTO_PATH.put("red_right5", getPath("red_right", 5));
    AUTO_PATH.put("red_right6", getPath("red_right", 6));
    AUTO_PATH.put("red_right7", getPath("red_right", 7));
    AUTO_PATH.put("red_right8", getPath("red_right", 8));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(int path) {

    String autoName = autoChooser.get().getName();
    Command autoCommand = null;

    if (path == 1) {
      drive.setPose(getInitialPose(autoName));
      autoCommand =
          AutoBuilder.followPath(AUTO_PATH.get(autoName + 1)) // path1: go to reef 1
              .alongWith((Commands.waitSeconds(0.25)).andThen(new SetWristAndElevator(this, 4)));

    } else if (path == 2) { // path2 shoot, go to source

      autoCommand = gotoSource(autoName + 2);

    } else if (path == 3) {

      autoCommand = sourceToReef(autoName + 3); // path3: source to reef 2

    } else if (path == 4) {

      autoCommand = gotoSource(autoName + 4); // path4: shoot go to source

    } else if (path == 5) {

      autoCommand = sourceToReef(autoName + 5); // path5: source to reef 3

    } else if (path == 6) {

      autoCommand = gotoSource(autoName + 6); // path6: shoot go to source

    } else if (path == 7) {

      autoCommand = sourceToReef(autoName + 7); // path7: source to reef 4

    } else if (path == 8) {

      autoCommand = gotoSource(autoName + 8); // path8: shoot go to source
    }

    return autoCommand;
  }

  private Command gotoSource(String autoName) {
    return new IntakeCommand(this, false)
        .withTimeout(1)
        .alongWith(
            Commands.waitSeconds(0.2).andThen(AutoBuilder.followPath(AUTO_PATH.get(autoName))))
        .alongWith(Commands.waitSeconds(0.25).andThen(new SetWristAndElevator(this, 0)));
  }

  private Command sourceToReef(String path) {

    return (new IntakeCommand(this, true)
            .andThen(Commands.waitSeconds(0.25).andThen(new SetWristAndElevator(this, 4))))
        .alongWith(Commands.waitSeconds(0.35).andThen(AutoBuilder.followPath(AUTO_PATH.get(path))));
  }

  private Pose2d getInitialPose(String autoName) {
    Pose2d startPose2d = null;

    switch (autoName) {
      case "blue_middle":
        startPose2d = new Pose2d(6, 3.88, Rotation2d.fromDegrees(0));
        break;
      case "blue_left":
        startPose2d = new Pose2d(7, 6.17, Rotation2d.fromDegrees(45));
        break;
      case "blue_right":
        startPose2d = new Pose2d(7, 1.87, Rotation2d.fromDegrees(-45));
        break;
      case "red_middle":
        startPose2d = new Pose2d(10.4, 3.88, Rotation2d.fromDegrees(180));
        break;
      case "red_left":
        startPose2d = new Pose2d(10.55, 1.87, Rotation2d.fromDegrees(-135));
        break;
      case "red_right":
        startPose2d = new Pose2d(10.55, 6.17, Rotation2d.fromDegrees(135));
        break;

      default:
        break;
    }

    return startPose2d;
  }

  private Pose2d getPosition(String autoName, int position) {

    Pose2d targetPose2d = null;
    switch (position) {
      case 0: // source
        switch (autoName) {
          case "blue_left":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("13");
            break;
          case "blue_right":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("12");
            break;
          case "red_left":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("1");
            break;
          case "red_right":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("2");
            break;
        }
        break;
      case 1: // first target
        switch (autoName) {
          case "blue_middle":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("21R5");
            break;
          case "blue_left":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("20R5");
            break;
          case "blue_right":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("22L5");
            break;
          case "red_middle":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("10R5");
            break;
          case "red_left":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("11R5");
            break;
          case "red_right":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("9L5");
            break;
        }
        break;
      case 2:
        // second target
        switch (autoName) {
          case "blue_left":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("19R5");
            break;
          case "blue_right":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("17L5");
            break;
          case "red_left":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("6R5");
            break;
          case "red_right":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("8L5");
            break;
        }
        break;
      case 3:
        // third target
        switch (autoName) {
          case "blue_left":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("19L5");
            break;
          case "blue_right":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("17R5");
            break;
          case "red_left":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("6L5");
            break;
          case "red_right":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("8R5");
            break;
        }
      case 4:
        // third target
        switch (autoName) {
          case "blue_left":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("18L5");
            break;
          case "blue_right":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("18R5");
            break;
          case "red_left":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("7L5");
            break;
          case "red_right":
            targetPose2d = vision.APRILTAG_TARGET_POSE.get("7R5");
            break;
        }
        break;
    }
    return targetPose2d;
  }

  private PathPlannerPath getPath(String autoName, int number) {

    Pose2d from = null;
    Pose2d to = null;
    switch (number) {
      case 1: // to first reef
        from = getInitialPose(autoName);
        to = getPosition(autoName, 1);
        break;
      case 2: // to first source
        from = getPosition(autoName, 1);
        to = getPosition(autoName, 0);
        break;
      case 3: // to second reef
        from = getPosition(autoName, 0);
        to = getPosition(autoName, 2);
        break;
      case 4: // to second source
        from = getPosition(autoName, 2);
        to = getPosition(autoName, 0);
        break;
      case 5: // to third reff
        from = getPosition(autoName, 0);
        to = getPosition(autoName, 3);
        break;
      case 6: // to third source
        from = getPosition(autoName, 3);
        to = getPosition(autoName, 0);
        break;
      case 7: // to fourth reff
        from = getPosition(autoName, 0);
        to = getPosition(autoName, 4);
        break;
      case 8: // to fourth source
        from = getPosition(autoName, 4);
        to = getPosition(autoName, 0);
        break;
    }
    return createPath(from, to, number);
  }
}
