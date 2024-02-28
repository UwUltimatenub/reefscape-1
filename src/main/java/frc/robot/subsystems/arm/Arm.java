// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.superstructure.arm.ArmIOInputsAutoLogged;
import frc.robot.util.Alert;
import frc.robot.util.EqualsUtil;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm {
  private static final LoggedTunableNumber kP =
      new LoggedTunableNumber("Arm/kP", ArmConstants.gains.kP());
  private static final LoggedTunableNumber kI =
      new LoggedTunableNumber("Arm/kI", ArmConstants.gains.kI());
  private static final LoggedTunableNumber kD =
      new LoggedTunableNumber("Arm/kD", ArmConstants.gains.kD());
  private static final LoggedTunableNumber kS =
      new LoggedTunableNumber("Arm/kS", ArmConstants.gains.ffkS());
  private static final LoggedTunableNumber kV =
      new LoggedTunableNumber("Arm/kV", ArmConstants.gains.ffkV());
  private static final LoggedTunableNumber kA =
      new LoggedTunableNumber("Arm/kA", ArmConstants.gains.ffkA());
  private static final LoggedTunableNumber kG =
      new LoggedTunableNumber("Arm/kG", ArmConstants.gains.ffkG());
  private static final LoggedTunableNumber maxVelocity =
      new LoggedTunableNumber("Arm/Velocity", ArmConstants.profileConstraints.maxVelocity);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Arm/Acceleration", ArmConstants.profileConstraints.maxAcceleration);
  private static final LoggedTunableNumber lowerLimitDegrees =
      new LoggedTunableNumber("Arm/LowerLimitDegrees", ArmConstants.minAngle.getDegrees());
  private static final LoggedTunableNumber upperLimitDegrees =
      new LoggedTunableNumber("Arm/UpperLimitDegrees", ArmConstants.maxAngle.getDegrees());

  @RequiredArgsConstructor
  public enum Goal {
    STATION_INTAKE(new LoggedTunableNumber("Arm/StationIntakeDegrees", 35)),
    AIM(new LoggedTunableNumber("Arm/StationIntakeDegrees", 30.0)),
    STOW(new LoggedTunableNumber("Arm/StowDegrees", -30.0)),
    AMP(new LoggedTunableNumber("Arm/AmpDegrees", 20.0)),
    SUBWOOFER(new LoggedTunableNumber("Arm/SubwooferDegrees", 30.0)),
    CUSTOM(new LoggedTunableNumber("Arm/CustomSetpoint", 20.0));

    private final DoubleSupplier armSetpointSupplier;

    private double getRads() {
      return Units.degreesToRadians(armSetpointSupplier.getAsDouble());
    }
  }

  @Getter @Setter private Goal goal = Goal.STOW;
  private boolean characterizing = false;

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private TrapezoidProfile motionProfile;
  private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();
  private ArmFeedforward ff;

  private final ArmVisualizer measuredVisualizer;
  private final ArmVisualizer setpointVisualizer;
  private final ArmVisualizer goalVisualizer;

  private final Alert leaderMotorDisconnected =
      new Alert("Arm leader motor disconnected!", Alert.AlertType.WARNING);
  private final Alert followerMotorDisconnected =
      new Alert("Arm follower motor disconnected!", Alert.AlertType.WARNING);
  private final Alert absoluteEncoderDisconnected =
      new Alert("Arm absolute encoder disconnected!", Alert.AlertType.WARNING);

  public Arm(ArmIO io) {
    this.io = io;
    io.setBrakeMode(true);

    motionProfile =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(maxVelocity.get(), maxAcceleration.get()));
    io.setPID(kP.get(), kI.get(), kD.get());
    ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get());

    // Set up visualizers
    measuredVisualizer = new ArmVisualizer("Measured", Color.kBlack);
    setpointVisualizer = new ArmVisualizer("Setpoint", Color.kGreen);
    goalVisualizer = new ArmVisualizer("Goal", Color.kBlue);
  }

  public void periodic() {
    // Process inputs
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    // Set alerts
    leaderMotorDisconnected.set(!inputs.leaderMotorConnected);
    followerMotorDisconnected.set(!inputs.followerMotorConnected);
    absoluteEncoderDisconnected.set(!inputs.absoluteEncoderConnected);

    // Update controllers
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setPID(kP.get(), kI.get(), kD.get()), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        () -> ff = new ArmFeedforward(kS.get(), kG.get(), kV.get(), kA.get()),
        kS,
        kG,
        kV,
        kA);

    LoggedTunableNumber.ifChanged(
        hashCode(),
        constraints ->
            motionProfile =
                new TrapezoidProfile(
                    new TrapezoidProfile.Constraints(constraints[0], constraints[1])),
        maxVelocity,
        maxAcceleration);

    if (!characterizing) {
      // Run closed loop
      setpointState =
          motionProfile.calculate(
              2,
              setpointState,
              new TrapezoidProfile.State(
                  MathUtil.clamp(
                      goal.getRads(),
                      Units.degreesToRadians(lowerLimitDegrees.get()),
                      Units.degreesToRadians(upperLimitDegrees.get())),
                  0.0));

      io.runSetpoint(
          setpointState.position, ff.calculate(setpointState.position, setpointState.velocity));
    }

    if (DriverStation.isDisabled()) {
      io.stop();
      // Reset profile when disabled
      setpointState = new TrapezoidProfile.State(inputs.armPositionRads, 0);
    }

    // Logs
    measuredVisualizer.update(inputs.armPositionRads);
    setpointVisualizer.update(setpointState.position);
    goalVisualizer.update(goal.getRads());
    Logger.recordOutput("Arm/GoalAngle", goal.getRads());
    Logger.recordOutput("Arm/SetpointAngle", setpointState.position);
    Logger.recordOutput("Arm/SetpointVelocity", setpointState.velocity);
    Logger.recordOutput("Arm/Goal", goal);
  }

  public void stop() {
    io.stop();
  }

  @AutoLogOutput(key = "Arm/AtGoal")
  public boolean atGoal() {
    return EqualsUtil.epsilonEquals(setpointState.position, goal.getRads(), 1e-3);
  }

  public Command getStaticCurrent() {
    Timer timer = new Timer();
    return Commands.run(() -> io.runCurrent(0.5 * timer.get()))
        .beforeStarting(
            () -> {
              characterizing = true;
              timer.restart();
            })
        .until(() -> Math.abs(inputs.armVelocityRadsPerSec) >= Units.degreesToRadians(10))
        .andThen(() -> Logger.recordOutput("Arm/staticCurrent", inputs.armTorqueCurrentAmps[0]))
        .finallyDo(
            () -> {
              io.stop();
              characterizing = false;
            });
  }
}
