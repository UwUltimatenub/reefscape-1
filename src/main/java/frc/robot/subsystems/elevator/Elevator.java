// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SuperStructureState;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  public static final double ELEVATOR_GEAR_REDUCTION = 6.0;
  public static final double ELEVATOR_SPROCKET_PERIMETER =
      Units.inchesToMeters(5.5) * 100; // inches

  // Hardware
  private final TalonFX talon;
  private final TalonFX followerTalon;

  @AutoLog
  public static class ElevatorIOInputs {
    public boolean motorConnected = true;
    public double motorPosition = 0;
    public double elevatorHeight = 0;
  }

  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  // Config
  private final TalonFXConfiguration config = new TalonFXConfiguration();

  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0.0).withUpdateFreqHz(0.0);
  public static final double minHeight = 0;
  public static final double maxHeight = 15;

  public Elevator() {
    talon = new TalonFX(11, "*");
    followerTalon = new TalonFX(15, "*");
    followerTalon.setControl(new Follower(talon.getDeviceID(), false));
    talon.setNeutralMode(NeutralModeValue.Brake);

    // Configure motor
    // Configure motor
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    config.Slot0 = new Slot0Configs().withKP(50).withKI(0).withKD(0);
    config.Feedback.SensorToMechanismRatio = ELEVATOR_GEAR_REDUCTION;
    config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
    config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
    config.CurrentLimits.StatorCurrentLimit = 80.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

    // ParentDevice.optimizeBusUtilizationForAll(talon);
  }

  public void setVoltage(double voltage) {
    // Set the power to the main motor
    talon.setControl(new VoltageOut(voltage));
  }

  // Periodic method called in every cycle (e.g., 20ms)
  @Override
  public void periodic() {
    inputs.motorConnected = talon.isConnected();
    inputs.motorPosition = talon.getPosition().getValueAsDouble();
    inputs.elevatorHeight = inputs.motorPosition * ELEVATOR_SPROCKET_PERIMETER;
    Logger.processInputs("Elevator", inputs);
    // if elevator height > height1 and wrist angle < angle1, stop elevator motor
  }

  public double getElevatorHeight() {
    // Get the position from the encoder
    return talon.getPosition().getValueAsDouble() * ELEVATOR_SPROCKET_PERIMETER;
  }

  public double getVelocity() {
    // Get the velocity from the encoder
    return talon.getVelocity().getValueAsDouble();
  }

  public void resetPosition() {
    // Reset the encoder to the specified position
    talon.setPosition(0);
  }

  public void setElevatorHeight(SuperStructureState state) {

    // Get target position (in radians)
    double targetHeight = MathUtil.clamp(state.height, minHeight, maxHeight);
    // Feedforward Model (Tune These Values)
    ElevatorFeedforward feedforward = new ElevatorFeedforward(0.0, 20, 0, 0);
    talon.setControl(
        positionTorqueCurrentRequest
            .withPosition(
                targetHeight / ELEVATOR_SPROCKET_PERIMETER) // perimeter of pinion gear in centmeter
            .withFeedForward(feedforward.calculate(0)));
    // if not working try this...
    // talon.setControl(new PositionVoltage(targetHeight /
    // ELEVATOR_SPROCKET_PERIMETER).withFeedForward(ffOutput));

  }

  public void stop() {
    talon.stopMotor();
  }

  public void setBrakeMode(boolean enabled) {
    new Thread(
            () -> talon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast))
        .start();
  }
}
