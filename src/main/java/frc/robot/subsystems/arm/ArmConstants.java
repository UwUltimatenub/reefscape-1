// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class ArmConstants {
  // reduction is 12:62 18:60 12:65
  public static double reduction = (62.0 / 12.0) * (60.0 / 18.0) * (65.0 / 12.0);
  public static Rotation2d positionTolerance = Rotation2d.fromDegrees(3.0);
  public static Translation2d armOrigin = new Translation2d(-0.238, 0.298);
  public static Rotation2d minAngle = Rotation2d.fromDegrees(-30.0);
  public static Rotation2d maxAngle = Rotation2d.fromDegrees(80.0);

  public static double armLength = Units.inchesToMeters(25.866);

  public static Gains gains = new Gains(4000, 0.0, 120, 5.75, 0.0, 0.0, 15);

  public static TrapezoidProfile.Constraints profileConstraints =
      new TrapezoidProfile.Constraints(2 * Math.PI, 10);

  public record Gains(
      double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}
