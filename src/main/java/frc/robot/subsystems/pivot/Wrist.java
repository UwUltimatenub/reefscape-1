// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  public static final Rotation2d minAngle = Rotation2d.fromDegrees(-140.0);
  public static final Rotation2d maxAngle = Rotation2d.fromDegrees(160.0);

  // Hardware
  private final WristIO pivotIO;
  private final WristIOInputsAutoLogged pivotInputs = new WristIOInputsAutoLogged();

  public Wrist(WristIO pivotIO) {
    this.pivotIO = pivotIO;
  }

  public void periodic() {
    pivotIO.updateInputs(pivotInputs);
  }

  public void wristAngle(Rotation2d position) {
    pivotIO.runPosition(position);
  }

  public Object wristAngle(double pROCESSOR_ANGLE) {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'wristAngle'");
  }
}
