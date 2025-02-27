// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

public class SuperStructureState {

  public static final double SOURCE_HEIGHT = 0;
  public static final double L2_HEIGHT = 5.5;
  public static final double L3_HEIGHT = 21.5;
  public static final double L4_HEIGHT = 52.5;
  public static final double LOW_ALGAE_HEIGHT = 10;
  public static final double MID_ALGAE_HEIGHT = 10;
  public static final double TOP_ALGAE_HEIGHT = 40;

  public static final double SOURCE_ANGLE = 50;
  public static final double L2_ANGLE = 75;
  public static final double L3_ANGLE = 75;
  public static final double L4_ANGLE = 125;

  public static final double PROCESSOR_ANGLE = 210;
  public static final double LOW_MID_ALGAE_ANGLE = 200;
  public static final double TOP_ALGAE_ANGLE = 180;

  public double height;
  public double angle;

  public SuperStructureState(double height, double angle) {
    this.height = height;
    this.angle = angle;
  }

  public static SuperStructureState STATE_SOURCE =
      new SuperStructureState(SOURCE_HEIGHT, SOURCE_ANGLE);
  public static SuperStructureState STATE_L2 = new SuperStructureState(L2_HEIGHT, L2_ANGLE);
  public static SuperStructureState STATE_L3 = new SuperStructureState(L3_HEIGHT, L3_ANGLE);
  public static SuperStructureState STATE_L4 = new SuperStructureState(L4_HEIGHT, L4_ANGLE);
  public static SuperStructureState STATE_SAFTY =
      new SuperStructureState(LOW_MID_ALGAE_ANGLE, L2_ANGLE); // angle to pass the safty zone

  public static SuperStructureState STATE_PROCESSOR =
      new SuperStructureState(L2_HEIGHT, PROCESSOR_ANGLE);
  public static SuperStructureState STATE_ALGAE_LOW =
      new SuperStructureState(LOW_ALGAE_HEIGHT, LOW_MID_ALGAE_ANGLE);
  public static SuperStructureState STATE_ALGAE_MID =
      new SuperStructureState(MID_ALGAE_HEIGHT, LOW_MID_ALGAE_ANGLE);
  public static SuperStructureState STATE_ALGAE_TOP =
      new SuperStructureState(TOP_ALGAE_HEIGHT, TOP_ALGAE_ANGLE);
}
