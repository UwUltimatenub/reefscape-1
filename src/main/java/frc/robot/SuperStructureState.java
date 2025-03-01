// Copyright (c) 2025 FRC 9785
// https://github.com/tonytigr/reefscape
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

public class SuperStructureState {

  public static final double SOURCE_HEIGHT = 0.5;
  public static final double SOURCE_ANGLE = 50;

  public static final double L1_HEIGHT = 5;
  public static final double L2_HEIGHT = 12;
  public static final double L3_HEIGHT = 33;
  public static final double L4_HEIGHT = 72;

  public static final double L1_ANGLE = 55;
  public static final double L2_ANGLE = 75;
  public static final double L3_ANGLE = 75;
  public static final double L4_ANGLE = 125;

  public static final double PROCESSOR_HEIGHT = 10;
  public static final double LOW_ALGAE_HEIGHT = 26;
  public static final double MID_ALGAE_HEIGHT = 40;
  public static final double TOP_ALGAE_HEIGHT = 75;

  public static final double PROCESSOR_ANGLE = 210;
  public static final double LOW_ALGAE_ANGLE = 240;
  public static final double MID_ALGAE_ANGLE = 240;
  public static final double TOP_ALGAE_ANGLE = 180;

  // angle to pass the safty zone
  public static SuperStructureState STATE_SAFTY = new SuperStructureState("Safety", 200, 75);
  public static SuperStructureState STATE_SOURCE =
      new SuperStructureState("Source", SOURCE_HEIGHT, SOURCE_ANGLE);

  public static SuperStructureState STATE_L1 = new SuperStructureState("CL1", L1_HEIGHT, L1_ANGLE);
  public static SuperStructureState STATE_L2 = new SuperStructureState("CL2", L2_HEIGHT, L2_ANGLE);
  public static SuperStructureState STATE_L3 = new SuperStructureState("CL3", L3_HEIGHT, L3_ANGLE);
  public static SuperStructureState STATE_L4 = new SuperStructureState("CL4", L4_HEIGHT, L4_ANGLE);

  public static SuperStructureState STATE_PROCESSOR =
      new SuperStructureState("PROCESSOR", PROCESSOR_HEIGHT, PROCESSOR_ANGLE);
  public static SuperStructureState STATE_ALGAE_LOW =
      new SuperStructureState("AL2", LOW_ALGAE_HEIGHT, LOW_ALGAE_ANGLE);
  public static SuperStructureState STATE_ALGAE_MID =
      new SuperStructureState("AL3", MID_ALGAE_HEIGHT, MID_ALGAE_ANGLE);
  public static SuperStructureState STATE_ALGAE_TOP =
      new SuperStructureState("AL4", TOP_ALGAE_HEIGHT, TOP_ALGAE_ANGLE);

  public double height;
  public double angle;
  public String name;

  public SuperStructureState(String name, double height, double angle) {
    this.name = name;
    this.height = height;
    this.angle = angle;
  }
}
