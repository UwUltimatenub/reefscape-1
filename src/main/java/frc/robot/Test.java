package frc.robot;

import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class Test {

    private static final double offset_side = -0.3;
    private static final double offset_forward = 0.2;
  private final static HashMap<String, Pose2d> APRILTAG_RADIAN = new HashMap<String, Pose2d>();

    // private final AprilTagFieldLayout APRILTAGFIELDLAYOUT =
    public static final AprilTagFieldLayout TAG_LAYOUT = AprilTagFieldLayout
            .loadField(AprilTagFields.k2025ReefscapeWelded);


  private static final Transform2d transform_left =
      new Transform2d(offset_forward ,offset_side,  Rotation2d.fromDegrees(0));
  private static final Transform2d transform_right =
      new Transform2d(offset_forward, -offset_side, Rotation2d.fromDegrees(0));

 
    public static void main(String[] args) {
        System.out.println("hello");



        APRILTAG_RADIAN.put(
            "6L", TAG_LAYOUT.getTagPose(6).get().toPose2d().transformBy(transform_left));
        APRILTAG_RADIAN.put(
            "7L", TAG_LAYOUT.getTagPose(7).get().toPose2d().transformBy(transform_left));
        APRILTAG_RADIAN.put(
            "8L", TAG_LAYOUT.getTagPose(8).get().toPose2d().transformBy(transform_left));
        APRILTAG_RADIAN.put(
            "9L", TAG_LAYOUT.getTagPose(9).get().toPose2d().transformBy(transform_left));
        APRILTAG_RADIAN.put(
            "10L", TAG_LAYOUT.getTagPose(10).get().toPose2d().transformBy(transform_left));
        APRILTAG_RADIAN.put(
            "11L", TAG_LAYOUT.getTagPose(11).get().toPose2d().transformBy(transform_left));
    
        APRILTAG_RADIAN.put(
            "17L", TAG_LAYOUT.getTagPose(17).get().toPose2d().transformBy(transform_left));
        APRILTAG_RADIAN.put(
            "18L", TAG_LAYOUT.getTagPose(18).get().toPose2d().transformBy(transform_left));
        APRILTAG_RADIAN.put(
            "19L", TAG_LAYOUT.getTagPose(19).get().toPose2d().transformBy(transform_left));
        APRILTAG_RADIAN.put(
            "20L", TAG_LAYOUT.getTagPose(20).get().toPose2d().transformBy(transform_left));
        APRILTAG_RADIAN.put(
            "21L", TAG_LAYOUT.getTagPose(21).get().toPose2d().transformBy(transform_left));
        APRILTAG_RADIAN.put(
            "22L", TAG_LAYOUT.getTagPose(22).get().toPose2d().transformBy(transform_left));
    
        APRILTAG_RADIAN.put(
            "6R", TAG_LAYOUT.getTagPose(6).get().toPose2d().transformBy(transform_right));
        APRILTAG_RADIAN.put(
            "7R", TAG_LAYOUT.getTagPose(7).get().toPose2d().transformBy(transform_right));
        APRILTAG_RADIAN.put(
            "8R", TAG_LAYOUT.getTagPose(8).get().toPose2d().transformBy(transform_right));
        APRILTAG_RADIAN.put(
            "9R", TAG_LAYOUT.getTagPose(9).get().toPose2d().transformBy(transform_right));
        APRILTAG_RADIAN.put(
            "10R", TAG_LAYOUT.getTagPose(10).get().toPose2d().transformBy(transform_right));
        APRILTAG_RADIAN.put(
            "11R", TAG_LAYOUT.getTagPose(11).get().toPose2d().transformBy(transform_right));
    
        APRILTAG_RADIAN.put(
            "17R", TAG_LAYOUT.getTagPose(17).get().toPose2d().transformBy(transform_right));
        APRILTAG_RADIAN.put(
            "18R", TAG_LAYOUT.getTagPose(18).get().toPose2d().transformBy(transform_right));
        APRILTAG_RADIAN.put(
            "19R", TAG_LAYOUT.getTagPose(19).get().toPose2d().transformBy(transform_right));
        APRILTAG_RADIAN.put(
            "20R", TAG_LAYOUT.getTagPose(20).get().toPose2d().transformBy(transform_right));
        APRILTAG_RADIAN.put(
            "21R", TAG_LAYOUT.getTagPose(21).get().toPose2d().transformBy(transform_right));
        APRILTAG_RADIAN.put(
            "22R", TAG_LAYOUT.getTagPose(22).get().toPose2d().transformBy(transform_right));
String asd="7L";

            System.out.println("hello" + APRILTAG_RADIAN.get(asd).getX()+"|" +APRILTAG_RADIAN.get(asd).getY()+"|"+APRILTAG_RADIAN.get(asd).getRotation());


    }
}
