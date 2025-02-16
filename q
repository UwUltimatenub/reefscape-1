[1mdiff --git a/src/main/java/frc/robot/subsystems/pivot/Wrist.java b/src/main/java/frc/robot/subsystems/pivot/Wrist.java[m
[1mindex 882d3e4..56cdcf3 100644[m
[1m--- a/src/main/java/frc/robot/subsystems/pivot/Wrist.java[m
[1m+++ b/src/main/java/frc/robot/subsystems/pivot/Wrist.java[m
[36m@@ -25,12 +25,11 @@[m [mimport edu.wpi.first.wpilibj2.command.SubsystemBase;[m
 import org.littletonrobotics.junction.AutoLog;[m
 [m
 public class Wrist extends SubsystemBase {[m
[31m-  public static final double reduction = 3.0;[m
[31m-  private static final Rotation2d offset = new Rotation2d();[m
[32m+[m[32m  public static final double reduction = 18.689; //58/10*58/18[m
[32m+[m[32m  private static final Rotation2d offset = new Rotation2d(60);//default wrist angle[m
   private static final int encoderId = 0;[m
[31m-  public static final Rotation2d minAngle = Rotation2d.fromDegrees(-140.0);[m
[31m-  public static final Rotation2d maxAngle = Rotation2d.fromDegrees(160.0);[m
[31m-  private static final double ARM_VELOCITY = 1;[m
[32m+[m[32m  public static final Rotation2d minAngle = Rotation2d.fromDegrees(60);[m
[32m+[m[32m  public static final Rotation2d maxAngle = Rotation2d.fromDegrees(240.0);[m
 [m
   // Hardware[m
   private final TalonFX talon;[m
