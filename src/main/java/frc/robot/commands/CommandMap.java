package frc.robot.commands;

import java.util.Map;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

    public class CommandMap {
        public static final Map<String, Command> namedCommands = Map.of(
            "setCoralL2", new SetWristAndElevator(RobotContainer robot, 2),
            "DriveForward", new DriveDistance(driveSubsystem, 2.0),
            "SimplePath", AutoBuilder.followPath(PathPlanner.loadPath("SimplePath", 2.0, 2.0)),
            "ComplexPath", AutoBuilder.followPath(PathPlanner.loadPath("ComplexPath", 3.0, 3.0)),
            "DoNothing", Commands.none() // Empty command
        );
    }