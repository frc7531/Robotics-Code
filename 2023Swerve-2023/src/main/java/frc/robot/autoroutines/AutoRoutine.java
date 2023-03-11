package frc.robot.autoroutines;

import java.util.HashMap;
import java.util.List;
import java.util.Set;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SwerveDrive;

public class AutoRoutine {
    public static Command getSwerveCommand(SwerveDrive swerve, String autoName, HashMap<String, Command> eventMap) {
        // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
        // for every path in the group
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(autoName, 1, 1, true);

        // This is just an example event map. It would be better to have a constant, global event map
        // in your code that will be used by all path following commands.
        // eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        // eventMap.put("intakeDown", new IntakeDown());

        // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
            swerve::getPose, // Pose2d supplier
            swerve::setOdometer, // Pose2d consumer, used to reset odometry at the beginning of auto
            swerve.getKinematics(), // SwerveDriveKinematics
            new PIDConstants(0.3, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
            new PIDConstants(0.05, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
            swerve::setModuleStates, // Module states consumer used to output to the drive subsystem
            eventMap,
            true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
            swerve // The drive subsystem. Used to properly set the requirements of path following commands
        );

        Command fullAuto = autoBuilder.fullAuto(pathGroup);

        return fullAuto;
    }
}