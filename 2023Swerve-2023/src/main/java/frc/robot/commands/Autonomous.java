package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveDrive;

public class Autonomous {
    public static Command getCommand(SwerveDrive swerve, List<Pose2d> poses, boolean turnAround) {
        Trajectory trajectory;

        // Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);

        // try {
        //     trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        // } catch (IOException e) {
        //     e.printStackTrace();
        //     System.out.println("ERROR 320: Filepath not valid, please try another");
        //     return new InstantCommand(() -> {});
        // }

        trajectory = TrajectoryGenerator.generateTrajectory(
            poses, new TrajectoryConfig(1, 0.5)
        );

        PIDController xController = new PIDController(0.4, 0, 0);
        PIDController yController = new PIDController(-0.4, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
            0.6, 0, 0, new TrapezoidProfile.Constraints(0.5, 0.3)
        );

        // thetaController.setTolerance(0.01);
        // xController.setTolerance(0.01);
        // yController.setTolerance(0.01);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        HolonomicDriveController ctrl = new HolonomicDriveController(xController, yController, thetaController);

        Command swerveCommand = new SwerveControllerCommand(
            trajectory,
            swerve::getPose,
            swerve.getKinematics(),
            ctrl,
            // Autonomous::updateTheta,
            swerve::setModuleStates,
            swerve
        );

        Pose2d starting = trajectory.getInitialPose();
        
        return new SequentialCommandGroup(
            new InstantCommand(() -> swerve.setOdometer(starting)),
            new InstantCommand(() -> SmartDashboard.putString("status", "Running")),
            // new InstantCommand(() -> swerve.setModuleStates(new SwerveModuleState[] {
            //     new SwerveModuleState(0.1, Rotation2d.fromDegrees(0)),
            //     new SwerveModuleState(0.1, Rotation2d.fromDegrees(0)),
            //     new SwerveModuleState(0.1, Rotation2d.fromDegrees(0)),
            //     new SwerveModuleState(0.1, Rotation2d.fromDegrees(0))
            // })),
            swerveCommand,
            new InstantCommand(() -> SmartDashboard.putString("status", "Done")),
            new InstantCommand(() -> swerve.stopMotors())
        );
    }

    // static Rotation2d updateTheta() {
    //     //return new Rotation2d(classic.getRawAxis(2), classic.getRawAxis(3));
    //     return Rotation2d.fromDegrees(180);
    // }
}