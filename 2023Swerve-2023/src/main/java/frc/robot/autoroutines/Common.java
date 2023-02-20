package frc.robot.autoroutines;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveDrive;

public class Common {
    PIDController xController = new PIDController(0.4, 0, 0);
    PIDController yController = new PIDController(0.4, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
      1, 0, 0, new TrapezoidProfile.Constraints(1, 0.5)
    );

    SwerveDrive swerve;


    public Common(SwerveDrive swerve) {
        xController.setTolerance(0.001);
        yController.setTolerance(0.001);
        thetaController.setTolerance(0.001);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.swerve = swerve;
    }

    public Command createSwerveCommand(Trajectory trajectory, Supplier<Rotation2d> desiredRotation) {
        return new SwerveControllerCommand(
            trajectory,
            swerve::getPose,
            swerve.getKinematics(),
            xController,
            yController,
            thetaController,
            desiredRotation,
            swerve::setModuleStates,
            swerve
        );
    }
}
