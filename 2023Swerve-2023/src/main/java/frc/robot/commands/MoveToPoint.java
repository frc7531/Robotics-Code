package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class MoveToPoint extends CommandBase {
    // private final HolonomicDriveController hdc;
    private final Pose2d target;
    private final SwerveDrive swerve;

    private final ProfiledPIDController xController;
    private final ProfiledPIDController yController;
    private final ProfiledPIDController thetaController;

    public MoveToPoint(SwerveDrive swerve, Pose2d target) {
        xController = new ProfiledPIDController(1, 0.1, 0, new Constraints(1, 1.5));
        yController = new ProfiledPIDController(1, 0.1, 0, new Constraints(1, 1.5));
        thetaController = new ProfiledPIDController(1, 0, 0, new Constraints(1, 1.5));
        
        // hdc = new HolonomicDriveController(xController, yController, thetaController);
        // hdc.setTolerance(new Pose2d(0.01, 0.01, Rotation2d.fromDegrees(1)));
        xController.setTolerance(0.01);
        yController.setTolerance(0.01);
        thetaController.setTolerance(0.05);

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        this.target = target;
        this.swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        swerve.getGyro().reset();
        swerve.setOdometer(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    }

    @Override
    public void execute() {
        swerve.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
                xController.calculate(swerve.getPose().getX(), target.getX()),
                yController.calculate(swerve.getPose().getY(), target.getY()),
                thetaController.calculate(MathUtil.angleModulus(swerve.getPose().getRotation().getRadians()), target.getRotation().getRadians())
            ),
            swerve.getGyro().getRotation2d()
        ));
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}