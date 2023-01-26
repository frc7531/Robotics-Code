package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveDrive;

public class PointToTarget extends CommandBase {
    private final CameraSubsystem cameraSystem;
    private final SwerveDrive swerve;
    private final PIDController rotationController;
    private final PIDController moveXController;
    private final PIDController yawController;

    public PointToTarget(CameraSubsystem cameraSystem, SwerveDrive swerve) {
        this.cameraSystem = cameraSystem;
        this.swerve = swerve;
        rotationController = new PIDController(0.3, 0, 0.0);
        moveXController = new PIDController(-1, 0, 0);
        yawController = new PIDController(0.3, 0, 0);
        yawController.enableContinuousInput(-Math.PI, Math.PI);
        yawController.setTolerance(0.05);

        addRequirements(cameraSystem);
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        Transform3d target = cameraSystem.getTagLocation(0);
        // if(target == null || cameraSystem.getReliability(0) < 0.03) {
        //     swerve.drive(new ChassisSpeeds(0, 0, 0), 1, 0);
        //     return;
        // }
        double rotation = rotationController.calculate(target.getY() / target.getX(), 0);
        double yaw = yawController.calculate(target.getRotation().getZ(), Math.PI);
        double moveX = moveXController.calculate(target.getX(), 1);
        SmartDashboard.putNumber("rotation value", rotation);
        swerve.drive(new ChassisSpeeds(yaw, moveX, -2 * rotation), 1, 0);
        SmartDashboard.putString("rotation of target", 
            "(" + Rotation2d.fromRadians(target.getRotation().getX()).getDegrees() + ", " +
            Rotation2d.fromRadians(target.getRotation().getY()).getDegrees() + ", " +
            Rotation2d.fromRadians(target.getRotation().getZ()).getDegrees() + ")");
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
