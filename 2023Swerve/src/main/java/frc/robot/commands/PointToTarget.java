package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.SwerveDrive;

public class PointToTarget extends CommandBase {
    private final CameraSubsystem cameraSystem;
    private final SwerveDrive swerve;
    private final PIDController controller;

    public PointToTarget(CameraSubsystem cameraSystem, SwerveDrive swerve) {
        this.cameraSystem = cameraSystem;
        this.swerve = swerve;
        controller = new PIDController(0.3, 0, 0);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
