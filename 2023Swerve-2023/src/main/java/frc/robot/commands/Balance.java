package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class Balance extends CommandBase {
    private final BuiltInAccelerometer accel;
    private final PIDController pid;

    private final SwerveDrive swerve;

    public Balance(SwerveDrive swerve) {
        accel = new BuiltInAccelerometer();
        pid = new PIDController(4, 0, 0);
        pid.setTolerance(0.05);
        this.swerve = swerve;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        double x = accel.getX();
        double z = accel.getZ();

        double calc = pid.calculate(z, 0.98);

        ChassisSpeeds speeds = new ChassisSpeeds(0, Math.copySign(calc, -x), 0);

        System.out.println(speeds);

        swerve.drive(speeds, 1, 0);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
