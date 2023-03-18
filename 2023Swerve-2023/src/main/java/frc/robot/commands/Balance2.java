package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.AutoBalance;
import frc.robot.subsystems.SwerveDrive;

public class Balance2 extends CommandBase {
    private final AutoBalance m_balancer;
    private final SwerveDrive m_swerve;

    public Balance2(SwerveDrive swerve) {
        m_balancer = new AutoBalance();
        m_swerve = swerve;

        addRequirements(swerve);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        m_swerve.drive(new ChassisSpeeds(0, m_balancer.autoBalanceRoutine(), 0));
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
