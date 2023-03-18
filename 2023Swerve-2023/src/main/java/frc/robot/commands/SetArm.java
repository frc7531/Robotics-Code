package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Telescope;

public class SetArm extends CommandBase {
    private final Shoulder m_shoulder;
    private final Telescope m_telescope;

    private final double m_targetHeight;
    private final double m_targetExtension;

    public SetArm(Shoulder shoulder, Telescope telescope, double targetHeight, double targetExtension) {
        m_shoulder = shoulder;
        m_telescope = telescope;

        m_targetHeight = targetHeight;
        m_targetExtension = targetExtension;

        addRequirements(shoulder, telescope);
    }

    @Override
    public void initialize() {
        m_shoulder.setHeight(m_targetHeight);
        m_telescope.setPosition(m_targetExtension);
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return m_shoulder.atSetpoint() && m_telescope.atSetpoint();
    }
}
