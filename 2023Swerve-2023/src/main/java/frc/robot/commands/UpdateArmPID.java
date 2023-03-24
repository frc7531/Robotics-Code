package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shoulder;
import frc.robot.subsystems.Telescope;

public class UpdateArmPID extends CommandBase {
    private final Shoulder shoulder;
    private final Telescope telescope;
    private final Joystick control;

    private double shoulderSetPoint;
    private double telescopeSetPoint;

    private boolean cone = true;
    private boolean cube = false;

    public UpdateArmPID(Shoulder shoulder, Telescope telescope, Joystick controller) {
        this.shoulder = shoulder;
        this.telescope = telescope;
        control = controller;
        addRequirements(shoulder);
    }

    @Override
    public void initialize() {

    }

    public void setCone() {
        cone = true;
        cube = false;
    }

    public void setCube() {
        cube = true;
        cone = false;
    }

    @Override
    public void execute() {
        if(cone) {
            if(control.getRawButton(1)) {
                shoulderSetPoint = 0.06;
                telescopeSetPoint = -600;
            } else if(control.getRawButton(2)) {
                shoulderSetPoint = 0.24;
                telescopeSetPoint = 0;
            } else if(control.getRawButton(4)) {
                shoulderSetPoint = 0.30625;
                telescopeSetPoint = -1300;
            }
        } else if(cube) {
            if(control.getRawButton(1)) {
                shoulderSetPoint = 0.06;
                telescopeSetPoint = -600;
            } else if(control.getRawButton(2)) {
                shoulderSetPoint = 0.19;
                telescopeSetPoint = -200;
            } else if(control.getRawButton(4)) {
                shoulderSetPoint = 0.266;
                telescopeSetPoint = -1300;
            }
        } if(control.getRawButton(8)) {
            telescope.reset();
            shoulderSetPoint = 0;
            telescopeSetPoint = 2000;
        } if(control.getRawButton(3)) {
            shoulderSetPoint = 0.237;
            telescopeSetPoint = 0;
        }
        shoulder.setHeight(shoulderSetPoint - control.getRawAxis(1) / 10);
        telescope.setPosition(telescopeSetPoint + control.getRawAxis(5) * 500);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
