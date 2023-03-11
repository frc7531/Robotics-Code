package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shoulder;

public class UpdateArmPID extends CommandBase {
    private final Shoulder shoulder;
    private final Joystick control;

    private double setpoint;

    public UpdateArmPID(Shoulder shoulder, Joystick controller) {
        this.shoulder = shoulder;
        control = controller;
        addRequirements(shoulder);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
               if(control.getRawButton(1)) {        // Low level            (A)
            setpoint = Shoulder.LOW_TARGET_HEIGHT;  //                      30°
        } else if(control.getRawButton(2)) {        // Mid level            (B)
            setpoint = Shoulder.MID_TARGET_HEIGHT;  //                      90°
        } else if(control.getRawButton(4)) {        // High level           (Y)
            setpoint = Shoulder.HIGH_TARGET_HEIGHT; //                      100°
        } else if(control.getRawButton(3)) {        // Human player level   (X)
            setpoint = Shoulder.HUMAN_PLAYER_HEIGHT;//                      80°
        } else if(control.getRawButton(8)) {        // Inward mode button   (=)
            setpoint = 0;                           //                      0°
        }
        shoulder.setHeight(setpoint - control.getRawAxis(1) / 10);
    }

    @Override
    public void end(boolean interrupted) {

    }
}
