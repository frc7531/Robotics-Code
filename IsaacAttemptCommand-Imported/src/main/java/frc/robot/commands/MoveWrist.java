package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Claw;



public class MoveWrist extends CommandBase{
    
    private final Claw clawSubsystem;
    private final Joystick logi;

        

    public MoveWrist(Claw clawSubsystem, Joystick logi){
        this.logi = logi;
        this.clawSubsystem = clawSubsystem;
        addRequirements(clawSubsystem);
    }
    @Override
    public void execute() {
     
    }
}