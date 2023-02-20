package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Claw;

public class CrabMove extends CommandBase{

    public final Claw clawSubsystem;
    public final Joystick xbox;
    
    public CrabMove(Claw clawSubsystem, Joystick xbox){
        this.clawSubsystem = clawSubsystem;
        this.xbox = xbox;
        addRequirements(clawSubsystem);
        
    }

    public void execute(){

    }
}