package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;


public class Claw extends SubsystemBase{
    private DoubleSolenoid leftClaw;
    private DoubleSolenoid rightClaw;
    private DoubleSolenoid wrist;
    private DoubleSolenoid arm;
    private Joystick logi;
    private boolean leftToggle;
    private boolean rightTriggered;
    private boolean leftTriggered;
    
    public Claw(Joystick logi){
        rightClaw = new DoubleSolenoid(10, PneumaticsModuleType.CTREPCM, 0, 1);
        leftClaw = new DoubleSolenoid(10, PneumaticsModuleType.CTREPCM, 2, 3);
        arm = new DoubleSolenoid(10, PneumaticsModuleType.CTREPCM, 4, 5);
        wrist = new DoubleSolenoid(10, PneumaticsModuleType.CTREPCM, 6, 7);
        this.logi = logi;
        rightTriggered = true;
        leftTriggered = true;
        
      
        
    }

    public void leftClaw(){
        leftTriggered = !leftTriggered;
        if(leftTriggered){
            leftClaw.set(Value.kForward);
        }else{
            leftClaw.set(Value.kReverse);
        }
    }

    public void rightClaw(){
        rightTriggered = !rightTriggered;
        if(rightTriggered){
            rightClaw.set(Value.kReverse);
        }else{
            rightClaw.set(Value.kForward);
        }
    }
    
    public void wristUp(){
            wrist.set(Value.kForward);
    }
    public void wristDown(){
        wrist.set(Value.kReverse);
    }

    public void armUp(){
        arm.set(Value.kReverse);
    }
    public void armDown(){
        arm.set(Value.kForward);
    }

    
    
}
