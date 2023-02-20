package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase{
    private DoubleSolenoid crab; 
    private boolean crabTriggered;
    
    public Claw(Joystick logi){
        crab = new DoubleSolenoid(9, PneumaticsModuleType.CTREPCM, 0, 1);
        crabTriggered = true;
        
      
        
    }

    public void crab(){
        crabTriggered = !crabTriggered;
        if(crabTriggered){
            crab.set(Value.kForward);
        }else{
            crab.set(Value.kReverse);
        }
    }

    
    
}
