package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UltraSensor extends SubsystemBase {
    private Ultrasonic sensor1;

    public UltraSensor() {
        sensor1 = new Ultrasonic(6, 7);
        sensor1.setEnabled(true);
        Ultrasonic.setAutomaticMode(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Ultra Sensor!!!", sensor1.getRangeMM());
    }
}
