package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;

public final class Objects {
    //xbox controllers (primary)
    public static final Joystick gXbox = new Joystick(0);
    public static final Joystick rXbox = new Joystick(1);

    //wii controllers (secondary)
    public static final Joystick wii = new Joystick(2);
    public static final Joystick nunchuck = new Joystick(3);
    public static final Joystick classic = new Joystick(4);

    //the robot gyro
    public static final ADXRS450_Gyro gyro = new ADXRS450_Gyro();
}
