// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;

import java.util.Set;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive swerve;
  private final Joystick leftStick;
  private final Joystick rightStick;
  private final Joystick classic;
  private final Joystick xbox;
  private final JoystickButton button6;
  private final ADXRS450_Gyro gyro;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TeleopDrive(SwerveDrive swerve, ADXRS450_Gyro gyro) {
    this.swerve = swerve;
    this.gyro = gyro;
    leftStick = new Joystick(0);
    rightStick = new Joystick(1);
    classic = new Joystick(4);
    xbox = new Joystick(3);
    button6 = new JoystickButton(leftStick, 6);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // MotorGroup[] motorGroups = swerve.getMotorGroups();
    // for(int i = 0; i < motorGroups.length; i++) {
    //   SmartDashboard.putNumber("module " + i, motorGroups[i].getAngle());
    // }
    swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      getThrottle(-xbox.getX()), 
      getThrottle(-xbox.getY()), 
      getThrottle(xbox.getRawAxis(4)),
      Rotation2d.fromDegrees(gyro.getAngle())), 
      1.0, 
      0.00
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  private double getThrottle(double joyStickInput) {
    if (Math.abs(joyStickInput) < .05){
      return 0;
    } 
    return joyStickInput * 1;
  }
}
