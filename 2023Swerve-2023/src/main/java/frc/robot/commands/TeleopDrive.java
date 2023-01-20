// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive swerve;
  
  private final Joystick controller;

  private final ADXRS450_Gyro gyro;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The swerve subsystem
   * @param gyro The robot's gyro
   */
  public TeleopDrive(SwerveDrive swerve, ADXRS450_Gyro gyro, Joystick controller) {
    this.swerve = swerve;

    this.controller = controller;

    this.gyro = gyro;
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
    //run the swerve drive method
    swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      getThrottle(controller.getX()), 
      getThrottle(controller.getY()), 
      getThrottle(-controller.getRawAxis(4)),
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

  private double getThrottle(double joystickInput) {
    if (Math.abs(joystickInput) < .05){
      return 0;
    } 
    return joystickInput * 1;
  }
}
