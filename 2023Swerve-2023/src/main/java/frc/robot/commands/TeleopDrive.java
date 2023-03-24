// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TeleopDrive extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final SwerveDrive swerve;
  
  private final Joystick controller;

  private final Gyro gyro;

  private final PIDController turnPID = new PIDController(0.005, 0.001, 0);

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The swerve subsystem
   * @param gyro The robot's gyro
   */
  public TeleopDrive(SwerveDrive swerve, Gyro gyro, Joystick controller) {
    this.swerve = swerve;

    this.controller = controller;

    this.gyro = gyro;

    turnPID.enableContinuousInput(0, 360);
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
    double xSpeed = -2;
    double ySpeed = -2;
    double turnSpeed = -2;
    if(controller.getRawButton(5)) {
      xSpeed = -1;
      ySpeed = -1;
      turnSpeed = -1;
    } if(controller.getRawAxis(3) >= 0.98) {
      xSpeed = -3;
      ySpeed = -3;
      turnSpeed = -3;
    }

    double turnThrottle = getThrottle(controller.getRawAxis(4));

    if(controller.getRawButton(1)) {
      turnThrottle = turnPID.calculate(Math.IEEEremainder(gyro.getAngle(), 360), 180);
    }
    else if (controller.getRawButton(4)) {
      turnThrottle = turnPID.calculate(gyro.getAngle(), 0);
    }

    swerve.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      xSpeed * getThrottle(controller.getX()), 
      ySpeed * getThrottle(controller.getY()), 
      turnSpeed * turnThrottle,
      Rotation2d.fromDegrees(gyro.getAngle())),
      // Rotation2d.fromDegrees(0)), 
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
    if (Math.abs(joystickInput) < .1){
      return 0;
    } 
    return joystickInput * 1;
  }
}
