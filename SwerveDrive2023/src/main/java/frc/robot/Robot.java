// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends TimedRobot {
  CANSparkMax[] motors;

  Translation2d frontLeftModule = new Translation2d(-5, 5);
  Translation2d frontRightModule = new Translation2d(5, 5);
  Translation2d rearRightModule = new Translation2d(5, -5);
  Translation2d rearLeftModule = new Translation2d(-5, -5);
  DutyCycleEncoder frontLeftEncoder = new DutyCycleEncoder(0);
  SwerveDriveKinematics sdk = new SwerveDriveKinematics(frontLeftModule, frontRightModule, rearRightModule, rearLeftModule);

  Joystick controller = new Joystick(0);

  @Override
  public void robotInit() {
    frontLeftEncoder.setConnectedFrequencyThreshold(900);
    for(int i = 0; i < 8; i++) {
      motors[i] = new CANSparkMax(i + 1, MotorType.kBrushless);
    }
  }

  @Override
  public void robotPeriodic() {

  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {
    swerveDrive(new ChassisSpeeds(controller.getRawAxis(0), controller.getRawAxis(1), controller.getRawAxis(2)));
  }

  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {

  }

  public void swerveDrive(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = sdk.toSwerveModuleStates(speeds);
    for(int i = 0; i < 4; i++) {
      swerveModuleControl(moduleStates[0], speeds.omegaRadiansPerSecond, i);
    }
  }

  public void swerveModuleControl(SwerveModuleState moduleState, double rotationSpeed, int motorNumber) {
    double speedMotor;
    double rotationMotor;
    speedMotor = moduleState.speedMetersPerSecond;
    if(frontLeftEncoder.get() % 1.0 < (moduleState.angle.getDegrees() - 5) / 360) {
      rotationMotor = rotationSpeed;
    } else if(frontLeftEncoder.get() % 1.0 > (moduleState.angle.getDegrees() - 5) / 360) {
      rotationMotor = -rotationSpeed;
    } else {
      rotationMotor = 0;
    }
    motors[motorNumber * 2].set(speedMotor + rotationMotor);
    motors[motorNumber * 2 + 1].set(speedMotor - rotationMotor);
  }
}