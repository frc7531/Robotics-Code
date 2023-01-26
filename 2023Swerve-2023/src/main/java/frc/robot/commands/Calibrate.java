// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class Calibrate extends CommandBase {
  private final SwerveDrive swerve;

  private final Joystick classic;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The swerve subsystem
   */
  public Calibrate(SwerveDrive swerve) {
    this.swerve = swerve;
    classic = new Joystick(0);
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
    if(classic.getRawButton(1)) {
      swerve.drive(new ChassisSpeeds(0.0, 1, 0), 1, 0);
    } else {
      swerve.drive(new ChassisSpeeds(0, 0, 0), 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
