// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Shoulder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class DumbShoulder extends CommandBase {
  private final Shoulder shoulder;

  private final Joystick xbox;

  /**
   * Creates a new ExampleCommand.
   *
   * @param swerve The swerve subsystem
   */
  public DumbShoulder(Shoulder shoulder) {
    this.shoulder = shoulder;
    xbox = new Joystick(1);
    
    addRequirements(shoulder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoulder.setSpeed(xbox.getRawAxis(1));
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
