// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.MoveClaw;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  Joystick logi;
  Joystick xbox2;
  private final DriveTrain drive;
  private final Claw claw;
  private JoystickButton rightBumper;
  private JoystickButton leftBumper;
  private JoystickButton y;
  private JoystickButton a;
  //private int pov;
  private Button povup;
  private Button povdown;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    logi = new Joystick(0);
    xbox2 = new Joystick(1);
    rightBumper = new JoystickButton(logi, 6);
    leftBumper = new JoystickButton(logi, 5);
    y = new JoystickButton(logi, 4);
    a = new JoystickButton(logi, 1);
    //pov = logi.getPOV();
    povup = new Button(() -> logi.getPOV() == 0);
    povdown = new Button(() -> logi.getPOV() == 180);
    

    
    drive = new DriveTrain();
    claw = new Claw(logi);
    // Configure the button bindings
    configureButtonBindings();
    drive.setDefaultCommand(new TeleopDrive(drive, logi, xbox2));
    //claw.setDefaultCommand(new MoveClaw(claw, logi));
    //ParallelCommandGroup clawCommand = new ParallelCommandGroup(new LeftClaw(claw, logi), new RightClaw(claw, logi));

    //claw.setDefaultCommand(clawCommand);
    rightBumper.whenPressed(new InstantCommand(() -> claw.rightClaw()));
    leftBumper.whenPressed(new InstantCommand(() -> claw.leftClaw()));
    y.whenPressed(new InstantCommand(() -> claw.wristUp()));  
    a.whenPressed(new InstantCommand(() -> claw.wristDown()));
    povup.whenPressed(new SequentialCommandGroup(
      new InstantCommand(() -> claw.armUp()), 
      new WaitCommand(.25),
      new InstantCommand(() -> claw.wristDown()))
      );
    povdown.whenPressed(new SequentialCommandGroup(
      new InstantCommand(() -> claw.wristUp()),
      new WaitCommand(.25),
      new InstantCommand(() -> claw.armDown()))
      //new InstantCommand(() -> claw.wristDown()))
      );
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
