// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import frc.robot.commands.Autonomous;
import frc.robot.commands.Balance2;
import frc.robot.commands.Calibrate;
import frc.robot.commands.DumbTelescope;
import frc.robot.commands.MoveToPoint;
import frc.robot.commands.SetArm;
import frc.robot.commands.TeleopDrive;
import frc.robot.commands.UpdateArmPID;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Telescope;
import frc.robot.subsystems.UltraSensor;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Shoulder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final SwerveDrive swerve;
  private final Shoulder shoulder;
  private final Telescope telescope;
  private final LEDs leds;
  private final Gyro gyro;
  private final Claw claw;
  // private final CameraSubsystem cameraSystem;
  
  // Controllers
  private final Joystick gXbox;
  private final Joystick rXbox;

  // Joystick Buttons
  private final JoystickButton clawButton;
  private final JoystickButton coneButton;
  private final JoystickButton cubeButton;

  private final JoystickButton balanceButton;

  private final JoystickButton resetGyroButton;
  // private final JoystickButton resetPIDButton;
  private final JoystickButton moveButton;

  // Sendable Choosers for SmartDashboard / ShuffleBoard
  private final SendableChooser<Command> teleopChooser;
  private final SendableChooser<String> autoChooser;
  
  public RobotContainer() {
    // Start capture of camera and broadcast to driver station
    CameraServer.startAutomaticCapture();
    // Configure LEDs
    leds = new LEDs();
    leds.setGamerMode(true);

    // Set up controllers
    gXbox = new Joystick(0);
    rXbox = new Joystick(1);
    
    // Initialize gyro
    gyro = new ADXRS450_Gyro();
    // Primary subsystems
    swerve = new SwerveDrive(gyro);
    shoulder = new Shoulder();
    telescope = new Telescope();
    claw = new Claw();
    // Secondary subsystems
    new UltraSensor();
    
    // Initialize JoystickButton objects on Green controller (Driver 2)
    coneButton = new JoystickButton(gXbox, 5);
    cubeButton = new JoystickButton(gXbox, 6);
    balanceButton = new JoystickButton(gXbox, 9);
    
    // Initialize JoystickButton objects on Red controller (Driver 1)
    clawButton = new JoystickButton(rXbox, 6);
    resetGyroButton = new JoystickButton(rXbox, 8);
    // resetPIDButton = new JoystickButton(rXbox, 10);
    moveButton = new JoystickButton(rXbox, 2);

    // Command to update the arm height and extension
    UpdateArmPID uap = new UpdateArmPID(shoulder, telescope, gXbox);

    // Set default commands for the subsystems
    swerve.setDefaultCommand(new TeleopDrive(swerve, gyro, rXbox));
    shoulder.setDefaultCommand(uap);
    telescope.setDefaultCommand(new DumbTelescope(telescope));
    
    // Button to change the arm presets to better suit cones
    coneButton.onTrue(new InstantCommand(() -> {
      leds.setFullStripColorRGB(128, 64, 0);
      uap.setCone();
    }));
    // Button to change the arm presets to better suit cubes
    cubeButton.onTrue(new InstantCommand(() -> {
      leds.setFullStripColor(Color.kPurple);
      uap.setCube();
    }));
    // Button to open and close the claw
    clawButton.onTrue(new InstantCommand(() -> {
      claw.crab();
    }));
    // Button to reset the gyro on the robot to reset its orientation
    resetGyroButton.whileTrue(new InstantCommand(() -> {
      gyro.reset();
    }));
    // Button to reset the PID controllers responsible for controlling the swerve modules
    // resetPIDButton.whileTrue(new InstantCommand(() -> {
    //   swerve.resetPID();
    // }));

    moveButton.whileTrue(new MoveToPoint(swerve, new Pose2d(0.5, -2, Rotation2d.fromDegrees(0))));

    // Temporary button to balance the robot on the charge station
    balanceButton.whileTrue(new Balance2(swerve));

    // cameraSystem = new CameraSubsystem(swerve);
    // cameraSystem.setDefaultCommand(new PointToTarget(cameraSystem, swerve));

    // resetArmEncoderButton.onTrue(new InstantCommand(() -> shoulder.resetPosition()));


    teleopChooser = new SendableChooser<>();
    teleopChooser.setDefaultOption("Drive", new TeleopDrive(swerve, gyro, rXbox));
    teleopChooser.addOption("Calibrate", new Calibrate(swerve));
    teleopChooser.addOption("None", null);
    SmartDashboard.putData("Drive Mode", teleopChooser);

    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Test Circle", "Test Circle nospin");
    autoChooser.addOption("None", null);
    SmartDashboard.putData("Autonomous", autoChooser);
  }

  public void startTeleOp() {
    System.out.println(teleopChooser.getSelected());
    swerve.setDefaultCommand(teleopChooser.getSelected());
    leds.setGamerMode(false);
    leds.setFullStripColor(Color.kBlack);
  }

  public void endTeleop() {
    leds.setGamerMode(true);
  }

  public void periodic() {
    // SmartDashboard.putNumber("Angle", gyro.getRotation2d().getDegrees());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *$
   * @return the command to run in autonomous
   * @throws IOException
   */

  // public Command getJsonPathCommand(String path) {
  //   // return Autonomous.getCommand(swerve, path);
  // }
  
  public Command getAutonomousCommand() {
    List<Pose2d> poses1 = new ArrayList<>();
    poses1.add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    poses1.add(new Pose2d(0, 0.24, Rotation2d.fromDegrees(0)));

    List<Pose2d> poses2 = new ArrayList<>();
    poses2.add(new Pose2d(0, 0.24, Rotation2d.fromDegrees(0)));
    poses2.add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    
    List<Pose2d> poses3 = new ArrayList<>();
    poses3.add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    poses3.add(new Pose2d(0, -0.5, Rotation2d.fromDegrees(180)));

    List<Pose2d> poses4 = new ArrayList<>();
    poses4.add(new Pose2d(0, 0, Rotation2d.fromDegrees(0)));
    poses4.add(new Pose2d(0, -1, Rotation2d.fromDegrees(0)));

    SwerveModuleState[] initialStates = {
      new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
      new SwerveModuleState(0, Rotation2d.fromDegrees(0))
    };

    Command leftAuto = new SequentialCommandGroup(
      new InstantCommand(() -> {
        leds.setGamerMode(false);
        leds.setFullStripColor(Color.kRed);
      }),
      new ParallelCommandGroup(
        new SetArm(shoulder, telescope, 0.266, -1300),
        Autonomous.getCommand(swerve, poses1)
      ),
      new InstantCommand(() -> claw.crab(false)),

      new ParallelCommandGroup(
        Autonomous.getCommand(swerve, poses2),
        new SequentialCommandGroup(
          new SetArm(shoulder, telescope, 0.29, 0),
          new SetArm(shoulder, telescope, 0.06, 0)
        )
      ),
      new MoveToPoint(swerve, new Pose2d(-0.25, -4, Rotation2d.fromDegrees(0))),
      new MoveToPoint(swerve, new Pose2d(0, 0, Rotation2d.fromDegrees(180))),

      new InstantCommand(() -> {
        leds.setFullStripColor(Color.kBlack);
        leds.setGamerMode(true);
      })

    );

    Command rightAuto = new SequentialCommandGroup(
      new InstantCommand(() -> {
        leds.setGamerMode(false);
        leds.setFullStripColor(Color.kRed);
      }),
      new ParallelCommandGroup(
        new SetArm(shoulder, telescope, 0.266, -1300),
        Autonomous.getCommand(swerve, poses1)
      ),
      new InstantCommand(() -> claw.crab(false)),
      new ParallelCommandGroup(
        Autonomous.getCommand(swerve, poses2),
        new SequentialCommandGroup(
          new SetArm(shoulder, telescope, 0.29, 0),
          new SetArm(shoulder, telescope, 0.06, 0)
        )
      ),
      new MoveToPoint(swerve, new Pose2d(0.25, -4, Rotation2d.fromDegrees(0))),
      new MoveToPoint(swerve, new Pose2d(0, 0, Rotation2d.fromDegrees(180))),
      new InstantCommand(() -> {
        leds.setFullStripColor(Color.kBlack);
        leds.setGamerMode(true);
      })

    );

    Command centerAuto = new SequentialCommandGroup(
      new InstantCommand(() -> {
        leds.setGamerMode(false);
        leds.setFullStripColor(Color.kRed);
      }),
      new ParallelCommandGroup(
        new SetArm(shoulder, telescope, 0.266, -1300),
        Autonomous.getCommand(swerve, poses1)
      ),
      new InstantCommand(() -> claw.crab(false)),
      new ParallelCommandGroup(
        Autonomous.getCommand(swerve, poses2),
        new SequentialCommandGroup(
          new SetArm(shoulder, telescope, 0.29, 0),
          new SetArm(shoulder, telescope, 0.06, 0)
        )
      ),
      new InstantCommand(() -> {
        leds.setFullStripColor(Color.kBlack);
        leds.setGamerMode(true);
      })

    );

    return leftAuto;

    // return null;
    // HashMap<String, Command> eventMap = new HashMap<>();

    // eventMap.put("Raise Shoulder High Target", new InstantCommand(() -> {
    // }, shoulder));

    // return AutoRoutine.getSwerveCommand(swerve, "Test Circle", eventMap);
  //   Trajectory trajectory;

  //   Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/BasicTest.wpilib.json");

  //   try {
  //     trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  //   } catch (IOException e) {
  //     e.printStackTrace();
  //     return new InstantCommand(() -> {});
  //   }

  //   PIDController xController = new PIDController(0.4, 0, 0);
  //   PIDController yController = new PIDController(0.4, 0, 0);
  //   ProfiledPIDController thetaController = new ProfiledPIDController(
  //     1, 0, 0, new TrapezoidProfile.Constraints(1, 0.5)
  //   );

  //   thetaController.setTolerance(0.001);
  //   xController.setTolerance(0.001);
  //   yController.setTolerance(0.001);

  //   thetaController.enableContinuousInput(-Math.PI, Math.PI);

  //   Command swerveCommand = new SwerveControllerCommand(
  //     trajectory,
  //     swerve::getPose,
  //     swerve.getKinematics(),
  //     xController,
  //     yController,
  //     thetaController,
  //     this::updateTheta,
  //     swerve::setModuleStates,
  //     swerve
  //   );
    
  //   return new SequentialCommandGroup(
  //     new InstantCommand(() -> swerve.setOdometer(new Pose2d(2, 2, Rotation2d.fromDegrees(0)))),
  //     new InstantCommand(() -> SmartDashboard.putString("status", "Running")),
  //     // swerveCommand,
  //     new InstantCommand(() -> SmartDashboard.putString("status", "Done")),
  //     new InstantCommand(() -> swerve.stopMotors())
  //   );
  }

  // Rotation2d updateTheta() {
  //   //return new Rotation2d(classic.getRawAxis(2), classic.getRawAxis(3));
  //   return new Rotation2d(0);
  // }
}