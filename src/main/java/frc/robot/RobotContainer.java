// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DiskSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterPIDSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ShooterPIDSubsystem m_Shooter = new ShooterPIDSubsystem();
  private final IntakeSubsystem m_Intake = new IntakeSubsystem();
  private final DriveSubsystem m_DriveTrain = new DriveSubsystem();
  private final DiskSubsystem m_DiskSpinner = new DiskSubsystem();

  public static final Joystick stick = new Joystick(OIConstants.kJoyPort);
  private static final Joystick ctrlBoard0 = new Joystick(OIConstants.kBoardport);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    m_DriveTrain.setDefaultCommand(
        new RunCommand(() -> m_DriveTrain.OporatorDrive(stick.getX(), stick.getY(), stick.getZ()), m_DriveTrain));
    m_Shooter.setDefaultCommand(new RunCommand(() -> m_Shooter.disable(), m_Shooter));
    m_Intake.setDefaultCommand(new RunCommand(() -> m_Intake.intakestop(), m_Intake));
    m_DiskSpinner.setDefaultCommand(new RunCommand(() -> m_DiskSpinner.stop(), m_DiskSpinner));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // flight stick
    final Button trigger = new Button(() -> stick.getRawButton(1));
    final Button thumb = new Button(() -> stick.getRawButton(2));
    final Button topLeft = new Button(() -> stick.getRawButton(4));
    final Button topRight = new Button(() -> stick.getRawButton(6));
    final Button bottomLeft = new Button(() -> stick.getRawButton(3));
    final Button bottomRight = new Button(() -> stick.getRawButton(4));
    final Button stickPovUP = new Button(() -> stick.getPOV() == 0);
    final Button stickPovDOWN = new Button(() -> stick.getPOV() == 180);
    final Button stickPovLEFT = new Button(() -> stick.getPOV() == 270);
    final Button stickPovRIGHT = new Button(() -> stick.getPOV() == 90);
    final Button button7 = new Button(() -> stick.getRawButton(7));
    final Button button8 = new Button(() -> stick.getRawButton(8));
    final Button button9 = new Button(() -> stick.getRawButton(9));
    final Button button10 = new Button(() -> stick.getRawButton(10));
    final Button button11 = new Button(() -> stick.getRawButton(11));
    final Button button12 = new Button(() -> stick.getRawButton(12));
    // Custom Controller Boards
    final Button Custom1 = new Button(() -> ctrlBoard0.getRawButton(1));
    final Button Custom2 = new Button(() -> ctrlBoard0.getRawButton(2));
    final Button Custom3 = new Button(() -> ctrlBoard0.getRawButton(3));
    final Button Custom4 = new Button(() -> ctrlBoard0.getRawButton(4));
    final Button Custom5 = new Button(() -> ctrlBoard0.getRawButton(5));
    final Button Custom6 = new Button(() -> ctrlBoard0.getRawButton(6));
    final Button Custom7 = new Button(() -> ctrlBoard0.getRawButton(7));
    final Button Custom8 = new Button(() -> ctrlBoard0.getRawButton(8));
    final Button Custom9 = new Button(() -> ctrlBoard0.getRawButton(9));
    final Button Custom10 = new Button(() -> ctrlBoard0.getRawButton(10));
    final Button Custom11 = new Button(() -> ctrlBoard0.getRawButton(11));
    final Button custom12 = new Button(() -> ctrlBoard0.getRawButton(12));

    // final Button Joy0Up = new
    // Button(()->ctrlBoard0.getRawAxis(OIConstants.kYAxisInt)>0);
    // final Button Joy0Down = new
    // Button(()->ctrlBoard0.getRawAxis(OIConstants.kYAxisInt)<0);
    // final Button Joy0Left = new
    // Button(()->ctrlBoard0.getRawAxis(OIConstants.kXAxisInt)<0);
    // final Button Joy0Right = new
    // Button(()->ctrlBoard0.getRawAxis(OIConstants.kXAxisInt)>0);
    // RIO
    final Button userButton = new Button(RobotController::getUserButton);
    // Shooter
    Custom11.whileHeld(() -> m_Shooter.enable(), m_Shooter);

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    TrajectoryConfig config = 
      new TrajectoryConfig(
        AutoConstants.maxVelocityMetersPerSecond, 
        AutoConstants.maxAccelerationMetersPerSecondSq)
      .setKinematics(DriveConstants.kDriveKinematics);
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
      //Start
      new Pose2d(0,0, m_DriveTrain.getAngle()),
      List.of(
        new Translation2d(1, 1),
        new Translation2d(2, -1)),
      //end
      new Pose2d(3, 0, new Rotation2d(0)), 
    config);
    /*
    MecanumControllerCommand mecanumControllerCommand = new MecanumControllerCommand(
      exampleTrajectory, 
      m_DriveTrain::getPose,
      DriveConstants.kFeedforward, 
      DriveConstants.kDriveKinematics, 
      new PIDController(AutoConstants.kPXController, 0, 0), 
      new PIDController(AutoConstants.kPYController, 0, 0),
      new ProfiledPIDController(
        AutoConstants.kPThetaController, 
        0, 0, 
        AutoConstants.kThetaControllerConstraints),
      
      AutoConstants.kMaxSpeedMetersPerSecond,

      new PIDController(DriveConstants.kPFrontLeftVel, 0, 0),
      new PIDController(DriveConstants.kPRearLeftVel, 0, 0),
      new PIDController(DriveConstants.kPFrontRightVel, 0, 0),
      new PIDController(DriveConstants.kPRearRightVel, 0, 0),

      m_DriveTrain::getWheelSpeeds,

      m_DriveTrain::setDriveSpeedControllerVolts,

      m_DriveTrain);
    */
    //MecanumControllerCommand m = new mecanumControllerCommand
    MecanumControllerCommand mecanumControllerCommand = new MecanumControllerCommand(
      exampleTrajectory, 
      m_DriveTrain::getPose, 
      DriveConstants.kFeedforward, 
      DriveConstants.kDriveKinematics,
      new PIDController(AutoConstants.kPXController, 0, 0), 
      new PIDController(AutoConstants.kPYController, 0, 0), 
      new ProfiledPIDController(
        AutoConstants.kPThetaController, 
        0, 0, 
        AutoConstants.kThetaControllerConstraints), 
      new Rotation2d(0), 
      AutoConstants.kMaxWheelVelocityMetersPerSecond,
      new PIDController(DriveConstants.kPFrontLeftVel, 0, 0),
      new PIDController(DriveConstants.kPRearLeftVel, 0, 0),
      new PIDController(DriveConstants.kPFrontRightVel, 0, 0),
      new PIDController(DriveConstants.kPRearRightVel, 0, 0), 
      m_DriveTrain::getWheelSpeeds, 
      m_DriveTrain::setDriveSpeedControllerVolts, 
      m_DriveTrain);
    return mecanumControllerCommand;
  }
}
