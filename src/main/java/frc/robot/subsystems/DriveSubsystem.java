// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import org.photonvision.LEDMode;
import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.Ports;
import frc.robot.Constants.VisionConstants;

public class DriveSubsystem extends SubsystemBase {

  private final VictorSP m_frontLeft = new VictorSP(Ports.kFrontLeftMotorPort);
  private final VictorSP m_frontRight = new VictorSP(Ports.kFrontRightMotorPort);
  private final VictorSP m_rearLeft = new VictorSP(Ports.kRearLeftMotorPort);
  private final VictorSP m_rearRight = new VictorSP(Ports.kRearRightMotorPort);

  private final MecanumDrive m_drive = new MecanumDrive(m_frontLeft, m_frontRight, m_rearLeft, m_rearRight);
  private final Encoder m_frontLeftEncoder = new Encoder(
    Ports.kFrontLeftEncoderPort[0], 
    Ports.kFrontLeftEncoderPort[1],
    DriveConstants.kFrontLeftEncoderReversed);

  private final Encoder m_frontRightEncoder = new Encoder(
    Ports.kFrontRightEncoderPort[0],
    Ports.kFrontRightEncoderPort[1], 
    DriveConstants.kFrontRightEncoderReversed);

  private final Encoder m_rearLeftEncoder = new Encoder(
    Ports.kRearLeftEncoderPort[0], 
    Ports.kRearLeftEncoderPort[1],
    DriveConstants.kRearLeftEncoderReversed);

  private final Encoder m_rearRightEncoder = new Encoder(
    Ports.kRearRightEncoderPorts[0],
    Ports.kRearRightEncoderPorts[1], 
    DriveConstants.kRearRightEncoderReversed);

  private final MecanumDriveOdometry m_Odometry = 
    new MecanumDriveOdometry(DriveConstants.kDriveKinematics, getAngle());
  
  private final PhotonCamera 
    goalcam = new PhotonCamera("goal"), 
    ballcam = new PhotonCamera("ball");

  private final PIDController controller = 
    new PIDController(VisionConstants.kP, 0, VisionConstants.kD);

  private final static AHRS navx = new AHRS(Port.kOnboard);

  // private final AnalogInput ultrasonic = new
  // AnalogInput(Ports.kUltrasonicPort[0]);
  private final Ultrasonic ultrasonic = new Ultrasonic(Ports.kUltrasonicPort[0], Ports.kUltrasonicPort[1],
      Unit.kMillimeters);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    m_frontLeftEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulseMeters);
    m_frontRightEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulseMeters);
    m_rearLeftEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulseMeters);
    m_rearRightEncoder.setDistancePerPulse(DriveConstants.kDistancePerPulseMeters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_Odometry.update(getAngle(), getWheelSpeeds());
  }
  //Drive Shit
  public void driveLinear(double x, double y, double rot){
    m_drive.driveCartesian(x, y, rot);
  }
  public void OporatorDrive(double x, double y, double rot){
    m_drive.driveCartesian(
      OIConstants.ex(x), 
      OIConstants.ex(y), 
      OIConstants.ex(rot), 
      -navx.getAngle());
  }
  public void setMaxOutput(double maxOutput){
    m_drive.setMaxOutput(maxOutput);
  }
  public void stop(){
    m_drive.stopMotor();
  }
  //Navx Shit
  public Rotation2d getAngle(){
    return Rotation2d.fromDegrees(-navx.getAngle());
  }
  public double getAngleDegrees(){
    return -navx.getAngle();
  }
  public void zeroHeading(){
    navx.reset();
  }
  public double getHeading(){
    return Math.IEEEremainder(navx.getAngle(), 360);
  }
  public double getTurnRate(){
    return navx.getRate();
  }
  //Ultra shit
  public double getRangeMeters(){
    return ultrasonic.getRangeMM()/1000;
  }
  //Odomentry Shit
  public MecanumDriveWheelSpeeds getWheelSpeeds(){
    return new MecanumDriveWheelSpeeds(
      m_frontLeftEncoder.getRate(), 
      m_frontRightEncoder.getRate(), 
      m_rearLeftEncoder.getRate(), 
      m_rearRightEncoder.getRate());
  }
  public Pose2d getPose(){
    return m_Odometry.getPoseMeters();
  }
  public void setDriveSpeedControllerVolts(MecanumDriveMotorVoltages volts){
    m_frontLeft.setVoltage(volts.frontLeftVoltage);
    m_rearLeft.setVoltage(volts.rearLeftVoltage);
    m_frontRight.setVoltage(volts.frontRightVoltage);
    m_rearRight.setVoltage(volts.rearRightVoltage);
  }
  public Encoder getFrontLeftEncoder(){return m_frontLeftEncoder;}
  public Encoder getFrontRightEncoder(){return m_frontRightEncoder;}
  public Encoder getRearLeftEncoder(){return m_rearLeftEncoder;}
  public Encoder getRearRightEncoder(){return m_rearRightEncoder;}
  //Vision shit
  private double ballRotAjust, goalRotAjust;
  public void ballAlign(double forward, double side){
    
    var result = goalcam.getLatestResult();
    if(result.hasTargets()){
      ballRotAjust = controller.calculate(result.getBestTarget().getYaw(), 0);
      ballcam.setLED(LEDMode.kBlink);
    }else{
      ballRotAjust = 0;
      ballcam.setLED(LEDMode.kOff);
    }
    driveLinear(side, forward, ballRotAjust);
  }
  public void goalAlign(double forward, double side){
    goalcam.setLED(LEDMode.kOn);
    ballcam.setLED(LEDMode.kBlink);
    var result = goalcam.getLatestResult();
    if(result.hasTargets()){
      goalRotAjust = controller.calculate(result.getBestTarget().getYaw(), 0);
    } else {
      goalRotAjust = 0;
    }
    driveLinear(side, forward, goalRotAjust);
  }
}
