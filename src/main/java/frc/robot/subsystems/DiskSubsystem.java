// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DiskSpinnerConstants;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.Constants.Ports;

public class DiskSubsystem extends SubsystemBase {
  private final VictorSPX diskspinner = new VictorSPX(Ports.kDiskSpinnerCANID);

  private final DoubleSolenoid piston = new DoubleSolenoid(
    Ports.kPCMCANID,
    Ports.kDiskPistonPorts[0],
    Ports.kDiskPistonPorts[1]
  );
  private final Encoder diskspinnerencoder = new Encoder(
    Ports.kDiskEncoderPort[0],
    Ports.kDiskEncoderPort[1],
    DiskSpinnerConstants.kWheelEncoderReversed
  );

  private final ColorSensorV3 colorSensor = new ColorSensorV3(Port.kOnboard); //TODO Check
  private final ColorMatch m_ColorMatch = new ColorMatch();
  private final Color 
    kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429),
    kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240),
    kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114),
    kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  /** Creates a new DiskSubsystem. */
  public DiskSubsystem() {
    diskspinnerencoder.setDistancePerPulse(DiskSpinnerConstants.kDistancePerPulseMeters);
    m_ColorMatch.addColorMatch(kBlueTarget);
    m_ColorMatch.addColorMatch(kGreenTarget);
    m_ColorMatch.addColorMatch(kRedTarget);
    m_ColorMatch.addColorMatch(kYellowTarget);
  }

  @Override
  public void periodic() {
    Color detectedColor = colorSensor.getColor();
    String colorString;
    ColorMatchResult match = m_ColorMatch.matchClosestColor(detectedColor);
    if (match.color == kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == kRedTarget) {
      colorString = "Red";
    } else if (match.color == kGreenTarget) {
      colorString = "Green";
    } else if (match.color == kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }
    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }
  public Color getColor(){
    return colorSensor.getColor();
  }
  public String getColorString(){
    if(colorSensor.getColor() == kBlueTarget){
      return "Blue";
    } else if (colorSensor.getColor() == kRedTarget){
      return "Red";
    } else if (colorSensor.getColor() == kYellowTarget){
      return "Yellow";
    } else if(colorSensor.getColor() == kGreenTarget){
      return "Green";
    } else {return "?";}
  }
  public char getMatchColor(){
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length()>0){
      char color;
      switch (gameData.charAt(0)){
        case 'B' :
          color = 'B';
          break;
        case 'G' :
          color = 'G';
          break;
        case 'R' :
          color = 'R';
          break;
        case 'Y' :
          color = 'Y';
          break;
        default :
          color = '!';
          break;
      }
      return color;
    }else{
      System.out.println("NO DATA FROM FMS");
      return '?';
    } 
  }
  public void resetEncoder(){
    diskspinnerencoder.reset();
  }
  public void spin(){diskspinner.set(ControlMode.PercentOutput, MotorSpeeds.kSpinnerspeed);}
  public void stop(){diskspinner.setNeutralMode(NeutralMode.Brake);}
  public void aligntocolor(){
    if(getColor() == kBlueTarget && getMatchColor() == 'B'){
      stop();
    } else if (getColor() == kGreenTarget && getMatchColor() == 'G'){
      stop();
    } else if (getColor() == kRedTarget && getMatchColor() == 'R'){
      stop();
    } else if (getColor() == kYellowTarget && getMatchColor() == 'Y'){
      stop();
    } else {
      spin();
    }
  }
  public void pistonUp(){piston.set(Value.kForward);}
  public void pistonDown(){piston.set(Value.kReverse);}
  public void pistonStop(){piston.set(Value.kOff);}
  public void pistonToggle(){piston.toggle();}
}
