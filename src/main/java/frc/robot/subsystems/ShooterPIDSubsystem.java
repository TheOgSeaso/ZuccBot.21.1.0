// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.MotorSpeeds;
import frc.robot.Constants.Ports;
import frc.robot.Constants.ShooterConstants;

public class ShooterPIDSubsystem extends PIDSubsystem {
  private static final WPI_TalonSRX m_Shooter = new WPI_TalonSRX(Ports.kShooterCANID);
  private static final WPI_VictorSPX m_ShooterSlave = new WPI_VictorSPX(Ports.kShooterSlaveCANID);
  private static final SimpleMotorFeedforward m_ShooterFeedForward = 
    new SimpleMotorFeedforward(ShooterConstants.ks, ShooterConstants.kv);
  
  /** Creates a new ShooterPIDSubsystem. */
  public ShooterPIDSubsystem() {
    super(
        // The PIDController used by the subsystem
        new PIDController(ShooterConstants.kp, 0, ShooterConstants.kd));
    getController().setTolerance(ShooterConstants.kTolerance);
    setSetpoint(MotorSpeeds.ShooterSpeed);
    m_Shooter.enableVoltageCompensation(ShooterConstants.VoltageComp);
    m_ShooterSlave.enableVoltageCompensation(ShooterConstants.VoltageComp);
    m_Shooter.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    m_ShooterSlave.follow(m_Shooter);
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    m_Shooter.setVoltage(output + m_ShooterFeedForward.calculate(setpoint));
  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return (m_Shooter.getSelectedSensorVelocity()*600)/ShooterConstants.kEncoderCPR;
  }
  public void MAX(){
    m_Shooter.set(ControlMode.PercentOutput, 1);
  }
  public boolean atSetpoint(){
    return m_controller.atSetpoint();
  }
}
