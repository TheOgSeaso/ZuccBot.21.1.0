// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.MecanumDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class Ports{
        //ANALOG IN
        public static final int kPressureSensorPort = 0;
        //DIO
        public static final int[] kUltrasonicPort = new int[]{11,12};
        public static final int[] kDiskEncoderPort = new int[]{0,1};
        public static final int[] kFrontLeftEncoderPort = new int[]{2,3};
        public static final int[] kRearLeftEncoderPort = new int[]{4,5};
        public static final int[] kFrontRightEncoderPort = new int[]{6,7};
        public static final int[] kRearRightEncoderPort = new int[]{8,9}; 
        //PWM
        public static final int kFrontLeftMotorPort = 1;
        public static final int kRearLeftMotorPort = 2;
        public static final int kFrontRightMotorPort = 3;
        public static final int kRearRightMotorPort = 4;
        //CANID
        public static final int kDiskSpinnerCANID = 1;
        public static final int kBeltCANID = 2;
        public static final int kIntakeCANID = 3;
        public static final int kShooterSlaveCANID = 4;
        public static final int kShooterCANID = 5;

        public static final int kPCMCANID = 0;
        public static final int kPDPCANID = 9;
        public static final int[] kIntakePistonPorts = {1,2};
        public static final int[] kDiskPistonPorts = {3,4};
		public static final DigitalSource[] kRearRightEncoderPorts = null;
        
    }
    public static final class OIConstants{
        public static final int kJoyPort = 0;
        public static final int kBoardport = 1;
        private static final double deadzone = .05;
        private static final double pow = 2.6;
        public static double ex(double x){
            if(x > deadzone || x < -deadzone){
                return Math.pow(x, pow);
            }else{
                return 0;
            }
        }
    }
    public static final class MotorSpeeds{
        public static final double kSpinnerspeed = 1;
		public static final double ShooterSpeed = 5000;
		public static final double intakeSpeed = .5;
		public static final double beltSpeed = .5;
    }
    public static final class DriveConstants{
        private static final double kWheelBase = 19/39.37;
        private static final double kTrackWidth = 23/39.37;

        private static final double kEncoderCPR = 8192;
        private static final double kWheelDiameterMeters = 8/39.37;
        private static final double kWheelCircumfranceMeters = 
            kWheelDiameterMeters*Math.PI;
        public static final double kDistancePerPulseMeters = //77.93 μm //0.0031 in
            kWheelCircumfranceMeters / kEncoderCPR;

        public static final boolean kFrontLeftEncoderReversed = false;
        public static final boolean kRearLeftEncoderReversed = true;
        public static final boolean kFrontRightEncoderReversed = false;
        public static final boolean kRearRightEncoderReversed = true;

        public static final MecanumDriveKinematics kDriveKinematics =
            new MecanumDriveKinematics(
                new Translation2d(kWheelBase/2, kTrackWidth/2), 
                new Translation2d(kWheelBase/2, -kTrackWidth/2), 
                new Translation2d(-kWheelBase/2, kTrackWidth/2), 
                new Translation2d(-kWheelBase/2, -kTrackWidth/2));
        public static final SimpleMotorFeedforward kFeedforward =
            new SimpleMotorFeedforward(1, 0.8, 0.15); //TODO CHANGE
		public static final double kPFrontLeftVel = 0;
		public static final double kPRearLeftVel = 0;
		public static final double kPFrontRightVel = 0;
		public static final double kPRearRightVel = 0;
    }
    public static final class AutoConstants{

		public static final double maxVelocityMetersPerSecond = 0;
		public static final double maxAccelerationMetersPerSecondSq = 0;
		public static final double kMaxSpeedMetersPerSecond = 1;
		public static final double kPXController = 0;
		public static final double kPYController = 0;
		public static final double kPThetaController = 0;
        public static final Constraints kThetaControllerConstraints = 
            new TrapezoidProfile.Constraints(kMaxSpeedMetersPerSecond, maxAccelerationMetersPerSecondSq); 
		public static final double kMaxWheelVelocityMetersPerSecond = 1;
		//public static final Subsystem kMaxWheelVelocityMetersPerSecond = null;

    }
    public static final class VisionConstants{
		public static final double kD = 1;
		public static final double kP = .1;
        
    }
    public static final class DiskSpinnerConstants{
        public static final boolean kWheelEncoderReversed = false;
        
        private static final double kEncoderCPR = 8192;
        private static final double kWheelDiameterMeters = .1;
        private static final double kWheelCircumfranceMeters  = 
            Math.PI*kWheelDiameterMeters;
        public static final double kDistancePerPulseMeters =
            kWheelCircumfranceMeters/kEncoderCPR;
        public static final double kDistanceToSpin3Times = 
            kWheelCircumfranceMeters*28;
    }
    public static final class ShooterConstants{
        public static final double ks = 1, kv = 1;

        public static final double kp = 1, kd = 1;

        public static final double kTolerance = 50;

		public static final boolean VoltageComp = false;

		public static final double kEncoderCPR = 4096;
    }
}
