// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {


  public static class OperatorConstants {
    //public static final int kDriverControllerPort = 0;
    public static final int kDriverLeftControllerPort = 0;
    public static final int kDriverRightControllerPort = 1;
    public static final int kOpPort = 2;
    /* 
    public static final int kDriverButtonA = 1;
    public static final int kDriverButtonB = 2;
    public static final int kDriverButtonX = 3;
    public static final int kDriverButtonY = 4;
    public static final int kDriverButtonLB = 5;
    public static final int kDriverButtonRB = 6;
    public static final int kDriverButtonBack = 7;
    public static final int kDriverButtonStart = 8;
    public static final int kDriverButtonLStick = 9;
    public static final int kDriverButtonRStick = 10;

    public static final int kDriverLXAxis = 0;
    public static final int kDriverLYAxis = 1;
    public static final int kDriverLTAxis = 2;
    public static final int kDriverRXAxis = 4;
    public static final int kDriverRYAxis = 5;
    public static final int kDriverRTAxis = 3;
     */
  }
  


  public static final class DriveConstants {
    //Drive motor CAN IDs
    public static final int kFrontLeftMotorPort = 20;
    public static final int kRearLeftMotorPort = 21;
    public static final int kFrontRightMotorPort = 22;
    public static final int kRearRightMotorPort = 23;

    //Drive motor inverse
    public static final boolean kFrontLeftMotorReversed = true; //TODO:  does reversing these help?
    public static final boolean kRearLeftMotorReversed = true;
    public static final boolean kFrontRightMotorReversed = false;
    public static final boolean kRearRightMotorReversed = false;

    //Testbed encoder ports (relative encoders on comp bot)
    public static final int[] kFrontLeftEncoderPorts = new int[] {0, 1};
    public static final int[] kRearLeftEncoderPorts = new int[] {2, 3};
    public static final int[] kFrontRightEncoderPorts = new int[] {4, 5};
    public static final int[] kRearRightEncoderPorts = new int[] {6, 7};

    //Encoder inverse
    public static final boolean kFrontLeftEncoderReversed = true;
    public static final boolean kRearLeftEncoderReversed = true;
    public static final boolean kFrontRightEncoderReversed = false;
    public static final boolean kRearRightEncoderReversed = false;

    //Gyro inverse
    public static final boolean kGyroReversed = false;

  }



  public static final class PhysicalConstants {
    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidthIn = 23; //inches
    public static final double kTrackWidth = kTrackWidthIn * 0.0254; //meters

  // Distance between centers of front and back wheels on robot
    public static final double kWheelBaseIn = 20; //inches
    public static final double kWheelBase = kWheelBaseIn * 0.0254; //meters

  // Drive base radius
     public static final double kDriveBaseRadius = Math.sqrt(Math.pow(kWheelBase / 2, 2) + Math.pow(kTrackWidth / 2, 2));
  

    public static final MecanumDriveKinematics kDriveKinematics =
        new MecanumDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Encoder CPR.
    public static final int kEncoderCPR = 4096; //Testbed CIMCoder quadrature 2 channel 20 pulses per channel 
    //public static final int kEncoderCPR = 42; //Neo integrated encoder

    // Gear ratio
    public static final double kGearRatio = 8.45; //Testbed 12.75:1
    //public static final double kGearRatio = 8.45; //Compbot 8.45:1

    // Encoder final CPR
    public static final double kEncoderFinalCPR = kEncoderCPR * kGearRatio;

    // Wheel diameter
    public static final double kWheelDiameterIn = 6; //inches
    public static final double kWheelDiameterMeters = kWheelDiameterIn * 0.0254; //meters
    public static final double kEncoderDistancePerPulse =
        (kWheelDiameterMeters * Math.PI) / (double) kEncoderFinalCPR;

    // Max RPM
    public static final double kMaxRPM = 5676 / kGearRatio; //Testbed CIM
    //public static final double kMaxRPM = 5676 / kGearRatio; //Neo 550

    // Max velocity
    //public static final double kMaxVelocity = kMaxRPM * (kWheelDiameterMeters * Math.PI) / 60; //meters per second
    public static final double kMaxVelocity = kMaxRPM * (kWheelDiameterMeters * Math.PI) / 60; //TODO: test value - adjust

    // Max acceleration
    public static final double kMaxAcceleration = 1.5; //meters per second squared TODO: tune this

    // Max angular velocity
    public static final double kMaxAngularVelocity = Math.PI; //radians per second --- 1/2 rotation per second TODO: tune this
    
    // Max angular acceleration
    public static final double kMaxAngularAcceleration = Math.PI; //radians per second squared

    // Wheel PID Constants
    public static final double kPFrontLeft = 25.77;
    public static final double kIFrontLeft = 0;
    public static final double kDFrontLeft = 0;

    public static final double kPRearLeft = 4.3996;
    public static final double kIRearLeft = 0;
    public static final double kDRearLeft = 0;

    public static final double kPFrontRight = 20.656;
    public static final double kIFrontRight = 0;
    public static final double kDFrontRight = 0;

    public static final double kPRearRight = 15.389;
    public static final double kIRearRight = 0;
    public static final double kDRearRight = 0;

    public static final String kMaxSpeedMetersPerSecond = null;

    public static double kPTranslation = 65.0;
    public static double kITranslation = 0.0;
    public static double kDTranslation = 35.0;
    
    public static double kPRotation = 0.5;
    public static double kIRotation = 0.0;
    public static double kDRotation = 0.0;



  }




  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 5.36; //TODO: adjust
    public static final double kMaxAccelerationMetersPerSecondSquared = 1.5; //TODO: adjust
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 0.5;
    public static final double kPYController = 0.5;
    public static final double kPThetaController = 0.5;

    // TODO: tune these values
    // These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
    // These characterization values MUST be determined either experimentally or theoretically
    // for *your* robot's drive.
    // The SysId tool provides a convenient method for obtaining these values for your robot.
    public static final SimpleMotorFeedforward kFeedforward =
        new SimpleMotorFeedforward(1.5893, 2.16, 0.44);

        /* 
    // Example value only - as above, this must be tuned for your drive!
    public static final double kPTranslation = 64.456; 
    //public static final double kPTranslation = 0.255;
    public static final double kITranslation = 0;
    public static final double kDTranslation = 31.613;
    //public static final double kDTranslation = 0;


    public static final double kPRotation =64.456;
    //public static final double kPRotation = 0.5;
    public static final double kIRotation = 0;
    public static final double kDRotation = 31.613;
    //public static final double kDRotation = 0;
*/


    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);


    // Path follower config        
    public static final HolonomicPathFollowerConfig kPathFollowerConfig = new HolonomicPathFollowerConfig( 
      new PIDConstants(PhysicalConstants.kPTranslation, PhysicalConstants.kITranslation, PhysicalConstants.kDTranslation), // Translation PID constants
      new PIDConstants(PhysicalConstants.kPRotation, PhysicalConstants.kIRotation, PhysicalConstants.kDRotation), // Rotation PID constants
      PhysicalConstants.kMaxVelocity, // Max module speed, in m/s
      PhysicalConstants.kDriveBaseRadius, // Drive base radius in meters. Distance from robot center to furthest module.
      new ReplanningConfig()
    );





  }

  public static final class IntakeConstants {
    //Intakemotor CAN Id
    public static final int kIntakeMotor = 30;

    //IntakeSensor 


    //deploy motor CAN Id
    public static final int kDeployMotor = 31;

    //Intake limitSwitches
    public static final int kIntakeLimitUp = 0;
    public static final int kIntakeLimitDown = 1;
    public static final int kIntakeStop = 2;
    //reversed
    public static final boolean kIntakeMotorReversed = false;
    public static final boolean kDeployMotorReversed = true;

    public static final int kDeployCPR = 4096;
    public static final double kDeployGearRatio = 125.0;
    public static final double kDeployFinalCPR = kDeployCPR*kDeployGearRatio;
    public static final double kDeployDistancePerPulse = 360 / (double) kDeployFinalCPR;

    public static final double kDeploySVolts = 1;
    public static final double kDeployGVolts = 0.24;
    public static final double kDeployVVoltsSecondPerDeg = 0.04;
    public static final double kDeployAVoltSecondSquaredPerDeg = 0.00;
    
    public static final double kDeployP = 1.0;
    public static final double kDeployI = 0.0;
    public static final double kDeployD = 0.0;

    public static final double kDeployMaxVelocityDegPerSecond = 5.64; //180
    public static final double kDeployMaxAccelerationDegPerSecondSquared = 5.57; //360

    public static final double kIntakeSpeed = 0.8;
    public static final double kStowedPosition = 0.0;
    public static final double kAmpPosition = 80.0;
    public static final double kDeployedPosition = 135.0;
    public static final double kArmTolerance = 2.0;
    

  }

  public static final class ShooterConstants {

    //Left & right feed motors
    public static final int kLeftFeedMotor = 40;
    public static final int kRightFeedMotor = 41;

    //Left & right shooting motors
    public static final int kLeftShootingMotor = 42;
    public static final int kRightShootingMotor = 43;

    //Sensor
    public static final int kLoadedSensor = 2;

    //Angle

        //Motor

        //Sensor

    //reversed
    public static final boolean kLeftFeedMotorReversed = false;
    public static final boolean kRightFeedMotorReversed = false;
    public static final boolean kLeftShootingMotorReversed = false;
    public static final boolean kRightShootingMotorReversed = false;

    public static final double kShooterSpeed = 1.0;
    
  }

  public static final class ClimberConstants {

    //Climber motor
    public static final int kClimbingMotor = 50;

    //Climber limit switches
    public static final int kClimberLimitUp = 2;
    public static final int kClimberLimitDown = 3;

    //reversed
    public static final boolean kClimbingMotorReversed = false;
  }

  public static final class VisionConstants {
    public static final String kCameraName = "PhotonVision"; //TODO: update this
    public static final double kCameraHeight = 0.5; //meters TODO: update this

  }

  public static final class FieldConstants {

  }
  
}
