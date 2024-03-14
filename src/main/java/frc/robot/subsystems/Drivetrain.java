// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveMotorVoltages;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.AutoConstants;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;




/** The drive subsystem for the robot. */
public class Drivetrain extends SubsystemBase {


    // Shuffleboard
    private final ShuffleboardTab driveTab = Shuffleboard.getTab("Drivetrain Data");
    private GenericEntry headingEntry;
    private GenericEntry turnRateEntry;
    private GenericEntry fieldRelativeEntry;

    private GenericEntry FLRateEntry;
    private GenericEntry RLRateEntry;
    private GenericEntry FRRateEntry;
    private GenericEntry RRRateEntry;

    private GenericEntry FLDistanceEntry;
    private GenericEntry RLDistanceEntry;
    private GenericEntry FRDistanceEntry;
    private GenericEntry RRDistanceEntry;

    private GenericEntry FLVoltageEntry;
    private GenericEntry RLVoltageEntry;
    private GenericEntry FRVoltageEntry;
    private GenericEntry RRVoltageEntry;

    private GenericEntry Joy1XEntry;
    private GenericEntry Joy1YEntry;
    private GenericEntry Joy2XEntry;


  private final CANSparkMax m_frontLeft = new CANSparkMax(DriveConstants.kFrontLeftMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_rearLeft = new CANSparkMax(DriveConstants.kRearLeftMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_frontRight = new CANSparkMax(DriveConstants.kFrontRightMotorPort, MotorType.kBrushless);
  private final CANSparkMax m_rearRight = new CANSparkMax(DriveConstants.kRearRightMotorPort, MotorType.kBrushless);
  
  private final MecanumDrive m_drive =
    new MecanumDrive(m_frontLeft::set, m_rearLeft::set, m_frontRight::set, m_rearRight::set);

  private Field2d field = new Field2d();


  //Encoders
  // The front-left-side drive encoder
  private final RelativeEncoder m_frontLeftEncoder;

  // The rear-left-side drive encoder
  private final RelativeEncoder m_rearLeftEncoder;

  // The front-right-side drive encoder
  private final RelativeEncoder m_frontRightEncoder;

  // The rear-right-side drive encoder
  private final RelativeEncoder m_rearRightEncoder;


  
  private final PIDController m_frontLeftPIDController =
      new PIDController(PhysicalConstants.kPFrontLeft, 
                        PhysicalConstants.kIFrontLeft,
                        PhysicalConstants.kDFrontLeft);

  private final PIDController m_rearLeftPIDController = 
      new PIDController(PhysicalConstants.kPRearLeft, 
                        PhysicalConstants.kIRearLeft,
                        PhysicalConstants.kDRearLeft);

  private final PIDController m_frontRightPIDController =
      new PIDController(PhysicalConstants.kPFrontRight, 
                        PhysicalConstants.kIFrontRight,
                        PhysicalConstants.kDFrontRight);

  private final PIDController m_rearRightPIDController =
      new PIDController(PhysicalConstants.kPRearRight, 
                        PhysicalConstants.kIRearRight,
                        PhysicalConstants.kDRearRight);


  // The gyro sensor
  private final AHRS m_gyro = new AHRS();



  // Odometry class for tracking robot pose
  MecanumDriveOdometry m_odometry =
      new MecanumDriveOdometry(
          PhysicalConstants.kDriveKinematics,
          m_gyro.getRotation2d(),
          new MecanumDriveWheelPositions());

  // The feedforward for the drive TODO: how do you tune this?
  private final SimpleMotorFeedforward m_FLfeedforward = new SimpleMotorFeedforward(1.5893, 15.323, 4.224);
  private final SimpleMotorFeedforward m_RLfeedforward = new SimpleMotorFeedforward(1.349, 15.451, 6.3319);
  private final SimpleMotorFeedforward m_FRfeedforward = new SimpleMotorFeedforward(0.20682, 27.605, 28.609);
  private final SimpleMotorFeedforward m_RRfeedforward = new SimpleMotorFeedforward(1.0352, 18.084, 2.7686);


  /** Constructor -- initialize values here */


  public Drivetrain() {
    // Calibrate gyro
      new Thread(() -> {
        try {
          Thread.sleep(1000);
          zeroHeading();
          } catch (Exception e){
        }
      }).start();



    // Initialize Shuffleboard widgets
    headingEntry = driveTab.add("Heading", 0).withWidget("Gyro").withPosition(0, 0).withSize(2, 2).getEntry();
    turnRateEntry = driveTab.add("Turn Rate", 0).withWidget("Dial").withPosition(2, 0).withSize(2, 2).getEntry();
    fieldRelativeEntry = driveTab.add("Field Relative", false).withWidget("Boolean Box").withPosition(4, 0).withSize(1, 1).getEntry();

    // Initialize Shuffleboard widgets for encoder distances
    FLDistanceEntry = driveTab.add("FL Encoder Distance", 0.0).getEntry();
    RLDistanceEntry = driveTab.add("RL Encoder Distance", 0.0).getEntry();
    FRDistanceEntry = driveTab.add("FR Encoder Distance", 0.0).getEntry();
    RRDistanceEntry = driveTab.add("RR Encoder Distance", 0.0).getEntry();

    // Initialize Shuffleboard widgets for voltages
    FLVoltageEntry = driveTab.add("FL Voltage", 0.0).getEntry();
    RLVoltageEntry = driveTab.add("RL Voltage", 0.0).getEntry();
    FRVoltageEntry = driveTab.add("FR Voltage", 0.0).getEntry();
    RRVoltageEntry = driveTab.add("RR Voltage", 0.0).getEntry();

    // Initialize Shuffleboard widgets for encoder rates
    FLRateEntry = driveTab.add("FL Encoder Rate", 0.0).getEntry();
    RLRateEntry = driveTab.add("RL Encoder Rate", 0.0).getEntry();
    FRRateEntry = driveTab.add("FR Encoder Rate", 0.0).getEntry();
    RRRateEntry = driveTab.add("RR Encoder Rate", 0.0).getEntry();

    // Initialize Shuffleboard widgets for joystick values
    Joy1XEntry = driveTab.add("Joy1 X", 0.0).getEntry();
    Joy1YEntry = driveTab.add("Joy1 Y", 0.0).getEntry();
    Joy2XEntry = driveTab.add("Joy2 X", 0.0).getEntry();



    //Testbed Factory reset motor controllers
    m_frontLeft.restoreFactoryDefaults();
    m_rearLeft.restoreFactoryDefaults();
    m_frontRight.restoreFactoryDefaults();
    m_rearRight.restoreFactoryDefaults();


    //break/coast mode TODO: should these be coast?
    m_frontLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rearLeft.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_frontRight.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_rearRight.setIdleMode(CANSparkMax.IdleMode.kBrake);

    // Set up encoders
    m_frontLeftEncoder = m_frontLeft.getEncoder();
    m_rearLeftEncoder = m_rearLeft.getEncoder();
    m_frontRightEncoder = m_frontRight.getEncoder();
    m_rearRightEncoder = m_rearRight.getEncoder();

    m_frontLeftEncoder.setPositionConversionFactor(PhysicalConstants.kEncoderDistancePerPulse);
    m_rearLeftEncoder.setPositionConversionFactor(PhysicalConstants.kEncoderDistancePerPulse);
    m_frontRightEncoder.setPositionConversionFactor(PhysicalConstants.kEncoderDistancePerPulse);
    m_rearRightEncoder.setPositionConversionFactor(PhysicalConstants.kEncoderDistancePerPulse);

    //m_frontLeftEncoder.setInverted(DriveConstants.kFrontLeftEncoderReversed);
    //m_rearLeftEncoder.setInverted(DriveConstants.kRearLeftEncoderReversed);
    //m_frontRightEncoder.setInverted(DriveConstants.kFrontLeftEncoderReversed);
    //m_rearRightEncoder.setInverted(DriveConstants.kRearLeftEncoderReversed);

    //invert motors on one side
  m_frontLeft.setInverted(DriveConstants.kFrontLeftMotorReversed);
  m_rearLeft.setInverted(DriveConstants.kRearLeftMotorReversed);
  m_frontRight.setInverted(DriveConstants.kFrontRightMotorReversed);
  m_rearRight.setInverted(DriveConstants.kRearRightMotorReversed);

    // Configure AutoBuilder

    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetPose, 
      this::getChassisSpeeds, 
      //this::driveFieldRelative,  
      this::driveRobotRelative,

      AutoConstants.kPathFollowerConfig,
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }

        return false;
      },
      this
    );
    
      
    
  }

  /**Sets the drive MotorController to a voltage. */
  public void setDriveMotorControllersVolts(MecanumDriveMotorVoltages volts) {
    m_frontLeft.setVoltage(volts.frontLeftVoltage);
    m_rearLeft.setVoltage(volts.rearLeftVoltage);
    m_frontRight.setVoltage(volts.frontRightVoltage);
    m_rearRight.setVoltage(volts.rearRightVoltage);
  }

  public void setWheelSpeeds(double forwardSpeedMetersPerSecond, double strafeSpeedMetersPerSecond, double angularSpeedRadiansPerSecond) {
    var chassisSpeeds = new ChassisSpeeds (forwardSpeedMetersPerSecond, strafeSpeedMetersPerSecond, angularSpeedRadiansPerSecond);
    var wheelSpeeds = PhysicalConstants.kDriveKinematics.toWheelSpeeds(chassisSpeeds);

    wheelSpeeds.desaturate(PhysicalConstants.kMaxVelocity);
    applyPidAndFeedforward(wheelSpeeds);
  }



  private void applyPidAndFeedforward(MecanumDriveWheelSpeeds targetWheelSpeeds) {
    // Calculate PID outputs and apply feedforward
    double flOutput = calculateOutput(m_frontLeftEncoder, targetWheelSpeeds.frontLeftMetersPerSecond, m_frontLeftPIDController, m_FLfeedforward); 
    double rlOutput = calculateOutput(m_rearLeftEncoder, targetWheelSpeeds.rearLeftMetersPerSecond, m_rearLeftPIDController, m_RLfeedforward); 
    double frOutput = calculateOutput(m_frontRightEncoder, targetWheelSpeeds.frontRightMetersPerSecond, m_frontRightPIDController, m_FRfeedforward); 
    double rrOutput = calculateOutput(m_rearRightEncoder, targetWheelSpeeds.rearRightMetersPerSecond, m_rearRightPIDController, m_RRfeedforward); 

    // Set motor voltages
    m_frontLeft.setVoltage(flOutput);
    m_rearLeft.setVoltage(rlOutput);
    m_frontRight.setVoltage(frOutput);
    m_rearRight.setVoltage(rrOutput);
  } 

  private double calculateOutput(RelativeEncoder m_frontLefEncoder, double targetSpeed, PIDController pidController, SimpleMotorFeedforward feedforward) {
    double currentSpeed = m_frontLefEncoder.getVelocity();
    //double currentSpeed = m_frontLeftEncoder2.getRate();
    double pidOutput = pidController.calculate(currentSpeed, targetSpeed);
    double feedforwardOutput = feedforward.calculate(targetSpeed);
    return pidOutput + feedforwardOutput;
  }
  /* 
  public void setTranslationSetpoint(double translationSpeed) {
    m_translationPID.setSetpoint(translationSpeed);
    // Logic to convert translation speed to individual wheel speeds
  }

public void setRotationSetpoint(double rotationSpeed) {
    m_rotationPID.setSetpoint(rotationSpeed);
    // Logic to convert rotation speed to individual wheel speeds
  }
  */

/* Periodic
 * 
 * 
 * 
 * 
 */

  @Override
  public void periodic() {

    //TODO update this
    // Update the odometry in the periodic block
    /* 
        var wheelSpeeds = new MecanumDriveWheelSpeeds(
          m_frontLeftEncoder.getVelocity(), m_rearLeftEncoder.getVelocity(),
          m_frontRightEncoder.getVelocity(), m_rearRightEncoder.getVelocity()
        );
    */
        
        var wheelPositions = new MecanumDriveWheelPositions(
          m_frontLeftEncoder.getPosition(), m_rearLeftEncoder.getPosition(),
          m_frontRightEncoder.getPosition(), m_rearRightEncoder.getPosition()
        );

    m_odometry.update(m_gyro.getRotation2d().times(-1), wheelPositions);

    // Update the field
    field.setRobotPose(getPose());

    showTelemetry();
  }

/*
 * 
 * 
 * 
 */

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  //TODO: update for aprilTag detection of current pose

  public void resetPose(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d().times(-1), getCurrentWheelDistances(), pose);
  }

 /*  Drive methods
  * 
  *
  *
  *
  */

  public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
/*
    // Adjusting the translation speed based on PID output
    double currentTranslation = // get current forward speed;
    double pidOutput = m_translationPID.calculate(currentTranslation);
        
    // Adjust the fieldRelativeSpeeds with the PID output
    fieldRelativeSpeeds.vxMetersPerSecond += pidOutput;
    
    // Convert to robot relative speeds and drive
    driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    */

  }

  
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

    //MecanumDriveWheelSpeeds targetStates = PhysicalConstants.kDriveKinematics.toWheelSpeeds(targetSpeeds);
    setWheelSpeeds(targetSpeeds.vxMetersPerSecond, targetSpeeds.vyMetersPerSecond, targetSpeeds.omegaRadiansPerSecond);
    //m_frontLeft.set(targetStates.frontLeftMetersPerSecond);
    //m_rearLeft.set(targetStates.rearLeftMetersPerSecond);
    //m_frontRight.set(targetStates.frontRightMetersPerSecond);
    //m_rearRight.set(targetStates.rearRightMetersPerSecond);
    //setWheelSpeeds(targetStates);
    //setWheelSpeeds(targetStates);

  }


/**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
    double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
      /*var mecanumDriveWheelSpeeds =
          PhysicalConstants.kDriveKinematics.toWheelSpeeds(
             ChassisSpeeds.discretize(
                  fieldRelative
                      ? ChassisSpeeds.fromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, m_gyro.getRotation2d())
                      : new ChassisSpeeds(xSpeed, ySpeed, rot),
                  periodSeconds));
      mecanumDriveWheelSpeeds.desaturate(PhysicalConstants.kMaxVelocity);
      setSpeeds(mecanumDriveWheelSpeeds);
      */
      if(fieldRelative){
        m_drive.driveCartesian(xSpeed, ySpeed, rot, m_gyro.getRotation2d().times(-1));
      }
      else{
        m_drive.driveCartesian(xSpeed, ySpeed, rot);
      }

      Joy1XEntry.setDouble(xSpeed);
      Joy1YEntry.setDouble(ySpeed);
      Joy2XEntry.setDouble(rot);


  }





  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(m_gyro.getRotation2d().times(-1), getCurrentWheelDistances(), pose);
  }




  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {

    //Testbed
    /* 
    m_frontLeftEncoder.reset();
    m_rearLeftEncoder.reset();
    m_frontRightEncoder.reset();
    m_rearRightEncoder.reset();
   */
      m_frontLeftEncoder.setPosition(0);
      m_rearLeftEncoder.setPosition(0);
      m_frontRightEncoder.setPosition(0);
      m_rearRightEncoder.setPosition(0);
  }


  public RelativeEncoder getFrontLeftEncoder() {
    //Testbed
    return m_frontLeftEncoder;
  }

  public RelativeEncoder getRearLeftEncoder() {
    //Testbed
    return m_rearLeftEncoder;
  }

  public RelativeEncoder getFrontRightEncoder() {
    //Testbed
    return m_frontRightEncoder;
  }


  public RelativeEncoder getRearRightEncoder() {

    //Testbed
    return m_rearRightEncoder;
  }

  /**
   * Gets the current wheel speeds.
   *
   * @return the current wheel speeds in a MecanumDriveWheelSpeeds object.
   */
  public MecanumDriveWheelSpeeds getCurrentWheelSpeeds() {
    
  return new MecanumDriveWheelSpeeds(
      m_frontLeftEncoder.getVelocity(),
      m_rearLeftEncoder.getVelocity(),
      m_frontRightEncoder.getVelocity(),
      m_rearRightEncoder.getVelocity());

        //Testbed
        /* 
        m_frontLeftEncoder.getRate(),
        m_rearLeftEncoder.getRate(),
        m_frontRightEncoder.getRate(),
        m_rearRightEncoder.getRate());
*/

  }

  /**
   * Gets the current wheel distance measurements.
   *
   * @return the current wheel distance measurements in a MecanumDriveWheelPositions object.
   */
  public MecanumDriveWheelPositions getCurrentWheelDistances() {


    //Testbed
    return new MecanumDriveWheelPositions(
      m_frontLeftEncoder.getPosition(),
        m_rearLeftEncoder.getPosition(),
        m_frontRightEncoder.getPosition(),
        m_rearRightEncoder.getPosition());
  }


   public ChassisSpeeds getChassisSpeeds(){
    // Convert to chassis speeds
    ChassisSpeeds chassisSpeeds = PhysicalConstants.kDriveKinematics.toChassisSpeeds(getCurrentWheelSpeeds());
    return chassisSpeeds;
   }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  
   public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
    m_gyro.setAngleAdjustment(90);
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return -m_gyro.getRotation2d().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -m_gyro.getRate();
  }

  public boolean getFieldRelative() {
    return m_gyro.isConnected();
  }

    /*
    // Set up custom logging to add the current path to a field 2d widget
    PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

    SmartDashboard.putData("Field", field);
    */

  public void showTelemetry(){
    // Update the Shuffleboard entries with current values
FLDistanceEntry.setDouble(m_frontLeftEncoder.getPosition());
    RLDistanceEntry.setDouble(m_rearLeftEncoder.getPosition());
    FRDistanceEntry.setDouble(m_frontRightEncoder.getPosition());
    RRDistanceEntry.setDouble(m_rearRightEncoder.getPosition());

    FLVoltageEntry.setDouble(m_frontLeft.getBusVoltage());
    RLVoltageEntry.setDouble(m_rearLeft.getBusVoltage());
    FRVoltageEntry.setDouble(m_frontRight.getBusVoltage());
    RRVoltageEntry.setDouble(m_rearRight.getBusVoltage());
    
    FLRateEntry.setDouble(m_frontLeftEncoder.getVelocity());
    RLRateEntry.setDouble(m_rearLeftEncoder.getVelocity());
    FRRateEntry.setDouble(m_frontRightEncoder.getVelocity());
    RRRateEntry.setDouble(m_rearRightEncoder.getVelocity());
    
    headingEntry.setDouble(getHeading());
    turnRateEntry.setDouble(getTurnRate());
    fieldRelativeEntry.setBoolean(getFieldRelative());
   
  
  }

  
}
