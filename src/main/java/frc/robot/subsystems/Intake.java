// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  //ProfiledPIDSubsystem {

  private final CANSparkMax m_intakeMotor = new CANSparkMax (IntakeConstants.kIntakeMotor,MotorType.kBrushless);
  private final CANSparkMax m_deployMotor = new CANSparkMax (IntakeConstants.kDeployMotor,MotorType.kBrushless);

  private final DigitalInput m_limitUp = new DigitalInput (IntakeConstants.kIntakeLimitUp);
  private final DigitalInput m_limitDown = new DigitalInput (IntakeConstants.kIntakeLimitUp);
  private final RelativeEncoder m_intakePositionEncoder;

  private final ArmFeedforward m_deployFeedforward = new ArmFeedforward(IntakeConstants.kDeploySVolts,IntakeConstants.kDeployGVolts,IntakeConstants.kDeployVVoltsSecondPerDeg);

  private final DigitalInput m_intakeStop = new DigitalInput(IntakeConstants.kIntakeStop);
  //declare sensor

  //Shuffleboard
  private final ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
  private GenericEntry intakeLimitUp;
  private GenericEntry intakeLimitDown;
 
  private final PIDController intakePID;


  /** Creates a new Intake. */
  public Intake() {

  intakePID = new PIDController(IntakeConstants.kDeployP,
                                IntakeConstants.kDeployI,
                                IntakeConstants.kDeployD);

    m_intakeMotor.restoreFactoryDefaults();
    m_deployMotor.restoreFactoryDefaults();

    m_intakeMotor.setIdleMode(IdleMode.kBrake);
    m_deployMotor.setIdleMode(IdleMode.kBrake);

    m_intakeMotor.setInverted(IntakeConstants.kIntakeMotorReversed);
    m_deployMotor.setInverted(IntakeConstants.kDeployMotorReversed);

    m_intakePositionEncoder = m_deployMotor.getEncoder();
    m_intakePositionEncoder.setPosition (0);
    m_intakePositionEncoder.setPositionConversionFactor(IntakeConstants.kDeployDistancePerPulse);
    

    //Shuffleboard
    intakeLimitUp = intakeTab.add("Intake Up",0).withWidget("Boolean Box").withSize(1,1).getEntry();
    intakeLimitDown = intakeTab.add("Intake Down",0).withWidget("Boolean Box").withSize(1,1).getEntry();

  }
  /* 
  @Override
  public void useOutput (double output, TrapezoidProfile.State setPoint){
    // Calculate the feed forward
    double feedforward = m_deployFeedforward.calculate(setPoint.position,setPoint.velocity);
    // Add feed forward to PID
    m_deployMotor.setVoltage(output+feedforward);
  }

  @Override
  public double getMeasurement(){
    return m_intakePositionEncoder.getPosition() + 0.0; 
  }
  */
  



  public void showIntakeTelemetry(){
    intakeLimitUp.setBoolean(m_limitUp.get());
    intakeLimitDown.setBoolean(m_limitDown.get());
  }
  public void runIntake(){
    if (m_intakeStop.get()){
        stopIntake(); 
    }else{
      m_intakeMotor.set(IntakeConstants.kIntakeSpeed);
    }
  }

  public void reverseIntake(){
    m_intakeMotor.set(-IntakeConstants.kIntakeSpeed);
  }

  public void stopIntake(){
    m_intakeMotor.set(0.0);
  }
  
  public void raiseIntake(){
    double setPoint = Math.min(IntakeConstants.kDeployedPosition, getIntakePositionDegrees());
    intakePID.setSetpoint(setPoint);
    m_intakeMotor.setVoltage(intakePID.calculate(getIntakePositionDegrees()) +
        m_deployFeedforward.calculate(setPoint, 0.0));
  }

  public void lowerIntake(){
    double setPoint = Math.max(IntakeConstants.kStowedPosition, getIntakePositionDegrees());
    intakePID.setSetpoint(setPoint);
    m_intakeMotor.setVoltage(intakePID.calculate(getIntakePositionDegrees()) +
        m_deployFeedforward.calculate(setPoint,0.0));
  }
  
  public void setAmpPosition(){
    double setPoint = Math.max(IntakeConstants.kAmpPosition, getIntakePositionDegrees()); //TODO: check math function
    intakePID.setSetpoint(setPoint);
    m_intakeMotor.setVoltage(intakePID.calculate(getIntakePositionDegrees()) +
        m_deployFeedforward.calculate(setPoint, 0.0));
  }

  public boolean isIntakeSetPoint(){
    return intakePID.atSetpoint();
  }

  public double getIntakePositionDegrees(){
    return m_intakePositionEncoder.getPosition()*360.0/125.0;
  }

  public void periodic(){
    SmartDashboard.putNumber("Intake Arm Position", getIntakePositionDegrees());
  }

}
