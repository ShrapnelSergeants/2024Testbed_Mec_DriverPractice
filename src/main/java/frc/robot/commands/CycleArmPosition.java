// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

public class CycleArmPosition extends Command {
  private final Intake intake;
  private final boolean cycleUp;

  public CycleArmPosition(Intake intake, boolean cycleUp) {
    this.intake = intake;
    this.cycleUp = cycleUp;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /*if (cycleUp){
      if(intake.getIntakePositionDegrees()< IntakeConstants.kAmpPosition){
        intake.setAmpPosition();
      } else if (intake.getIntakePositionDegrees()<
      IntakeConstants.kStowedPosition){
        intake.lowerIntake();
      } else {
        intake.raiseIntake();
      }
    } else {
      if(intake.getIntakePositionDegrees()> IntakeConstants.kAmpPosition){
        intake.setAmpPosition();
      } else if (intake.getIntakePositionDegrees()>
      IntakeConstants.kDeployedPosition){
        intake.raiseIntake();
      } else {
        intake.lowerIntake();
      }

    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //nothing to do here, the arm position is being set in the initialize() method
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //nothing to do here
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.isIntakeSetPoint();
  }*/

  }
}
