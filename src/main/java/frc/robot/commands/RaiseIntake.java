// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class RaiseIntake extends Command {
  /** Creates a new RaiseIntake. */
  private final Intake intake;
  public RaiseIntake(Intake intake) {
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.raiseIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //nothing to do here, the intake is being raised in the initialize() method
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
  }
}
