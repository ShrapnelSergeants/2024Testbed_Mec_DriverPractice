// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import frc.robot.Constants.AutoConstants;
//import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
//import frc.robot.Constants.PhysicalConstants;
//import frc.robot.commands.Autos;
//import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.Drivetrain;
//import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;

import java.lang.management.OperatingSystemMXBean;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;

//import java.util.List;

//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.controller.ProfiledPIDController;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.math.trajectory.Trajectory;
//import edu.wpi.first.math.trajectory.TrajectoryConfig;
//import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.MecanumControllerCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.POVButton;




/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final Drivetrain m_robotDrive = new Drivetrain();
  //private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();

  private final SendableChooser<Command> autoChooser;

  // Register Named Commands TODO: add any commands here
  //NamedCommands.registerCommand("autoBalance", swerve.autoBalanceCommand());
  //NamedCommands.registerCommand("exampleCommand", exampleSubsystem.exampleCommand());
  //NamedCommands.registerCommand("someOtherCommand", new SomeOtherCommand());
  

  // The driver's controller
  //XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  Joystick m_LeftDriverJoystick = new Joystick(OperatorConstants.kDriverLeftControllerPort);
  Joystick m_RightDriverJoystick = new Joystick(OperatorConstants.kDriverRightControllerPort);
  XboxController m_opController = new XboxController(OperatorConstants.kOpPort);
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right.
        new RunCommand(
          
            () ->
                m_robotDrive.drive(
                    m_LeftDriverJoystick.getX(),
                    m_LeftDriverJoystick.getY(),
                    -m_RightDriverJoystick.getX(),
                    //m_driverController.getLeftX(),
                    //m_driverController.getLeftY(),
                    //-m_driverController.getRightX(),
                    m_robotDrive.getFieldRelative(),
                    0.02), //should this be 0.02?
            m_robotDrive)
            

            );


    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the right bumper is held
    
    new JoystickButton(m_RightDriverJoystick, 1)
        .onTrue(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.1))) //1
        .onFalse(new InstantCommand(() -> m_robotDrive.setMaxOutput(0.5)));
/* 
    new JoystickButton(m_opController, 6)
        .onTrue(new InstantCommand(() -> m_shooter.runShooter()))
        .onFalse(new InstantCommand(() -> m_shooter.stopShooter()));
*/
    new JoystickButton(m_opController, Button.kA.value)
        .onTrue(new InstantCommand(() -> m_intake.runIntake()))
        .onFalse(new InstantCommand(() -> m_intake.stopIntake()));
    
    new JoystickButton(m_opController, Button.kB.value)
        .onTrue(new InstantCommand(() -> m_intake.reverseIntake()))
        .onFalse(new InstantCommand(() -> m_intake.stopIntake()));

    new POVButton((m_opController), 0)
        .onTrue(new InstantCommand(() -> m_intake.raiseIntake()))
        .onFalse(new InstantCommand(() -> m_intake.stopArm()));

    new POVButton((m_opController), 180)
        .onTrue(new InstantCommand(() -> m_intake.lowerIntake()))
        .onFalse(new InstantCommand(() -> m_intake.stopArm()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

    
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}
