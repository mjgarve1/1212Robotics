// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//hello

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LadderConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ResetGyroCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

// This class is where the bulk of the robot should be declared. Since Command-based is a
// "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
// periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
// subsystems, commands, and trigger mappings) should be declared here.
public class RobotContainer {
  // The robot's subsystems and commands are defined here

  // Swerve drive (wheel motors) subsystem
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  // Autonomous robot control configuration
  private final SendableChooser<Command> autosChooser;
  private final Command midAuto;
  private final Command taxiAuto;
  
  // Controller input configuration
  private final Joystick driverJoystickOne = new Joystick(OIConstants.kDriverControllerOnePort);
  private final Joystick driverJoystickTwo = new Joystick(OIConstants.kDriverControllerTwoPort);

  // Constructor
  // Sets default commands for subsystems
  // Creates the commands used in autonomous
  // Creates the chooser and builder for autonomous
  // Runs configure bindings method which assigns commands to buttons
  public RobotContainer() {

    // Hook up the swerve subsystem to the Swerve Joystick Command (take controller joystick and translate it to direction/rotation/velocity)
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
              swerveSubsystem,
              () -> -driverJoystickOne.getRawAxis(OIConstants.kRobotForwardAxis),
              () -> driverJoystickOne.getRawAxis(OIConstants.kRobotSidewaysAxis),
              () -> driverJoystickOne.getRawAxis(OIConstants.kRobotRotateAxis),
              () -> !driverJoystickOne.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
              () -> driverJoystickOne.getRawButton(OIConstants.kFineTurningButton),
              () -> driverJoystickOne.getRawButton(OIConstants.kAimAtGoalButton)
              ));
  
    // Creates all named commands for pathPlanner
    // Lets fix everything else before we touch this....
    
    // Set up autonomous mode to be able to do something
    // Lets fix everything else before we touch this....
    autosChooser = new SendableChooser<>();
    midAuto = Autos.middleAuto(swerveSubsystem);
    taxiAuto = Autos.taxiAuto(swerveSubsystem);

    // Default to Middle Auto, which works out to drive the robot forward
    autosChooser.setDefaultOption("Middle Auto", midAuto);
    // No idea what taxiAuto is, figure it out later
    autosChooser.addOption("taxiAuto", taxiAuto);

    // Add the ability to swap these around (useful in competition to choose
    // a different autonomous operation depending on starting position)
    SmartDashboard.putData("Autos Chooser", autosChooser);    


    //Add in the ability to tune the ladder
    SmartDashboard.putNumber("Top Ladder", LadderConstants.kLiftHighSetPoint);
    SmartDashboard.putNumber("Middle Ladder", LadderConstants.kLiftMidSetPoint);
    SmartDashboard.putNumber("Bottom Ladder", LadderConstants.kLiftLowSetPoint);
    SmartDashboard.putNumber("Trough Ladder", LadderConstants.kLiftTroughSetPoint);
    SmartDashboard.putNumber("Ladder P", LadderConstants.kLiftPVal);
    SmartDashboard.putNumber("Ladder I", LadderConstants.kLiftIVal);
    SmartDashboard.putNumber("Ladder D", LadderConstants.kLiftDVal);

    // Configure button mapping
    configureBindings();
  }

  /// configureBindings
  /// Button bindings are configured by creating a new button object
  /// Once assigned, the pressing of the button returns a boolean which is used to run a command
  /// These commands can be toggled when the button is pressed, (toggleOnTrue)
  /// Run only while the button is pressed, (whileTrue)
  /// or a number of other options (on release, on activation, when switching from on to off OR off to on etc...)
  private void configureBindings()
  {
    // Controller One Button Mapping

    // While the button is pressed, move the climb arm in towards the middle of the robot
    //new JoystickButton(driverJoystickOne, OIConstants.kClimberIn).whileTrue(new ClimbCmd(climbSubsystem, ClimbConstants.kClimbInSpeed));

    // While the button is pressed, move the climb arm out away from the robot to climb
    //new JoystickButton(driverJoystickOne, OIConstants.kClimberOut).whileTrue(new ClimbCmd(climbSubsystem, ClimbConstants.kClimbOutSpeed));

    // While this button is pressed, reset the gyro used to tell the robot which direction is forward
    // Likely a suspect in robot orientation setup.....
    new JoystickButton(driverJoystickOne, OIConstants.kResetGyroButton).whileTrue(new ResetGyroCmd(swerveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // Return the default autonomous command defined above
    return autosChooser.getSelected();
  }
}