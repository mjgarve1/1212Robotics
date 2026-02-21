// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//hello

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.HerderConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.GenericJoystickCmd;
import frc.robot.commands.GenericMotorMoveCmd;
import frc.robot.commands.ResetGyroCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.GenericMotorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// This class is where the bulk of the robot should be declared. Since Command-based is a
// "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
// periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
// subsystems, commands, and trigger mappings) should be declared here.
public class RobotContainer {
  // The robot's subsystems and commands are defined here

  // Swerve drive (wheel motors) subsystem
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  // PROGRAMMER COMMENT
  // Create a new generic motor subsystem for each motor that exists
  private final GenericMotorSubsystem herderSubsystem = new GenericMotorSubsystem(HerderConstants.kHerderMotorPort, MotorType.kBrushless);
  //  private final GenericMotorSubsystem herderSubsystemTwo = new GenericMotorSubsystem(HerderConstants.kHerderMotorPortTwo, MotorType.kBrushless);
  // etc...

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

    // Hook up the swerve subsystem to the Swerve Joystick Command (take controller
    // joystick and translate it to direction/rotation/velocity)
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
        swerveSubsystem,
        () -> driverJoystickOne.getRawAxis(OIConstants.kRobotForwardAxis),
        () -> driverJoystickOne.getRawAxis(OIConstants.kRobotSidewaysAxis),
        () -> driverJoystickOne.getRawAxis(OIConstants.kRobotRotateAxis),
        () -> !driverJoystickOne.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
        () -> driverJoystickOne.getRawButton(OIConstants.kFineTurningButton),
        () -> driverJoystickOne.getRawButton(OIConstants.kAimAtGoalButton)));

    // PROGRAMMER COMMENT
    // You can assign the subsystem a default command to be the joystick value
    // This will drive the motor forward or backward according to the value of the
    // joystick position with a deadband of +/- .15, so the joystick has to be past
    // 15% forward or backward before the motor will move
    // Basically, you pass in the subsystem as a parameter, the second parameter is
    // formatted like this:
    // () -> (value to return when it gets called, like a the current joystick
    // reading)
    herderSubsystem.setDefaultCommand(new GenericJoystickCmd(
        herderSubsystem,
        () -> driverJoystickTwo.getRawAxis(OIConstants.kRobotForwardAxis)));

    // PROGRAMMER COMMENT
    // Much like the comment below in the buttons function, this too can be created for
    // multiple subsystems corresponding to the same joystick input and you can
    // reverse the joystick input to drive the motor "mirrored"
    // Below is sample code to do it for a second motor

    // herderSubsystemTwo.setDefaultCommand(new GenericJoystickCmd(
    // herderSubsystemTwo,
    // () -> -driverJoystickTwo.getRawAxis(OIConstants.kRobotForwardAxis)));

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

    // Configure button mapping
    configureBindings();
  }

  /// configureBindings
  /// Button bindings are configured by creating a new button object
  /// Once assigned, the pressing of the button returns a boolean which is used to
  /// run a command
  /// These commands can be toggled when the button is pressed, (toggleOnTrue)
  /// Run only while the button is pressed, (whileTrue)
  /// or a number of other options (on release, on activation, when switching from
  /// on to off OR off to on etc...)
  private void configureBindings() {
    // Controller One Button Mapping

    // PROGRAMMER COMMENT
    // While the button is pressed, herderSubsystem will move the herder motor at
    // 100% forward
    // Send in -1.0 to move it backwards, etc...
    // You can create a new JoystickButton and do a whileTrue and pass in the same
    // subsystem to drive it a different speed/direction
    // If there are two motors that need to be driven, you can create two subsystems
    // create the command twice
    new JoystickButton(driverJoystickOne, OIConstants.kHerderIn)
        .whileTrue(new GenericMotorMoveCmd(herderSubsystem, HerderConstants.kHerderInSpeed));

    new JoystickButton(driverJoystickOne, OIConstants.kHerderOut)
        .whileTrue(new GenericMotorMoveCmd(herderSubsystem, HerderConstants.kHerderOutSpeed));

    // PROGRAMMER COMMENT
    // For example, you can duplicate the above code and create herderSubsystemTwo
    // where herderSubsystemTwo is assigned to SparkMax ID 101, and then invert the
    // speed that you send:

    // new JoystickButton(driverJoystickOne, OIConstants.kHerderIn)
    // .whileTrue(new GenericMotorMoveCmd(herderSubsystemTwo, -kHerderInSpeed));

    // new JoystickButton(driverJoystickOne, OIConstants.kHerderOut)
    // .whileTrue(new GenericMotorMoveCmd(herderSubsystemTwo, HerderConstants.kHerderOutSpeed));

    // While this button is pressed, reset the gyro used to tell the robot which
    // direction is forward
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