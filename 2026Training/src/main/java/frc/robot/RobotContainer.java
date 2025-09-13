// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathfindingCommand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LadderConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ClimbCmd;
import frc.robot.commands.CoralSenseIntakeCmd;
import frc.robot.commands.LadderJoystickCmd;
import frc.robot.commands.LadderMove;
import frc.robot.commands.LadderMoveAuto;
import frc.robot.commands.ResetGyroCmd;
import frc.robot.commands.SpinIntakeCmd;
import frc.robot.commands.SpinIntakeJoystickCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LadderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// This class is where the bulk of the robot should be declared. Since Command-based is a
// "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
// periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
// subsystems, commands, and trigger mappings) should be declared here.
public class RobotContainer {
  // The robot's subsystems and commands are defined here

  // Swerve drive (wheel motors) subsystem
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  // Ladder up/down motor subsystem
  private final LadderSubsystem ladderSubsystem = new LadderSubsystem();

  // Coral in/out motor subsystem
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

  // Climb arm in/out motor subsystem
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();

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
              () -> driverJoystickOne.getRawButton(OIConstants.kFineTurningButton)));
    
    // Hook up the ladder to be controlled by the joystick as long as no other command is currently executing
    // Probably some improvement here since I think the buttons do not give up control when they are no longer pressed
    ladderSubsystem.setDefaultCommand(new LadderJoystickCmd(
              ladderSubsystem, 
              () -> driverJoystickTwo.getRawAxis(OIConstants.kRobotForwardAxis),
              () -> driverJoystickTwo.getRawButtonPressed(OIConstants.kLiftResetEncoderButton)));
    
    // Hook up the coral intake to take in joystick as long as no other command is currently executing
    // Probably some improvement here as above....
    intakeSubsystem.setDefaultCommand(new SpinIntakeJoystickCmd(
              intakeSubsystem, 
              () -> driverJoystickTwo.getRawAxis(OIConstants.kSpinIntakeOutAxis),
              () -> driverJoystickTwo.getRawAxis(OIConstants.kSpinIntakeInAxis)));

    // Creates all named commands for pathPlanner
    // Lets fix everything else before we touch this....
    NamedCommands.registerCommand("LadderL1", new LadderMoveAuto(ladderSubsystem, LadderConstants.kLiftTroughSetPoint));
    NamedCommands.registerCommand("LadderRecieve", new LadderMoveAuto(ladderSubsystem, LadderConstants.kLiftRecieveSetPoint));
    NamedCommands.registerCommand("LadderL2", new LadderMoveAuto(ladderSubsystem, LadderConstants.kLiftLowSetPoint));
    NamedCommands.registerCommand("LadderL3", new LadderMoveAuto(ladderSubsystem, LadderConstants.kLiftMidSetPoint));
    NamedCommands.registerCommand("LadderL4", new LadderMoveAuto(ladderSubsystem, LadderConstants.kLiftHighSetPoint));
    NamedCommands.registerCommand("Score", new SpinIntakeCmd(intakeSubsystem, -IntakeConstants.kIntakeSpeed));
    NamedCommands.registerCommand("Recieve", new SpinIntakeCmd(intakeSubsystem, IntakeConstants.kIntakeSpeed));

    // Set up autonomous mode to be able to do something
    // Lets fix everything else before we touch this....
    autosChooser = new SendableChooser<>();
    midAuto = Autos.middleAuto(swerveSubsystem, intakeSubsystem);
    taxiAuto = Autos.taxiAuto(swerveSubsystem);

    // Default to Middle Auto, which works out to drive the robot forward
    autosChooser.setDefaultOption("Middle Auto", midAuto);
    // No idea what taxiAuto is, figure it out later
    autosChooser.addOption("taxiAuto", taxiAuto);

    // Add the ability to swap these around (useful in competition to choose
    // a different autonomous operation depending on starting position)
    SmartDashboard.putData("Autos Chooser", autosChooser);    
   
    // Pathfinder
    // Lets fix everything else before we touch this....
    PathfindingCommand.warmupCommand().schedule();

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
    new JoystickButton(driverJoystickOne, OIConstants.kClimberIn).whileTrue(new ClimbCmd(climbSubsystem, ClimbConstants.kClimbInSpeed, ClimbConstants.kClimbInStop));

    // While the button is pressed, move the climb arm out away from the robot to climb
    new JoystickButton(driverJoystickOne, OIConstants.kClimberOut).whileTrue(new ClimbCmd(climbSubsystem, ClimbConstants.kClimbOutSpeed, ClimbConstants.kOutStop));

    // While this button is pressed, reset the gyro used to tell the robot which direction is forward
    // Likely a suspect in robot orientation setup.....
    new JoystickButton(driverJoystickOne, OIConstants.kResetGyroButton).whileTrue(new ResetGyroCmd(swerveSubsystem));

    // Controller Two Button Mapping
    
    // When this button is pressed, send the ladder to lowest, low, mid, top scoring positions
    // Note: These are TOGGLE, so pressing a button will hold the position permanently
    //       This may end up disabling joystick control, probably room for improvement here.
    new JoystickButton(driverJoystickTwo, OIConstants.kLiftHighButton).toggleOnTrue(new LadderMove(ladderSubsystem, LadderConstants.kLiftHighSetPoint));
    new JoystickButton(driverJoystickTwo, OIConstants.kLiftMidButton).toggleOnTrue(new LadderMove(ladderSubsystem, LadderConstants.kLiftMidSetPoint));
    new JoystickButton(driverJoystickTwo, OIConstants.kLiftLowButton).toggleOnTrue(new LadderMove(ladderSubsystem, LadderConstants.kLiftLowSetPoint));
    new JoystickButton(driverJoystickTwo, OIConstants.kliftTroughButton).toggleOnTrue(new LadderMove(ladderSubsystem, LadderConstants.kLiftTroughSetPoint));

    // While this button is pressed move the coral from the middle of the robot out at a constant fixed rate
    new JoystickButton(driverJoystickTwo, OIConstants.kIntakeInButton).whileTrue(new SpinIntakeCmd(intakeSubsystem, IntakeConstants.kIntakeSpeed));

    //While this button is pressed move the coral from the outside of the robot towards the middle at a fixed constant rate
    new JoystickButton(driverJoystickTwo, OIConstants.kIntakeOutButton).whileTrue(new CoralSenseIntakeCmd(intakeSubsystem));
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