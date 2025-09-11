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

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here
  // 
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final LadderSubsystem ladderSubsystem = new LadderSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
  //private final SendableChooser<Command> autoChooser;
  private final SendableChooser<Command> autosChooser;
  private final Command midAuto;
  private final Command taxiAuto;
  
  private final Joystick driverJoystickOne = new Joystick(OIConstants.kDriverControllerOnePort);
  private final Joystick driverJoystickTwo = new Joystick(OIConstants.kDriverControllerTwoPort);
/* 
  private final UsbCamera usbCamera;
  private final CvSink cvSink;
  private final CvSource outputStream;
*/

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  /// Constructor
  /// Sets default commands for subsystems
  /// Creates the commands used in autonomous
  /// Creates the chooser and builder for autonomous
  /// Runs configure bindings method which assigns commands to buttons
  public RobotContainer() {
    
   

    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
              swerveSubsystem,
              () -> -driverJoystickOne.getRawAxis(OIConstants.kDriverYAxis),
              () -> driverJoystickOne.getRawAxis(OIConstants.kDriverXAxis),
              () -> driverJoystickOne.getRawAxis(OIConstants.kDriverRotAxisXbox),
              () -> !driverJoystickOne.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
              () -> driverJoystickOne.getRawButton(OIConstants.kFineTurningButton)));
    
    ladderSubsystem.setDefaultCommand(new LadderJoystickCmd(
              ladderSubsystem, 
              () -> driverJoystickTwo.getRawAxis(OIConstants.kDriverYAxis),
              () -> driverJoystickTwo.getRawButtonPressed(OIConstants.kLiftResetEncoderButton)));
    
    intakeSubsystem.setDefaultCommand(new SpinIntakeJoystickCmd(
              intakeSubsystem, 
              () -> driverJoystickTwo.getRawAxis(OIConstants.kSpinIntakeOutAxis),
              () -> driverJoystickTwo.getRawAxis(OIConstants.kSpinIntakeInAxis)));

    //Creates all named commands for pathPlanner
    NamedCommands.registerCommand("LadderL1", new LadderMoveAuto(ladderSubsystem, LadderConstants.kLiftTroughSetPoint));
    NamedCommands.registerCommand("LadderRecieve", new LadderMoveAuto(ladderSubsystem, LadderConstants.kLiftRecieveSetPoint));
    NamedCommands.registerCommand("LadderL2", new LadderMoveAuto(ladderSubsystem, LadderConstants.kLiftLowSetPoint));
    NamedCommands.registerCommand("LadderL3", new LadderMoveAuto(ladderSubsystem, LadderConstants.kLiftMidSetPoint));
    NamedCommands.registerCommand("LadderL4", new LadderMoveAuto(ladderSubsystem, LadderConstants.kLiftHighSetPoint));
    NamedCommands.registerCommand("Score", new SpinIntakeCmd(intakeSubsystem, -IntakeConstants.kIntakeSpeed));
    NamedCommands.registerCommand("Recieve", new SpinIntakeCmd(intakeSubsystem, IntakeConstants.kIntakeSpeed));

    //autoChooser = AutoBuilder.buildAutoChooser();

    autosChooser = new SendableChooser<>();
    midAuto = Autos.middleAuto(swerveSubsystem, intakeSubsystem);
    taxiAuto = Autos.taxiAuto(swerveSubsystem);

    autosChooser.setDefaultOption("Middle Auto", midAuto);
    autosChooser.addOption("taxiAuto", taxiAuto);

    //SmartDashboard.putData("Auto Chooser",autoChooser);

    SmartDashboard.putData("Autos Chooser", autosChooser);

    
    /* 
    usbCamera = CameraServer.startAutomaticCapture();
    usbCamera.setResolution(640, 480);
    usbCamera.setFPS(10);

    cvSink = CameraServer.getVideo();
    outputStream = CameraServer.putVideo("Rectangle", 640, 480);
    */


    
   
    PathfindingCommand.warmupCommand().schedule();

    // Configure the trigger bindings
    configureBindings();
  }

  /// configureBindings
  /// Button bindings are configured by creating a new button object
  /// Joystick buttons are assigned to a joystick and then a button on the joystick
  /// Once assigned, the pressing of the button returns a boolean which is used to run a command
  /// These commands can be toggled when the button is pressed, (toggleOnTrue)
  /// Run only while the button is pressed, (whileTrue)
  /// or a number of other options.
  private void configureBindings() {

    //Ladder now on controller two with joystick to manually control height.
    //toggle on true to make the robot stay at a setpoint until another command is given.
    //Command Sequences used for Bottom and Top because it is too "fast" I guess, they(Bernie) want it slower when going far but faster when going short.
    //afsfnaklvnagojlfkdlb more pid tuning needed
    new JoystickButton(driverJoystickTwo, OIConstants.kLiftHighButton).toggleOnTrue(new LadderMove(ladderSubsystem, LadderConstants.kLiftHighSetPoint));
    //new JoystickButton(driverJoystickTwo, OIConstants.kLiftHighButton).toggleOnTrue(new BottomToTopCmdSequence(ladderSubsystem)); //This is Bernie's idea. 

    new JoystickButton(driverJoystickTwo, OIConstants.kLiftMidButton).toggleOnTrue(new LadderMove(ladderSubsystem, LadderConstants.kLiftMidSetPoint));
    
    new JoystickButton(driverJoystickTwo, OIConstants.kLiftLowButton).toggleOnTrue(new LadderMove(ladderSubsystem, LadderConstants.kLiftLowSetPoint));
    //new JoystickButton(driverJoystickTwo, OIConstants.kLiftLowButton).toggleOnTrue(new TopToBottomCmdSequence(ladderSubsystem));

    new JoystickButton(driverJoystickTwo, OIConstants.kliftTroughButton).toggleOnTrue(new LadderMove(ladderSubsystem, LadderConstants.kLiftTroughSetPoint));
    
    //recieve and bottom are same, don't need extra button, will hold on just in case
    //new JoystickButton(driverJoystickTwo, OIConstants.kLiftRecieveButton).toggleOnTrue(new LadderMove(ladderSubsystem, LadderConstants.kLiftRecieveSetPoint));

    //Reset encoder has been wierd, I can try to fix it when I get the chance.
    //-new JoystickButton(driverJoystickTwo, OIConstants.kLiftResetEncoderButton).whileTrue(new ResetLadderEncoder(ladderSubsystem));

    //these two button move the ladder without a setPoint
    //new JoystickButton(driverJoystickTwo, OIConstants.kliftSpeedUpButton).whileTrue(new LadderShift(ladderSubsystem, LadderConstants.kLiftSpeedUp));
    //new JoystickButton(driverJoystickTwo, OIConstants.kliftSpeedDownButton).whileTrue(new LadderShift(ladderSubsystem, LadderConstants.kliftSpeedDown));

    //Spins our intake in and out, we haven't attached sensors and we're running out of time
    new JoystickButton(driverJoystickTwo, OIConstants.kIntakeInButton).whileTrue(new SpinIntakeCmd(intakeSubsystem, IntakeConstants.kIntakeSpeed));
    new JoystickButton(driverJoystickTwo, OIConstants.kIntakeOutButton).whileTrue(new CoralSenseIntakeCmd(intakeSubsystem));  
    
    //Spins Climb in and out, the robot needs to be strong enough to hold itself up 
    new JoystickButton(driverJoystickOne, OIConstants.kClimberIn).whileTrue(new ClimbCmd(climbSubsystem, ClimbConstants.kClimbInSpeed, ClimbConstants.kClimbInStop));
    new JoystickButton(driverJoystickOne, OIConstants.kClimberOut).whileTrue(new ClimbCmd(climbSubsystem, ClimbConstants.kClimbOutSpeed, ClimbConstants.kOutStop));

    
    //simple command to zero heading of robot, hopefully it works despite potential issues.
    new JoystickButton(driverJoystickOne, OIConstants.kResetGyroButton).whileTrue(new ResetGyroCmd(swerveSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    // Pathplanner
    //return autoChooser.getSelected();
    
    // Autos
    return autosChooser.getSelected();

    //return Autos.middleAuto(swerveSubsystem, intakeSubsystem);
  }
}