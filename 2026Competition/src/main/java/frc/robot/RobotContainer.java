// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//hello

package frc.robot;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.BeltConstants;
import frc.robot.Constants.HerderConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ResetGyroCmd;
import frc.robot.commands.ShooterJoystickCmd;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.HerderSubsystem;
import frc.robot.commands.WinchJoystickCmd;

// This class is where the bulk of the robot should be declared. Since Command-based is a
// "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
// periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
// subsystems, commands, and trigger mappings) should be declared here.
public class RobotContainer {
  // The robot's subsystems and commands are defined here

  // Swerve drive (wheel motors) subsystem
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final ShooterSubsystem shooterSubsystem =
      new ShooterSubsystem(ShooterConstants.kShooter1MotorPort, ShooterConstants.kShooter2MotorPort);


  // PROGRAMMER COMMENT
  // Create a new generic motor subsystem for each motor that exists
  private final HerderSubsystem herderSubsystem = new HerderSubsystem(HerderConstants.kHerderMotorPort, HerderConstants.kWinchMotorPort);
  private final BeltSubsystem beltSubsystem = new BeltSubsystem(BeltConstants.kBelt1MotorPort, BeltConstants.kBelt2MotorPort);
  // Autonomous robot control configuration
  private final SendableChooser<Command> autosChooser;
  private final Command midAuto;
  private final Command taxiAuto;
  private final Command visionAcquireAuto;

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

    shooterSubsystem.setDefaultCommand(new ShooterJoystickCmd(
      shooterSubsystem, beltSubsystem, herderSubsystem, swerveSubsystem,
      () -> driverJoystickTwo.getRawAxis(OIConstants.kShootFuelButton),
      () -> driverJoystickTwo.getRawAxis(OIConstants.kHerdFuelButton),
      () -> driverJoystickTwo.getRawButton(OIConstants.kDumpFuelButton))
      );
    herderSubsystem.setDefaultCommand(new WinchJoystickCmd(
      herderSubsystem,
      () -> driverJoystickTwo.getRawAxis(OIConstants.kWinchAxis)));
    autosChooser = new SendableChooser<>();
    midAuto = Autos.middleAuto(swerveSubsystem);
    taxiAuto = Autos.taxiAuto(swerveSubsystem);
  visionAcquireAuto = Autos.visionAcquireAuto(swerveSubsystem, beltSubsystem, shooterSubsystem);

    // Default to Middle Auto, which works out to drive the robot forward
    autosChooser.setDefaultOption("Vision Acquire Auto", visionAcquireAuto);
    // No idea what taxiAuto is, figure it out later
    autosChooser.addOption("taxiAuto", taxiAuto);
    autosChooser.addOption("Vision Acquire Auto", visionAcquireAuto);

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