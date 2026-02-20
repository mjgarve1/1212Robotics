// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralSenseIntakeCmd extends Command {
  /** Creates a new CoralSenseIntakeCmd. */
  private IntakeSubsystem intakeSubsystem;
  private boolean initialMoveState;
  
  public CoralSenseIntakeCmd(IntakeSubsystem intakeSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Initialize if the robot currently sees coral
    //
    // If there is already coral there then the robot
    // turns the wheel until it stops seeing the coral.
    //
    // If the coral is NOT there already, that means
    // it needs to spin the wheel until it detects
    // the coral and then stop when it stops
    // detecting coral.
    //
    // Essentially:
    // Coral already there? Go until its not
    // No coral? Go until we see it and then stop seeing it again
    initialMoveState = !intakeSubsystem.isCoralSensed();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    boolean isCoralSensed = intakeSubsystem.isCoralSensed();

    // If we sense coral at all, clear initialMoveState
    initialMoveState = initialMoveState && !isCoralSensed;

    // If the robot initially needs to move the coral
    // or if the robot detects coral, then the wheel needs
    // to spin.
    if( initialMoveState || isCoralSensed )
    {
      //intakeSubsystem.spinMotor(IntakeConstants.kIntakeSpeed / 3.0);
    }
    else
    {
      // Not supposed to move the coral anymore
      intakeSubsystem.spinMotor(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    // No matter what, when the button is no longer pressed,
    // stop moving the wheel
    intakeSubsystem.spinMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
