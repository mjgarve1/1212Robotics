// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.oldLadderCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LadderConstants;
import frc.robot.subsystems.LadderSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LadderTrough extends Command {
  /** Creates a new LadderTrough. */
  private final LadderSubsystem ladderSub;
  private final PIDController m_PidController;


  public LadderTrough(LadderSubsystem ladderSub) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.m_PidController = new PIDController(LadderConstants.kLiftPVal, LadderConstants.kLiftIVal, LadderConstants.kLiftDVal);
    m_PidController.setSetpoint(LadderConstants.kLiftTroughSetPoint);

    this.ladderSub = ladderSub;
    
    addRequirements(ladderSub);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = m_PidController.calculate(ladderSub.getLiftEncoder());

    ladderSub.driveLift(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ladderSub.driveLift(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (ladderSub.getLiftEncoder() == LadderConstants.kLiftTroughSetPoint)
      return true;
    return false;
  }
}
