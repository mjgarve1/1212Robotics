// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.LadderConstants;
import frc.robot.subsystems.LadderSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

///
/// The key difference between LadderMoveAuto and LadderMove is the the isFinished method
/// The LadderMove command won't stop until it gets overriden, this keeps it in place and fights gravity
/// LadderMoveAuto stops once it reaches its setpoint, this is based on how I believe auto works
/// In which the auto won't move on until the previous command finishes
public class LadderMoveAuto extends Command {
  private final LadderSubsystem ladderSub;
  private final PIDController m_PidController;


  private final double setPoint;
  private boolean isAbove;

  public LadderMoveAuto(LadderSubsystem ladderSub, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.setPoint = setPoint;
    this.m_PidController = new PIDController(LadderConstants.kLiftPVal, LadderConstants.kLiftIVal, LadderConstants.kLiftDVal);
    m_PidController.setSetpoint(this.setPoint);

    this.ladderSub = ladderSub;

    isAbove = setPoint < ladderSub.getLiftEncoder();
      
    
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
    ladderSub.driveLift(LadderConstants.kStop);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // checks encoder to see if it has passed the setpoint
    if((isAbove && ladderSub.getLiftEncoder() >= setPoint)
    || (!isAbove && ladderSub.getLiftEncoder() <= setPoint)){
      return true;
    }
    return false;
  }
}
