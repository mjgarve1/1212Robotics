// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.oldLadderCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LadderConstants;
import frc.robot.subsystems.LadderSubsystem;

public class LadderShift extends Command {
  private LadderSubsystem ladderSub;
  private double speed;
  /** Creates a new LadderUp. */
  public LadderShift(LadderSubsystem ladderSub, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ladderSub = ladderSub;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    SmartDashboard.putNumber("ManualSpeed", speed);
    ladderSub.setLastPoint(ladderSub.getLiftEncoder());
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
    if(ladderSub.getLiftEncoder() > LadderConstants.kLadderBottom || ladderSub.getLiftEncoder() < LadderConstants.kLadderTop)
      return true;
    return false;
  }
}
