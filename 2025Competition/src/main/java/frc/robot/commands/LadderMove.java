// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LadderConstants;
import frc.robot.subsystems.LadderSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LadderMove extends Command {
  private final LadderSubsystem ladderSub;
  private final PIDController m_PidController;

  private final double setPoint;

  //private final Supplier<Boolean> changeOffsetfx;

  public LadderMove(LadderSubsystem ladderSub, double setPoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ladderSub = ladderSub;
    this.setPoint = setPoint;
    this.m_PidController = new PIDController(LadderConstants.kLiftPVal, LadderConstants.kLiftIVal, LadderConstants.kLiftDVal);
   // m_PidController.setSetpoint(this.setPoint + LadderSubsystem.getOffset());


    
    
    addRequirements(ladderSub);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_PidController.setSetpoint(this.setPoint + LadderSubsystem.getOffset());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //ladderSub.setLastPoint(setPoint+ LadderSubsystem.getOffset());

    double speed = m_PidController.calculate(ladderSub.getLiftEncoder());

    ladderSub.driveLift(speed);
    SmartDashboard.putNumber("speed", speed);
    SmartDashboard.putNumber("Setpoint", setPoint);
    
    SmartDashboard.putNumber("Setpoint offset", LadderSubsystem.getOffset());
    SmartDashboard.putNumber("Error", m_PidController.getError());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ladderSub.driveLift(LadderConstants.kStop);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
