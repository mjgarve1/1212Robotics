// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.LadderConstants;
import frc.robot.subsystems.LadderSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LadderMove extends Command {
  private final LadderSubsystem ladderSub;
  private PIDController m_PidController;

  private double setPoint;
  private final Supplier<Boolean> unlockLadderSupplier;
  private final Supplier<Double> setPointSupplier;

  //private final Supplier<Boolean> changeOffsetfx;

  public LadderMove(LadderSubsystem ladderSub, double setPoint, Supplier<Boolean> unlockLadderSupplier, Supplier<Double> setPointSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ladderSub = ladderSub;
    this.setPoint = setPoint;
    this.m_PidController = new PIDController(LadderConstants.kLiftPVal, LadderConstants.kLiftIVal, LadderConstants.kLiftDVal);    
    this.unlockLadderSupplier = unlockLadderSupplier;
    this.setPointSupplier = setPointSupplier;
    addRequirements(ladderSub);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    setPoint = setPointSupplier.get();
    this.m_PidController = new PIDController(SmartDashboard.getNumber("Ladder P", LadderConstants.kLiftPVal), SmartDashboard.getNumber("Ladder I", LadderConstants.kLiftIVal), SmartDashboard.getNumber("Ladder D", LadderConstants.kLiftDVal)); 
    m_PidController.setSetpoint(this.setPoint + LadderSubsystem.getOffset());
  }

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
    // If the button is pressed, cancel the hold
    return unlockLadderSupplier.get();
  }
}
