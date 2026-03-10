// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BeltConstants;
import frc.robot.Constants.HerderConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.BeltSubsystem;
import frc.robot.subsystems.HerderSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterJoystickCmd extends Command {
  /** Creates a new GenericJoystickCmd. */

  private final ShooterSubsystem m_shooterSubsystem;
  private final Supplier<Double> m_shootFunction;
  private final Supplier<Double> m_herdFunction;
  private final Supplier<Boolean> m_dumpFunction;
  private final BeltSubsystem m_beltSubsystem;
  private final HerderSubsystem m_herderSubsystem;
  private final SwerveSubsystem m_swerveSubsystem;
  public ShooterJoystickCmd(ShooterSubsystem shooterSubsystem, 
  BeltSubsystem beltSubsystem, HerderSubsystem herderSubsystem, SwerveSubsystem swerveSubsystem, Supplier<Double> shootFunction, Supplier<Double> herdFunction, Supplier<Boolean> dumpFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooterSubsystem = shooterSubsystem;
    m_shootFunction = shootFunction;
    m_herdFunction = herdFunction;
    m_dumpFunction = dumpFunction;
    m_swerveSubsystem = swerveSubsystem;
    m_beltSubsystem = beltSubsystem;
    m_herderSubsystem = herderSubsystem;
    addRequirements(shooterSubsystem, beltSubsystem, herderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //1. get real time joystick input
    double shootSpeed = m_shootFunction.get();
    double herdSpeed = m_herdFunction.get();
    boolean dump = m_dumpFunction.get();
    //2. apply deadband
    if(Math.abs(shootSpeed) > OIConstants.kTriggerDeadband) {
      m_beltSubsystem.setSpeed(BeltConstants.kBeltInSpeed);
      m_herderSubsystem.setHerderSpeed(HerderConstants.kHerderInSpeed);
      // calculate shooter speed based on distance to goal
      double distanceToGoal = m_swerveSubsystem.getGoalDistance();
      // simple linear relationship between distance and shooter speed (tune as necessary)
      double shooterSpeed = ShooterConstants.kShooterMotorSpeed * (distanceToGoal / ShooterConstants.kMaxGoalDistance);
      if(distanceToGoal > ShooterConstants.kMaxGoalDistance) {
        shooterSpeed = ShooterConstants.kShooterMotorSpeed; // cap at max speed
      }
      else if(distanceToGoal < ShooterConstants.kMinGoalDistance) {
        shooterSpeed = ShooterConstants.kShooterMotorSpeed * 0.1; // minimum speed to prevent jamming
      }
      m_shooterSubsystem.setSpeed(shooterSpeed);
    }
    else if (Math.abs(herdSpeed) > OIConstants.kTriggerDeadband) {
      m_beltSubsystem.setSpeed(BeltConstants.kBeltInSpeed);
      m_herderSubsystem.setHerderSpeed(HerderConstants.kHerderInSpeed);
      m_shooterSubsystem.setSpeed(ShooterConstants.kBackShooterMotorSpeed);
    }
    else if(dump) {
      m_beltSubsystem.setSpeed(BeltConstants.kBeltOutSpeed);
      m_herderSubsystem.setHerderSpeed(HerderConstants.kHerderOutSpeed);
      m_shooterSubsystem.setSpeed(ShooterConstants.kBackShooterMotorSpeed);
    }
    else {
      m_shooterSubsystem.setSpeed(0);
      m_beltSubsystem.setSpeed(0);
      m_herderSubsystem.setHerderSpeed(0);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
