// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.HerderSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class WinchJoystickCmd extends Command {
  /** Creates a new GenericJoystickCmd. */

  private final Supplier<Double> m_herdFunction;
  private final HerderSubsystem m_herderSubsystem;
  public WinchJoystickCmd(HerderSubsystem herderSubsystem, Supplier<Double> herdFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_herdFunction = herdFunction;
    m_herderSubsystem = herderSubsystem;
    addRequirements(herderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //1. get real time joystick input
    double herdSpeed = m_herdFunction.get();
    //2. apply deadband
    herdSpeed = Math.abs(herdSpeed) > OIConstants.kControllerAxisDeadband ? herdSpeed : 0.0;

    m_herderSubsystem.setWinchSpeed(herdSpeed);
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
