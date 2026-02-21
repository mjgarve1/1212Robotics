// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.GenericMotorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShooterJoystickCmd extends Command {
  /** Creates a new GenericJoystickCmd. */

  private final ShooterSubsystem shooterSubsystem;
  private final Supplier<Double> speedFunction;
  public ShooterJoystickCmd(ShooterSubsystem shooterSubsystem, Supplier<Double> spdFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    speedFunction = spdFunction;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //1. get real time joystick input
    double speed = speedFunction.get();
    //2. apply deadband
    speed = Math.abs(speed) > OIConstants.kControllerAxisDeadband ? speed : 0;

    //3. Output speed to motor

    shooterSubsystem.setSpeed(ShooterConstants.kShooterMotorSpeed);

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
