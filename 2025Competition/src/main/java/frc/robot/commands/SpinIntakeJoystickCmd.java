// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SpinIntakeJoystickCmd extends Command {
  /** Creates a new SpinIntakeJoystickCmd. */
  IntakeSubsystem intakeSubsystem;
  Supplier<Double> speedOutFunction;
  Supplier<Double> speedInFunction;
  double speed;
  public SpinIntakeJoystickCmd(IntakeSubsystem intakeSubsystem, Supplier<Double> speedOutFunction, Supplier<Double> speedInFunction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.speedInFunction = speedInFunction;
    this.speedOutFunction = speedOutFunction;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (speedInFunction.get() > OIConstants.kDeadband) {
      speed = -speedInFunction.get();
    }
    else if (speedOutFunction.get() > OIConstants.kDeadband) {
      speed = speedOutFunction.get();
    }
    else{
      speed = 0;
    }
    intakeSubsystem.spinMotor(speed/2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.spinMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
