// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LadderConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.LadderSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LadderJoystickCmd extends Command {
  /** Creates a new LadderJoystickCmd. */

  private final LadderSubsystem ladderSubsystem;
  private final Supplier<Double> speedFunction;
  private final Supplier<Boolean> offsetResetSupplier;
  public LadderJoystickCmd(LadderSubsystem ladderSubsystem, Supplier<Double> spdFunction, Supplier<Boolean> offsetReset) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ladderSubsystem = ladderSubsystem;
    speedFunction = spdFunction;
    offsetResetSupplier = offsetReset;
    addRequirements(ladderSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //1. get real time joystick input
    if(offsetResetSupplier.get()){
      LadderSubsystem.setOffset(ladderSubsystem.getLiftEncoder());
    }
    double speed = speedFunction.get();

    //2. apply deadband
    speed = Math.abs(speed) > OIConstants.kControllerAxisDeadband ? speed : 0;

    //3. Apply speed limit for up and down
    if (speed > 0)
      speed*= LadderConstants.kLiftSpeedUp;
    else if (speed< 0)
      speed*= LadderConstants.kliftSpeedDown;

    //4. Output speed to motor

    ladderSubsystem.driveLift(-speed);

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
