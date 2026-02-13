// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.LadderConstants;
import frc.robot.subsystems.LadderSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TopToBottomCmdSequence extends SequentialCommandGroup {
  /** Creates a new TopToBottomCmdSequence. */
  LadderSubsystem ladderSubsystem;
  public TopToBottomCmdSequence(LadderSubsystem ladderSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.ladderSubsystem = ladderSubsystem;
    //creates a soft stopping point when going from top to bottom

    
    if (this.ladderSubsystem.getLiftEncoder() > LadderConstants.kLiftTroughSetPoint) {
      addCommands(new LadderMoveAuto(ladderSubsystem, LadderConstants.kLiftTroughSetPoint),
      new LadderMove(ladderSubsystem, LadderConstants.kLiftLowSetPoint, () -> false, () -> 0.0));
    }
    else{
      addCommands(new LadderMove(ladderSubsystem, LadderConstants.kLiftLowSetPoint, () -> false, () -> 0.0));
    }
  }
}
