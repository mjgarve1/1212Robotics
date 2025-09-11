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
public class BottomToTopCmdSequence extends SequentialCommandGroup {
  /** Creates a new BottomToTopCmdSequence. */
  LadderSubsystem ladderSubsystem;
  public BottomToTopCmdSequence(LadderSubsystem ladderSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.ladderSubsystem = ladderSubsystem;
    //Creates a soft stopping point to prevent slamming into top when going from bottom
    if (this.ladderSubsystem.getLiftEncoder() < LadderConstants.kLiftTroughSetPoint) {
      addCommands(new LadderMoveAuto(ladderSubsystem, LadderConstants.kLiftTroughSetPoint), 
      new LadderMove(ladderSubsystem, LadderConstants.kLiftHighSetPoint));
    }
    else{
      addCommands(new LadderMove(ladderSubsystem, LadderConstants.kLiftHighSetPoint));
    }
  }
}
