// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveSubsystem;

public final class Autos {

  //
  public static Command middleAuto(SwerveSubsystem swerveSubsystem){
       return Commands.sequence(
        new AutoDriveStraightCmd(swerveSubsystem, 1, 0).withTimeout(3.5),
        new StopSwerveCmd(swerveSubsystem).withTimeout(0.1)
       );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
