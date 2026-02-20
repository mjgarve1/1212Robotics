// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.AutoConstants;

public final class Autos {
  /** Example static factory for an autonomous command. */
  //public static Command exampleAuto(ExampleSubsystem subsystem) {
    //return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  //}

  //
  public static Command testAuto(SwerveSubsystem swerveSubsystem){
    return Commands.sequence(
      new AutoDriveCmd(swerveSubsystem, 01, 0, 0).withTimeout(2),
      new StopSwerveCmd(swerveSubsystem).withTimeout(2),
      new AutoDriveCmd(swerveSubsystem, 0, 0, 2).withTimeout(2),
      new StopSwerveCmd(swerveSubsystem).withTimeout(2),
      new AutoDriveCmd(swerveSubsystem, 01, 0, 0).withTimeout(2),
      new StopSwerveCmd(swerveSubsystem).withTimeout(2)
    );
  }

  //
  public static Command middleAuto(SwerveSubsystem swerveSubsystem){
       return Commands.sequence(
        new AutoDriveStraightCmd(swerveSubsystem, 1, 0).withTimeout(3.5),
        new StopSwerveCmd(swerveSubsystem).withTimeout(0.1)
       );
  }

  //
  public static Command leftAuto(SwerveSubsystem swerveSubsystem){
    return Commands.sequence(
      new AutoDriveCmd(swerveSubsystem, AutoConstants.kMidDriveForwardSpeed, 0, 0).withTimeout(AutoConstants.kMidDriveForwardTime),
      new StopSwerveCmd(swerveSubsystem),
      new AutoDriveCmd(swerveSubsystem, 0, 0, (Math.PI / 4.0)).withTimeout(2),
      new StopSwerveCmd(swerveSubsystem)

    );
  }

  //
  public static Command taxiAuto(SwerveSubsystem swerveSubsystem){
    return Commands.sequence(
      new AutoDriveCmd(swerveSubsystem, -1 , 0 , 0).withTimeout(2)
    );
  }



  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
