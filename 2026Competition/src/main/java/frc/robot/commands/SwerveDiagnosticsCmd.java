// Simple diagnostics command for swerve
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveDiagnosticsCmd extends Command {
  private final SwerveSubsystem swerve;

  public SwerveDiagnosticsCmd(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    // One-shot: compute and log diagnostic states
    swerve.logDiagnosticStates();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    // complete immediately after initialize
    return true;
  }
}
