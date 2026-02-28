package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

/** Drive forward until a valid vision pose estimate is available or timeout (2s) */
public class DriveForwardUntilVisionCmd extends Command {
  private final SwerveSubsystem swerve;
  private final Timer timer = new Timer();
  private boolean gotVision = false;

  public DriveForwardUntilVisionCmd(SwerveSubsystem swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    gotVision = false;
  }

  @Override
  public void execute() {
    // Drive forward at 1 m/s (field-relative handled in subsystem)
    ChassisSpeeds speeds = new ChassisSpeeds(1.0, 0.0, 0.0);
    swerve.setChassisSpeed(speeds);
    swerve.setModuleStates();

    // Poll Limelight for a pose estimate (blue frame)
    LimelightHelpers.PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if (est != null && est.tagCount > 0) {
      // Acceptable vision estimate found: reset pose estimator to this pose and finish
      swerve.zeroHeading(est.pose.getRotation().getDegrees());
      swerve.resetPose(est.pose);
      gotVision = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.driveRobotRelative(new ChassisSpeeds(0, 0, 0));
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    // Finish if we got a vision estimate or 2 seconds have elapsed
    return gotVision || timer.hasElapsed(2.0);
  }
}
