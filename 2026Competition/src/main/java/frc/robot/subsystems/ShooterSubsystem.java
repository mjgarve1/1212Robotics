package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
    private final RelativeEncoder encoder;
    private SparkMax shooterMotorLeader;
    private SparkMax shooterMotorFollower;
    private SparkMaxConfig config;

    public ShooterSubsystem(int sparkMaxId, int sparkMaxFollowerId) {
        // need id and motor type
        shooterMotorLeader = new SparkMax(sparkMaxId, MotorType.kBrushless);
        shooterMotorFollower = new SparkMax(sparkMaxFollowerId, MotorType.kBrushless);
        encoder = shooterMotorFollower.getEncoder();

        SparkMaxConfig shooterMotorLeaderConfig = new SparkMaxConfig();
        SparkMaxConfig shooterMotorFollowerConfig = new SparkMaxConfig();

        shooterMotorLeaderConfig
                .inverted(false);

        shooterMotorFollowerConfig
                .inverted(true);

        // how to follow?
        //shooterMotorFollower.follow(shooterMotorLeader);
    
        shooterMotorLeader.configure(shooterMotorLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotorFollower.configure(shooterMotorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void spinMotor(double speed) {
        shooterMotorLeader.set(speed);
        shooterMotorFollower.set(speed);
    }
    public double getEncoderPosition(){
    return encoder.getPosition();
  }

  public void resetEncoderPosition(){
    encoder.setPosition(0);
  }

  public void setSpeed(double speed){
    shooterMotorLeader.set(speed);
    shooterMotorFollower.set(speed);
  }

  public void calculateAndSetSpeed(SwerveSubsystem swerveSubsystem) {
  
    double calculatedSpeed = 0; // Replace with actual calculation
    //Get the distance here, similar to how the robot orientation calculation happens in the SwerveJoystickCmd
    //Test shooting distance at different motor speeds and figure out how to translate that
    setSpeed(calculatedSpeed);
  }

  @Override
  public void periodic() {

}
}
