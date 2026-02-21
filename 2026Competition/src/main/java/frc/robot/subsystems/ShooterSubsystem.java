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

    public ShooterSubsystem(int sparkMaxId, MotorType motorType) {
        // need id and motor type
        shooterMotorLeader = new SparkMax(2, MotorType.kBrushless);
        shooterMotorFollower = new SparkMax(52, MotorType.kBrushless);
        encoder = shooterMotorFollower.getEncoder();

        SparkMaxConfig shooterMotorLeaderConfig = new SparkMaxConfig();
        SparkMaxConfig shooterMotorFollowerConfig = new SparkMaxConfig();

        shooterMotorLeaderConfig
                .inverted(false);

        shooterMotorFollowerConfig
                .follow(shooterMotorLeader)
                .inverted(true);

        // how to follow?
        //shooterMotorFollower.follow(shooterMotorLeader);
    
        shooterMotorLeader.configure(shooterMotorLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shooterMotorFollower.configure(shooterMotorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void spinMotor(double speed) {
        shooterMotorLeader.set(speed);
    }
    public double getEncoderPosition(){
    return encoder.getPosition();
  }

  public void resetEncoderPosition(){
    encoder.setPosition(0);
  }

  public void setSpeed(double speed){
    shooterMotorLeader.set(speed);
  }

  @Override
  public void periodic() {

}
}
