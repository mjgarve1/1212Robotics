package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HerderSubsystem extends SubsystemBase{
    private final SparkMax herderMotor;
    private final RelativeEncoder herderEncoder;
    private SparkMaxConfig config;

   public HerderSubsystem(int sparkMaxId, MotorType motorType) {
    herderMotor = new SparkMax(sparkMaxId, motorType);
    herderEncoder = herderMotor.getEncoder();
    
    config = new SparkMaxConfig();

    config.idleMode(IdleMode.kBrake);

    herderMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public double getEncoderPosition(){
    return herderEncoder.getPosition();
  }

  public void resetEncoderPosition(){
    herderEncoder.setPosition(0);
  }

  public void setSpeed(double speed){
    herderMotor.set(speed);
  }

  @Override
  public void periodic() {

  }
}
