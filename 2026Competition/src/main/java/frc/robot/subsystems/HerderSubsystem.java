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
    private final SparkMax winchMotor;
    private final RelativeEncoder winchEncoder;
    private SparkMaxConfig config;

   public HerderSubsystem(int sparkMaxId, int sparkMaxId2) {
    herderMotor = new SparkMax(sparkMaxId, MotorType.kBrushless);
    herderEncoder = herderMotor.getEncoder();
    winchMotor = new SparkMax(sparkMaxId2, MotorType.kBrushless);
    winchEncoder = winchMotor.getEncoder();
    
    config = new SparkMaxConfig();

    config.idleMode(IdleMode.kBrake);

    herderMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void setHerderSpeed(double speed){
    herderMotor.set(speed);
  }

  public void setWinchSpeed(double speed){
    double maxWinchLimit = 10; //example, find real limit
    double minWinchLimit = 0;
    if (winchEncoder.getPosition() > maxWinchLimit || winchEncoder.getPosition() < minWinchLimit) {
      speed = 0;
    }
    //TODO: Determine the encoder position limits for the winch and implement logic to prevent the winch from moving beyond those limits
    winchMotor.set(speed);
  }

  public double getWinchEncoderPosition(){
    return winchEncoder.getPosition();
  }

  public void resetEncoderPosition(){
    winchEncoder.setPosition(0);
  }

  @Override
  public void periodic() {

  }
}
