package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class BeltSubsystem extends SubsystemBase{
  private final SparkMax beltMotor;
  private final SparkMax beltMotor2;
  private final RelativeEncoder beltEncoder;
  private SparkMaxConfig config;

  public BeltSubsystem(int sparkMaxId, int sparkMaxId2) {
    beltMotor = new SparkMax(sparkMaxId, MotorType.kBrushless);
    beltMotor2 = new SparkMax(sparkMaxId2, MotorType.kBrushless);
    beltEncoder = beltMotor.getEncoder();
    
    config = new SparkMaxConfig();

    config.idleMode(IdleMode.kBrake);

    beltMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public double getEncoderPosition(){
    return beltEncoder.getPosition();
  }

  public void resetEncoderPosition(){
    beltEncoder.setPosition(0);
  }

  public void setSpeed(double speed){
    beltMotor.set(speed);
    beltMotor2.set(-speed);
  }

  @Override
  public void periodic() {

  }
}
