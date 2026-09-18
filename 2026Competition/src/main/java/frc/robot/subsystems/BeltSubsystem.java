package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class BeltSubsystem extends SubsystemBase{
  private final SparkMax beltMotorLeader;
  //private final SparkMax beltMotorFollower;
  private final RelativeEncoder beltEncoder;
  private SparkMaxConfig config;

  public BeltSubsystem(int sparkMaxId, int sparkMaxFollowerId) {
    beltMotorLeader = new SparkMax(sparkMaxId, MotorType.kBrushless);
   // beltMotorFollower = new SparkMax(sparkMaxFollowerId, MotorType.kBrushless); 
    beltEncoder = beltMotorLeader.getEncoder(); 

    SparkMaxConfig beltMotorLeaderConfig = new SparkMaxConfig();
    //SparkMaxConfig beltMotorFollowerConfig = new SparkMaxConfig();

    beltMotorLeaderConfig
            .inverted(false);

   // beltMotorFollowerConfig
     //       .inverted(true);

    /* 
     config.idleMode(IdleMode.kBrake);
    
   //if (DriverStation.isAutonomous()) {
     config.smartCurrentLimit(25,30);
    } 
    else if (DriverStation.isTeleop()) {
      config.smartCurrentLimit(22,30);  //limit current for beltmotor
    };
    */
    beltMotorLeader.configure(beltMotorLeaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //beltMotorFollower.configure(beltMotorFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }       
  
  public double getEncoderPosition(){
    return beltEncoder.getPosition();
  }

  public void resetEncoderPosition(){
    beltEncoder.setPosition(0);
  }

  public void setSpeed(double speed){
    beltMotorLeader.set(speed);
    //beltMotorFollower.set(speed);
  }

  @Override
  public void periodic() {

  }
}
