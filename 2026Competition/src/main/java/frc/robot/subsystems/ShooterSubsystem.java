package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterSubsystem {
     private static final SparkMaxConfig AbsoluteEncoderConfig = null;
         private SparkMax shooterMotorLeader;
          private SparkMax shooterMotorFollower;
     
     
         public ShooterSubsystem(){
             //need id and motor type
             shooterMotorLeader = new SparkMax(103, null);
             shooterMotorFollower = new SparkMax(104, null);
     
             SparkMaxConfig shooterMotorLeaderConfig = new SparkMaxConfig();
             SparkMaxConfig shooterMotorFollowerConfig = new SparkMaxConfig();
     
             shooterMotorLeaderConfig
                 .inverted(false);

             shooterMotorFollowerConfig
                 .follow(shooterMotorLeader)
                 .inverted(true);

        //how to follow?
        //shooterMotorFollower.follow(shooterMotorLeader);
        }

    public void spinMotor(double speed){
        shooterMotorLeader.set(speed);
    }
}
