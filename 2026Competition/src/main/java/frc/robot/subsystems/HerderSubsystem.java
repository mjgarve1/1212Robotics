package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

public class HerderSubsystem {
     private SparkMax herderMotor;

    public HerderSubsystem(){
        //need id and motor type
        herderMotor = new SparkMax(101, null);
    }

    public void spinMotor(double speed){
        herderMotor.set(speed);
    }
}

