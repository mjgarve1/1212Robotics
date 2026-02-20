package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

public class BeltSubsystem {
    private SparkMax beltMotor;

    public BeltSubsystem(){
        //need id and motor type
        beltMotor = new SparkMax(100, null);
    }

    public void spinMotor(double speed){
        beltMotor.set(speed);
    }
}
