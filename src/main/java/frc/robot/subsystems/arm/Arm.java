package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import frc.utils.motorcontrol.StormSpark;

import static frc.robot.Constants.*;

public class Arm {
    private boolean running;
    
    public MotorController shoulder;
    public MotorController elbow;
    
    public Arm() {
        //constructor    
        shoulder = new StormSpark(armShoulderID, CANSparkMaxLowLevel.MotorType.kBrushless, StormSpark.MotorKind.kNeo);
        elbow = new StormSpark(armElbowID, CANSparkMaxLowLevel.MotorType.kBrushless, StormSpark.MotorKind.kNeo);
    }

    /*@Override
    public void periodic() {
        shoulder.setVoltage(0);
        elbow.setVoltage(0);
        
    } */


    private void printStatus() {
   
    }
}


