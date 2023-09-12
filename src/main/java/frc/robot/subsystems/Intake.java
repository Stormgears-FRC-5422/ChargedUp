package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.motorcontrol.StormSpark;

import static frc.robot.constants.Constants.ArmConstants.armShoulderID;

public class Intake extends SubsystemBase {

    final double INTAKE_OUTPUT_POWER = 1.0; //change power based on tests

    final double INTAKE_HOLD_POWER = 0.07;


    StormSpark intakeMotor = new StormSpark(1,
            CANSparkMaxLowLevel.MotorType.kBrushless,
            StormSpark.MotorKind.kNeo); //NEED RIGHT DEVICE ID

    public void setIntakeMotorSpeed(double speed){
        intakeMotor.set(speed);
    }

    public void intake(){
        setIntakeMotorSpeed(INTAKE_OUTPUT_POWER);
    }

    public void hold(){
        setIntakeMotorSpeed(INTAKE_HOLD_POWER);

    }

    public void out(){
        setIntakeMotorSpeed(0);

    }

//    public Command holdCommand(){
//        return new InstantCommand(this::hold);
//    } not needed for now, may need


    public Command outCommand(){
        return new InstantCommand(this::out);
    }





}
