package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.utils.motorcontrol.StormSpark;

import frc.robot.constants.Constants.ArmConstants;

public class Intake extends SubsystemBase {

    //change power based on tests

    private final double inPower = ArmConstants.intakeInPower;
    private final double holdPower = ArmConstants.intakeHoldPower;

    private final double outPower = ArmConstants.intakeOutPower;



    StormSpark intakeMotor = new StormSpark(ArmConstants.intakeMotorID,
                CANSparkMaxLowLevel.MotorType.kBrushless,
                StormSpark.MotorKind.kNeo);


    public void setIntakeMotorSpeed(double speed) {
        intakeMotor.set(speed);
    }



    public void intake() {
        setIntakeMotorSpeed(inPower);
    }

    public void hold() {
        setIntakeMotorSpeed(holdPower);

    }

    public void out() {
        setIntakeMotorSpeed(outPower);

    }

//    public Command holdCommand(){
//        return new InstantCommand(this::hold);
//    } not needed for now, may need


    public Command outCommand() {
        return new InstantCommand(this::out);
    }


}
