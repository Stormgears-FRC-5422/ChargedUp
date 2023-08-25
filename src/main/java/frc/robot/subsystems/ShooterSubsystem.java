// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.motorcontrol.StormSpark;


import edu.wpi.first.wpilibj.DigitalInput;

public class ShooterSubsystem extends SubsystemBase {

    private final DigitalInput sensor = new DigitalInput(0);

    private double intakemc2speed = 0;
    private double motorSpeed = 0;
    private double intakemc1Speed = 0;
    private final StormSpark testWheel = new StormSpark(17, CANSparkMaxLowLevel.MotorType.kBrushless, StormSpark.MotorKind.kNeo);
    private final StormSpark intakemc1 = new StormSpark(16, CANSparkMaxLowLevel.MotorType.kBrushless, StormSpark.MotorKind.kNeo);
    private final StormSpark intakemc2 = new StormSpark(18, CANSparkMaxLowLevel.MotorType.kBrushless, StormSpark.MotorKind.kNeo);

  public void setMotorSpeed(double speed){
    motorSpeed = speed;
  }
  public void setIntakeSpeed (double speed) {
    intakemc1Speed = -speed;
    intakemc2speed = speed;

  }
  public boolean getSensor () {
    return sensor.get();
  }

  public double getMotorSpeed () {
    return testWheel.get();
  }

  public void setintakemc2 (double speed) {
    intakemc2speed = speed;
  }

  @Override
  public void periodic() {
    intakemc1.set(intakemc1Speed);
    intakemc2.set(intakemc2speed);
    testWheel.set(motorSpeed);
  }
}
