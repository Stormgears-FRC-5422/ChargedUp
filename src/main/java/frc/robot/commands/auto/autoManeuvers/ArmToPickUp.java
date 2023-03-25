package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.arm.ArmTrajectoryToPose;
import frc.robot.commands.arm.pathFollowing.ArmToTranslation;
import frc.robot.constants.Constants;
import frc.robot.subsystems.NeoPixel;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.stormnet.StormNet;

public class ArmToPickUp extends CommandBase {

  Arm arm;

  StormNet stormNet;

  boolean commandCondition;

  ArmTrajectoryToPose armTrajectoryToPose;


  public ArmToPickUp(Arm arm, StormNet stormNet, ArmTrajectoryToPose armTrajectoryToPose){
    this.arm = arm;
    this.stormNet = stormNet;
    this.armTrajectoryToPose = armTrajectoryToPose;
  }

  @Override
  public void initialize() {
    System.out.println("ArmToPickUp command running");
  }

  @Override
  public void execute() {
    new ArmToTranslation(arm, Constants.ArmConstants.pickDoubleSubstation, 2, 2);
    commandCondition = true;
//    double distance = stormNet.getLidarDistance();
//
//    if (distance >= 0.13 && distance <= 0.257) {
//      commandCondition = true;
//    }
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return commandCondition;
  }
}
