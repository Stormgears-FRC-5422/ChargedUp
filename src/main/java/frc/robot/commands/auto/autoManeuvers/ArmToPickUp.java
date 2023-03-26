package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.stormnet.StormNet;

import static frc.robot.constants.Constants.kXYArmManualSpeed;

public class ArmToPickUp extends CommandBase {

  Arm arm;

  StormNet stormNet;
  Pose2d gripperPose;
  double yPos;


  private boolean commandCondition;


  public ArmToPickUp(Arm arm, StormNet stormNet){
    this.arm = arm;
    this.stormNet = stormNet;
  }

  @Override
  public void initialize() {
    System.out.println("ArmToPickUp command running");

    gripperPose = arm.getGripperPose();
    yPos = gripperPose.getY();
  }

  @Override
  public void execute() {
    double distance = stormNet.getLidarDistance();
    double dx = 0;
    double dy = 0;

    if (distance <= 0.13){
      dx = -kXYArmManualSpeed;
    } else if (distance >= 0.257 && distance < 0.5) {
      dx = kXYArmManualSpeed;
    }

    double y = arm.getGripperPose().getY();
    if (y > yPos + 0.015) {
      dy = -kXYArmManualSpeed;
    } else if (y < yPos - 0.015) {
      dy = kXYArmManualSpeed;
    }

    System.out.println("Lidar distance: " + distance + " dx: " + dx + " dy: " + dy + " y: " + y + " yPos: " + yPos);
    arm.xyMoveArm(new ChassisSpeeds(dx, dy, 0));


//
//    if (cubeCone.equals(CubeCone.CONE)) {
//      if (distance >= 0.13 && distance <= 0.257) {
//        commandCondition = true;
//      }
//    } else if (cubeCone.equals(CubeCone.CUBE)) {
//      if (distance <= 0.21){
//        commandCondition = true;
//      }
//    }
  }

  @Override
  public boolean isFinished() {
    return commandCondition;
  }

  @Override
  public void end(boolean interrupted) {
    arm.stopArm();
  }

}
