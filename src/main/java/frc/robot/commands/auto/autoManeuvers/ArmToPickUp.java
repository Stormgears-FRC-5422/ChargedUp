package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.constants.Constants;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.stormnet.StormNet;

import static frc.robot.constants.Constants.kXYArmManualSpeed;

public class ArmToPickUp extends CommandBase {
  Arm arm;
  StormNet stormNet;

  Pose2d gripperPose;
  double yTarget;
  double dx;
  double dy;
  boolean closeToTarget;

  public ArmToPickUp(Arm arm, StormNet stormNet){
    this.arm = arm;
    this.stormNet = stormNet;
    this.addRequirements(arm);
  }

  @Override
  public void initialize() {
    System.out.println("ArmToPickUp command running");

    gripperPose = arm.getGripperPose();
    yTarget = Constants.ArmConstants.pickDoubleSubstation.getY();
    dx = 0;
    dy = 0;
  }

  @Override
  public void execute() {
    gripperPose = arm.getGripperPose();
    double distance = stormNet.getLidarDistance();
    var currentRange = RobotState.getInstance().getLidarRange();

    closeToTarget = false;
    if (distance <= currentRange.getMin()) { // Too close to target
      dx = -kXYArmManualSpeed;
    } else if (distance >= currentRange.getMax() && distance < 0.5) {  // Too far from target
      dx = kXYArmManualSpeed;
    } else if (distance >= 0.5) { // Way to far from target
      dx = 0;
    } else { // close to target
      dx = 0;
      closeToTarget = true;
    }

    double y = gripperPose.getY();
    if (y < yTarget - 0.015 || dy > 0 && y < yTarget) {
      dy = kXYArmManualSpeed;
    } else if (y > yTarget + 0.015 || dy < 0 && y > yTarget) {
      dy = -kXYArmManualSpeed;
    } else {
      dy = 0;
    }

//    System.out.println("Lidar distance: " + distance + " dx: " + dx + " dy: " + dy + " y: " + y + " yPos: " + yPos);
    arm.xyMoveArm(new ChassisSpeeds(dx, dy, 0));
  }

  @Override
  public boolean isFinished() {
    return (dx == 0 && dy == 0 && closeToTarget);
  }

  @Override
  public void end(boolean interrupted) {
    arm.stopArm();
  }

}
