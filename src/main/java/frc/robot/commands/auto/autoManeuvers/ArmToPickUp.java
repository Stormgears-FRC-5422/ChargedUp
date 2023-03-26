package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotState;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.stormnet.StormNet;

import static frc.robot.constants.Constants.kXYArmManualSpeed;

public class ArmToPickUp extends CommandBase {

  Arm arm;

  StormNet stormNet;
  Pose2d gripperPose;
  double yPos;

  private boolean commandCondition;
  private static double tolerance = 0.05;

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
    Translation2d gripper = arm.getGripperPose().getTranslation();
    double distance = stormNet.getLidarDistance();
    double dx = 0;
    double dy = 0;
    var currentRange = RobotState.getInstance().getLidarRange();

//    if (distance <= currentRange.getMin()) {
//      dx = -kXYArmManualSpeed;
//    } else if (distance >= currentRange.getMax() && distance < 0.5) {
//      dx = kXYArmManualSpeed;
//    }
    if (distance <= 0.5)
      dx = Math.signum(distance - currentRange.getCenter()) * kXYArmManualSpeed;

    double y = gripper.getY();
    if (y > yPos + 0.015) {
      dy = -kXYArmManualSpeed;
    } else if (y < yPos - 0.015) {
      dy = kXYArmManualSpeed;
    }

//    System.out.println("Lidar distance: " + distance + " dx: " + dx + " dy: " + dy + " y: " + y + " yPos: " + yPos);
    arm.xyMoveArm(new ChassisSpeeds(dx, dy, 0));
    commandCondition = gripper.getX() >= currentRange.getCenter() - tolerance &&
            gripper.getX() <= currentRange.getCenter() + tolerance;
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
