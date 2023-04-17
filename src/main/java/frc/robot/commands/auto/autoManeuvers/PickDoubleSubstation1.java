package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotState;
import frc.robot.commands.arm.pathFollowing.ArmToTranslation;
import frc.robot.commands.arm.pathFollowing.StowArm;
import frc.robot.constants.Constants;
import frc.robot.constants.ShuffleboardConstants;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.stormnet.StormNet;

import static frc.robot.constants.Constants.ArmConstants.pickDoubleSubstationCone;
import static frc.robot.constants.Constants.ArmConstants.pickDoubleSubstationCube;

public class PickDoubleSubstation1 extends SequentialCommandGroup {

     public PickDoubleSubstation1(Arm arm, Compression compression, StormNet stormNet) {
         addCommands(
                 compression.getReleaseCommand(),
                 new ArmToTranslation(arm, Constants.ArmConstants::getPickupLocation, 4, 4),
                 new ArmToPickUp(arm, stormNet),
                 compression.getGrabCommand(),
                 new PrintCommand("Opened Gripper!"),
                 new StowArm(arm), new InstantCommand(() -> compression.setOnOffSolenoid(compression.isGripperButtonPosition())
         ));
     }
}
