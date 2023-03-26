package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.arm.pathFollowing.ArmToTranslation;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.stormnet.StormNet;
import frc.utils.joysticks.DriveJoystick;


public class PickFromSubstationSequence extends SequentialCommandGroup {

  public PickFromSubstationSequence(DrivetrainBase drivetrain, Arm arm, Compression compression,
                                    FieldConstants.Side side, StormNet stormNet, DriveJoystick joystick) {
    if (Constants.Toggles.useStormNet) {
      addCommands(
              new ParallelCommandGroup(
                      compression.getReleaseCommand(),
                      new ArmToTranslation(arm, Constants.ArmConstants.pickDoubleSubstationCone, 3, 3)),
              new ArmToPickUp(arm, stormNet),
              new InstantCommand(compression::grabCubeOrCone),
              new ArmToTranslation(arm, Constants.ArmConstants.pickDoubleSubstationCone, 3, 3),
              new ArmToTranslation(arm, Constants.ArmConstants.stowPosition, 3, 3)
      );
////      addCommands(
////              new ParallelCommandGroup(
////                      new AlignToDoubleSubstation(drivetrain, joystick, side),
////                      new SequentialCommandGroup(
////                              new InstantCommand(compression::release),
////                              new ArmToTranslation(arm, pickDoubleSubstation, 3, 3),
////                              new ArmToPickUp(arm, stormNet),
////                              new PrintCommand("Arm to PICK up ended!"),
////                              new InstantCommand(compression::grabCubeOrCone),
////                              new StowArm(arm)
////                      )
//              )
//      );
      addRequirements(drivetrain, arm, compression);
    }
    else {
//      addCommands(
//              new ParallelCommandGroup(
//                      new AlignToDoubleSubstation(drivetrain, joystickXSupplier, joystickZSupplier, side),
//                      new InstantCommand(compression::release),
//                      new ArmToTranslation(arm, Constants.ArmConstants.pickDoubleSubstation, 2, 2)),
//              new ArmToTranslation(arm, Constants.ArmConstants.tempArmPickUpLocation, 2, 2),
//              new InstantCommand(compression::grabCubeOrCone),
//              new ArmToTranslation(arm, Constants.ArmConstants.pickDoubleSubstation, 2, 2),
//              new ArmToTranslation(arm, Constants.ArmConstants.stowPosition, 2, 2)
//      );
      addRequirements(drivetrain, arm, compression);
    }
  }
}
