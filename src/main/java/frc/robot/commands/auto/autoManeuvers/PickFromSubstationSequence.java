package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.GripperCommand;
import frc.robot.commands.arm.pathFollowing.ArmToTranslation;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.robot.subsystems.stormnet.StormNet;



public class PickFromSubstationSequence extends SequentialCommandGroup {

  public PickFromSubstationSequence(DrivetrainBase drivetrain, Arm arm, Compression compression,
                                    DriveToDoubleSubstation.Position side, StormNet stormNet) {
    if (Constants.Toggles.useStormNet) {
      addCommands(
              new ParallelCommandGroup(
//                    new DriveToDoubleSubstation(drivetrain, side),
                      new InstantCommand(compression::release),
                      new ArmToTranslation(arm, Constants.ArmConstants.pickDoubleSubstation, 2, 2)),
              new ArmToTranslation(arm, Constants.ArmConstants.tempArmPickUpLocation, 2, 2),
              new GripperCommand(compression, stormNet),
              new ArmToTranslation(arm, Constants.ArmConstants.pickDoubleSubstation, 2, 2),
              new ArmToTranslation(arm, Constants.ArmConstants.stowPosition, 2, 2)
      );
      addRequirements(drivetrain, arm, compression);
    }
  }
}
