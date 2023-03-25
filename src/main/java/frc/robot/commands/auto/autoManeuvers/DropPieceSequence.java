package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DrivetrainBase;

public class DropPieceSequence extends SequentialCommandGroup {
  public DropPieceSequence(DrivetrainBase drivetrain, Arm arm, Compression compression, NodeSelector nodeSelector){
    addCommands(
//            new DriveToNode(drivetrain, nodeSelector::getSelectedNode ),
            new ArmToNode(arm, nodeSelector::getSelectedNode),
            new InstantCommand(compression::release),
            new PrintCommand("Finished with DropPiece command"));
    };
  }
