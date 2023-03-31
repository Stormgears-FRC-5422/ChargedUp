package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.pathFollowing.StowArm;
import frc.robot.commands.auto.autoManeuvers.ArmToNode;
import frc.robot.commands.auto.autoManeuvers.DropPieceSequence;
import frc.robot.commands.drive.pathFollowing.PathFollowingCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShuffleboardConstants;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DrivetrainBase;

public class AutoSelector {
    private SendableChooser<Integer> nodeColumnChooser = new SendableChooser<>();
    private SendableChooser<Integer> nodeHeightChooser = new SendableChooser<>();

    public AutoSelector() {
        for (int i = 0; i < 9; i++) {
            if (i == 0) {
                nodeColumnChooser.setDefaultOption("Column " + i, i);
                continue;
            }
            nodeColumnChooser.addOption("Column " + i, i);
        }

        String[] heights = {"HIGH", "MID", "HYBRID"};
        for (int i = 0; i < 3; i++) {
            if (i == 0) {
                nodeHeightChooser.setDefaultOption(heights[i], i);
                continue;
            }
            nodeHeightChooser.addOption(heights[i], i);
        }

        ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Column?", nodeColumnChooser);
        ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Height?", nodeHeightChooser);
    }

    public AutoRoutine buildAuto(DrivetrainBase drivetrain, Arm arm, Compression compression) {
        return buildAuto(drivetrain, arm, compression,
                nodeColumnChooser.getSelected(), nodeHeightChooser.getSelected(), false);
    }

    private AutoRoutine buildAuto(DrivetrainBase drivetrain, Arm arm, Compression compression,
                              int col, int row, boolean balance) {
        PathPlannerTrajectory path = PathPlanner
                .loadPath(col + "", new PathConstraints(2, 1.5));
        FieldConstants.Grids.ScoringNode node = FieldConstants.Grids.getNodeAbsolute(col, row);

        Command command = new SequentialCommandGroup(
                getFirstPlaceCommand(arm, compression, node),
                getPathFollowCommand(drivetrain, path)
        );
        return new AutoRoutine(command, node.scoringPosition);
    }

    private Command getFirstPlaceCommand(Arm arm, Compression compression, FieldConstants.Grids.ScoringNode node) {
        return new SequentialCommandGroup(
                new ArmToNode(arm, () -> node),
                compression.getReleaseCommand(),
                new StowArm(arm)
        );
    }

    private Command getPathFollowCommand(DrivetrainBase drivetrain, PathPlannerTrajectory path) {
        return new PathFollowingCommand(drivetrain, path, true);
    }
}
