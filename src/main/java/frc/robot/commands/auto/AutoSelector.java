package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.arm.pathFollowing.StowArm;
import frc.robot.commands.auto.autoManeuvers.ArmToNode;
import frc.robot.commands.auto.autoManeuvers.AutoBalance;
import frc.robot.commands.drive.BalancePitchCommand;
import frc.robot.commands.drive.pathFollowing.PathFollowingCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShuffleboardConstants;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.ArrayList;
import java.util.List;

public class AutoSelector {
    private SendableChooser<Integer> nodeColumnChooser = new SendableChooser<>();
    private SendableChooser<Integer> nodeHeightChooser = new SendableChooser<>();
    private SendableChooser<Boolean> taxiChooser = new SendableChooser<>();
    private SendableChooser<Boolean> balanceChooser = new SendableChooser<>();

    private Intake intake = new Intake();

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

        balanceChooser.setDefaultOption("No", false);
        balanceChooser.setDefaultOption("Yes", true);

        taxiChooser.setDefaultOption("No", false);
        taxiChooser.setDefaultOption("Yes", true);

        ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Column?", nodeColumnChooser).withPosition(0, 0);
        ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Height?", nodeHeightChooser).withPosition(0, 1);
        ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Taxi?", taxiChooser).withPosition(0, 2);
        ShuffleboardConstants.getInstance().autoSelectionLayout
                .add("Balance?", balanceChooser).withPosition(0, 3);
    }

    public AutoRoutine buildAuto(DrivetrainBase drivetrain, Arm arm, Intake intake, NavX navX) {
        return buildAuto(drivetrain, arm, intake, navX,
                nodeColumnChooser.getSelected(), nodeHeightChooser.getSelected(),
                taxiChooser.getSelected(), balanceChooser.getSelected());
    }

    private AutoRoutine buildAuto(DrivetrainBase drivetrain, Arm arm, Intake intake, NavX navX,
                              int col, int row, boolean taxi, boolean balance) {
        // get path and first node
        List<PathPlannerTrajectory> path = PathPlanner
                .loadPathGroup(col + "",
                        new PathConstraints(2, 2),
                        new PathConstraints(3, 4));

        FieldConstants.Grids.ScoringNode node = FieldConstants.Grids.getNodeAbsolute(col, row);

        List<CommandBase> commands = new ArrayList<>();
        commands.add(getFirstPlaceCommand(arm, intake, node));
        if (taxi) {
            commands.add(getPathFollowCommand(drivetrain, path.get(0)));
            if (balance) {
                commands.add(getPathFollowCommand(drivetrain, path.get(1)));
                commands.add(new AutoBalance(drivetrain, navX));
            }
        }

        return new AutoRoutine(Commands.sequence(commands.toArray(CommandBase[]::new)), node.scoringPosition);
    }

    private CommandBase getFirstPlaceCommand(Arm arm, Intake intake, FieldConstants.Grids.ScoringNode node) {
        return new SequentialCommandGroup(
                new ArmToNode(arm, () -> node),
                new IntakeCommand(intake, false),
                new StowArm(arm)
        );
    }

    private CommandBase getPathFollowCommand(DrivetrainBase drivetrain, PathPlannerTrajectory path) {
        return new PathFollowingCommand(drivetrain, path, true);
    }
}
