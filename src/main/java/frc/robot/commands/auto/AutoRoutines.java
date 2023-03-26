package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.pathFollowing.ArmToTranslation;
import frc.robot.commands.arm.pathFollowing.StowArm;
import frc.robot.commands.auto.autoManeuvers.ArmToNode;
import frc.robot.commands.drive.BalancePitchCommand;
import frc.robot.commands.drive.pathFollowing.PathFollowingCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.FieldConstants.Grids.ScoringNode;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.NavX;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DrivetrainBase;

import java.util.ArrayList;
import java.util.HashMap;

import static frc.robot.constants.Constants.ArmConstants.*;

public final class AutoRoutines {
    public static ArrayList<AutoRoutine> autoRoutines = new ArrayList<>();
    public static HashMap<String, Command> markerCommands = new HashMap<>();


    private static final Pair<PathPlannerTrajectory, PathPlannerTrajectory> topConeTaxi =
            new Pair<>(
                    PathPlanner.loadPath("blue top cone taxi", new PathConstraints(1.0, 1.5)),
                    PathPlanner.loadPath("red top cone taxi", new PathConstraints(1.0, 1.5)));
    private static final Pair<PathPlannerTrajectory, PathPlannerTrajectory> middleConeBalance =
            new Pair<>(
                    PathPlanner.loadPath("blue middle cone balance", new PathConstraints(1.5, 2)),
                    PathPlanner.loadPath("red middle cone balance", new PathConstraints(1.5, 2)));

//    private static PathPlannerTrajectory bumpConeBalance = PathPlanner.loadPath(
//            "bump cone balance", new PathConstraints(1.5, 1.5));
//    private static PathPlannerTrajectory topConeBalance = PathPlanner.loadPath(
//            "top cone balance", new PathConstraints(1.5, 2));
//    private static PathPlannerTrajectory bumpConePick = PathPlanner.loadPath(
//            "bump cone pick", new PathConstraints(1.5, 1.5));
//    private static PathPlannerTrajectory topConePick = PathPlanner.loadPath(
//            "top cone pick ", new PathConstraints(1.5, 2));

    private static BalancePitchCommand balanceCommand;

    private static DrivetrainBase drivetrain;
    private static NavX navX;
    private static Arm arm;
    private static Compression compression;

    public static void initAutoRoutines(DrivetrainBase drivetrain, NavX navX, Arm arm, Compression compression) {
        AutoRoutines.drivetrain = drivetrain;
        AutoRoutines.navX = navX;
        AutoRoutines.arm = arm;
        AutoRoutines.compression = compression;

        markerCommands.put("stow", new ArmToTranslation(arm, stowPosition, 2, 2));
        markerCommands.put("pick ground", new ArmToTranslation(arm, pickGround, 2, 2));
        markerCommands.put("open", new InstantCommand(compression::release));
        markerCommands.put("close", new InstantCommand(compression::grabCubeOrCone));
        balanceCommand = new BalancePitchCommand(drivetrain, navX::getPitch);

        // init auto commands
//        initAutoCommand(topConePick, getGrid()[8][0], true, "TOP HIGH CONE TAXI");
//        initAutoCommand(topConePick, getGrid()[8][1], true, "TOP MID CONE TAXI");
//
//        initAutoCommand(middleConeBalance, getGrid()[3][0], true, "MIDDLE HIGH CONE BALANCE");
//        initAutoCommand(middleConeBalance, getGrid()[3][1], true, "MIDDLE MID CONE BALANCE");
//
//        initAutoCommand(bumpConePick, getGrid()[0][0], true, "BUMP HIGH CONE TAXI");
//        initAutoCommand(bumpConePick, getGrid()[0][2], true, "BUMP MID CONE TAXI");

//        initAutoCommand(
//                PathPlanner.loadPath("red middle cone balance", new PathConstraints(1.5, 2)),
//                        getGrid()[8][1], false, "ONLY FLUXING AUTO");

        makeAuto(topConeTaxi, 8, 0, "TOP HIGH CONE TAXI");
        makeAuto(topConeTaxi, 8, 1, "TOP MID CONE TAXI");
        makeAuto(topConeTaxi, 8, 2, "TOP HYBRID CONE TAXI");

//        makeAuto(middleConeBalance, 3, 0, "MIDDLE HIGH CONE BALANCE");
//        makeAuto(middleConeBalance, 3, 1, "MIDDLE MID CONE BALANCE");
//        makeAuto(middleConeBalance, 3, 2, "MIDDLE HYBRID CONE BALANCE");

//        makeAuto(bumpConeTaxi, 8, 0, "BUMP HIGH CONE TAXI");
//        makeAuto(bumpConeTaxi, 8, 1, "BUMP MID CONE TAXI");
//        makeAuto(bumpConeTaxi, 8, 2, "BUMP HYBRID CONE TAXI");
    }

    private static void makeAuto(Pair<PathPlannerTrajectory, PathPlannerTrajectory> paths,
                                 int col, int row, String name) {
        var blueNode = FieldConstants.Grids.blueAllianceGrid[col][row];
//        System.out.println(blueNode);
        SequentialCommandGroup blueCommand = new SequentialCommandGroup(
                _getInitialPlaceCommand(arm, compression, blueNode),
                new WaitCommand(0.1),
                getPathFollow(paths.getFirst(), drivetrain));
//        if (shouldBalance)
//            command = command.andThen(balanceCommand);
        new AutoRoutine(blueCommand, blueNode.scoringPosition, "BLUE: " + name);

        var redNode = FieldConstants.Grids.redAllianceGrid[col][row];
        SequentialCommandGroup redCommand = new SequentialCommandGroup(
                _getInitialPlaceCommand(arm, compression, redNode),
                new WaitCommand(0.1),
                getPathFollow(paths.getSecond(), drivetrain));
        new AutoRoutine(redCommand, redNode.scoringPosition, "RED: " + name);
    }

    public static FollowPathWithEvents getPathFollow(PathPlannerTrajectory path, DrivetrainBase drivetrain) {
        return new FollowPathWithEvents(
                new PathFollowingCommand(drivetrain, path, true),
                path.getMarkers(),
                markerCommands
        );
    }

    private static Command _getPlaceCommand(Arm arm, Compression compression, ScoringNode node) {
        return new SequentialCommandGroup(
                new ArmToNode(arm, () -> node),
                new WaitCommand(0.2),
                new InstantCommand(compression::release)
        );
    }

    private static Command _getInitialPlaceCommand(Arm arm, Compression compression, ScoringNode node) {
        return new SequentialCommandGroup(
                new InstantCommand(compression::grabCubeOrCone),
                new WaitCommand(0.2),
                _getPlaceCommand(arm, compression, node),
                new WaitCommand(0.2),
                new StowArm(arm)
        );
    }
}
