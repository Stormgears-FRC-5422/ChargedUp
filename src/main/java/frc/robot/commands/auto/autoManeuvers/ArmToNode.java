package frc.robot.commands.auto.autoManeuvers;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.arm.pathFollowing.ArmPathFollowingCommand;
import frc.robot.constants.Constants;
import frc.robot.subsystems.arm.Arm;

import java.util.function.Supplier;

import static frc.robot.constants.FieldConstants.Grids.ScoringNode;
import static frc.robot.commands.drive.pathFollowing.Paths.calcHeading;

public class ArmToNode extends ArmPathFollowingCommand {

    // TODO: find x and y positions for each node {high, mid, hybrid}
    private static final double[] goalXPositions = {1.15, 0.984, 0.5};


    private static final double CONE_OFFSET = Units.inchesToMeters(10.0);
    private static final double CUBE_OFFSET = Units.inchesToMeters(6.0);

    private final Arm arm;
    private final Supplier<ScoringNode> nodeSupplier;

    public ArmToNode(Arm arm, Supplier<ScoringNode> nodeSupplier) {
        super(arm);
        this.arm = arm;
        this.nodeSupplier = nodeSupplier;
    }

    @Override
    public void initialize() {
        ScoringNode node = nodeSupplier.get();
        System.out.println("Moving arm to score on " + node.height + " of type " + node.type);
        final boolean isCube = node.type == ScoringNode.NodeType.CUBE;
//        double goalY = isCube?
//                cubeYPositions[node.row] : coneYPositions[node.row];
//        double goalX = goalXPositions[node.row];

        Translation3d nodeTranslation = node.translation;

        // added 4.0 inches due to testing
        double goalX = Math.abs(node.scoringPosition.getX() - nodeTranslation.getX());
        if (node.height == ScoringNode.NodeHeight.HIGH)
            goalX += Units.inchesToMeters(4.0);
        else if (node.height == ScoringNode.NodeHeight.MIDDLE)
            goalX += Units.inchesToMeters(2.5);
        double goalZ = nodeTranslation.getZ() + ((isCube)? CUBE_OFFSET : CONE_OFFSET);

        Translation3d globalCoordinate = new Translation3d(
                goalX, 0, goalZ
        );
        Translation2d goal = Arm.fromGlobalTranslation(globalCoordinate);
        double goalY = goal.getY();

        Translation2d current = arm.getGripperPose().getTranslation();
        double deltaY = MathUtil.clamp(goalY - current.getY(), 0.01, 2.0);
        boolean isHybrid = (node.height == ScoringNode.NodeHeight.HYBRID);

        Rotation2d startHeading = isHybrid?
                new Rotation2d(Math.PI / 6) : new Rotation2d(Math.PI / 2);
        PathPoint start = new PathPoint(current, startHeading, new Rotation2d())
                .withNextControlLength(isHybrid? 0.2 : deltaY * 1.2);

        PathPoint end = new PathPoint(goal, new Rotation2d(-Math.PI / 2.0), new Rotation2d())
                .withPrevControlLength(deltaY);

        var path = PathPlanner.generatePath(
                new PathConstraints(4.0, 2.0),
                start, end);
        addPath(path);
        super.initialize();
    }
}
