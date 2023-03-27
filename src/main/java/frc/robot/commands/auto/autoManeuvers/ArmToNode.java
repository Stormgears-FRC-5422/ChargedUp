package frc.robot.commands.auto.autoManeuvers;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
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


    private static final double CONE_OFFSET = Units.inchesToMeters(7.0);
    private static final double CUBE_OFFSET = Units.inchesToMeters(12.0);

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

        double goalX = Math.abs(node.scoringPosition.getX() - nodeTranslation.getX()) + Units.inchesToMeters(4.0);
        double goalZ = nodeTranslation.getZ() + ((isCube)? CUBE_OFFSET : CONE_OFFSET);

        Translation3d globalCoordinate = new Translation3d(
                goalX, 0, goalZ
        );
        Translation2d goal = Arm.fromGlobalTranslation(globalCoordinate);
        double goalY = goal.getY();

        Translation2d current = arm.getGripperPose().getTranslation();
        final boolean isRight = current.getX() >= goalX;
        Translation2d intermediate = isRight?
                new Translation2d(goalX, current.getY()) : new Translation2d(current.getX(), goalY);

        // control radius of arc using divisor
        final double startControlLength = current.getDistance(intermediate) * 1.5;
//        Rotation2d startHeading = new Rotation2d(calcHeading(current, intermediate));
        PathPoint start = new PathPoint(current, new Rotation2d(Math.PI / 2.0), new Rotation2d())
                .withNextControlLength(startControlLength);

//        Rotation2d endHeading = new Rotation2d(calcHeading(intermediate, goal));
        final double endControlLength = (isCube? CUBE_OFFSET : CONE_OFFSET) * 4.0;
        PathPoint end = new PathPoint(goal, new Rotation2d(-Math.PI / 2.0), new Rotation2d())
                .withPrevControlLength(endControlLength);

        var path = PathPlanner.generatePath(
                new PathConstraints(2.0, 2.0),
                start, end);
        addPath(path);
        super.initialize();
    }
}
