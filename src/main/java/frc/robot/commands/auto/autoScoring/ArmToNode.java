package frc.robot.commands.auto.autoScoring;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.commands.arm.pathFollowing.ArmPathFollowingCommand;
import frc.robot.subsystems.arm.Arm;

import java.util.function.Supplier;

import static frc.robot.constants.FieldConstants.Grids.ScoringNode;
import static frc.robot.commands.drive.pathFollowing.Paths.calcHeading;

public class ArmToNode extends ArmPathFollowingCommand {

    // TODO: find x and y positions for each node {high, mid, hybrid}
    private static final double[] goalXPositions = {1.38, 0.984, 0.5};
    private static final double[] coneYPositions = {1.25, 0.95, 0.22};
    private static final double[] cubeYPositions = {0.96, 0.67, 0.22};

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
        double goalY = isCube?
                cubeYPositions[node.row] : coneYPositions[node.row];
        double goalX = goalXPositions[node.row];

        Translation2d current = arm.getGripperPose().getTranslation();
        final boolean isRight = current.getX() >= goalX;
        Translation2d intermediate = isRight?
                new Translation2d(goalX, current.getY()) : new Translation2d(current.getX(), goalY);
        Translation2d goal = new Translation2d(goalX, goalY);

        // control radius of arc using divisor
        final double startControlLength = current.getDistance(intermediate) / 0.5;
        Rotation2d startHeading = new Rotation2d(calcHeading(current, intermediate));
        PathPoint start = new PathPoint(current, startHeading, new Rotation2d())
                .withNextControlLength(startControlLength);

//        Rotation2d endHeading = new Rotation2d(calcHeading(intermediate, goal));
        final double endControlLength = isCube? Units.inchesToMeters(7.0) : Units.inchesToMeters(12.0);
        PathPoint end = new PathPoint(goal, new Rotation2d(Math.PI), new Rotation2d())
                .withPrevControlLength(endControlLength);

        var path = PathPlanner.generatePath(
                new PathConstraints(2.0, 2.0),
                start, end);
        addPath(path);
        super.initialize();
    }
}
