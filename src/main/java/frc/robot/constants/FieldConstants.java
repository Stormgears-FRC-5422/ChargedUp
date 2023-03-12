package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Objects;

import static frc.robot.constants.Constants.*;

public final class FieldConstants {
    public final static double FIELD_LENGTH = Units.feetToMeters(54) + Units.inchesToMeters(3.25);
    public final static double FIELD_WIDTH = Units.feetToMeters(26) + Units.inchesToMeters(3.5);

    public final static double HALF_FIELDLENGTH = FIELD_LENGTH / 2.0;

    public static final AprilTagFieldLayout APRILTAG_FIELD_LAYOUT;

    static {
        try {
            APRILTAG_FIELD_LAYOUT = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch (IOException e) {
            System.out.println("Could not load april tag field layout! error msg: \n " + e);
            throw new RuntimeException(e);
        }
    }

    public static Pose3d getTagPose(int id) {
        var tagPose = APRILTAG_FIELD_LAYOUT.getTagPose(id);
        if (tagPose.isPresent()) {
            return tagPose.get();
        } else {
            System.out.println("Couldn't get april tag pose!");
            return new Pose3d();
        }
    }

    /** Grid arrays [0][0] are the highest and the farthest from substation */
    public final static class Grids {
        public static final ScoringNode[][] blueAllianceGrid = new ScoringNode[9][3];
        public static final ScoringNode[][] redAllianceGrid = new ScoringNode[9][3];

        private static final double halfRobotLengthWithBumper = (ROBOT_LENGTH / 2.0) + BUMPER_THICKNESS;
        private static final double distBetweenNodes = Units.inchesToMeters(22.0);
        private static final double distToFirstNodeY = Units.inchesToMeters(20.0);
        //heights of cone nodes cube nodes don't really matter maybe we can make them a bit lower than the cone nodes
        //hybrid node (last in array) is 10 inches of ground just because
        private static final ScoringNode.NodeHeight[] nodeHeights = {
            ScoringNode.NodeHeight.HIGH, ScoringNode.NodeHeight.MIDDLE, ScoringNode.NodeHeight.HYBRID
        };
        //last one is kind of a guess (hybrid node)
        private static final double[] nodeXs = {Units.inchesToMeters(14.32), Units.inchesToMeters(31.35), Units.inchesToMeters(47.0)};

        // Calaculate regions for every 3 by 3 of nodes
        private static final double distBetweenChargingStationGrid = Units.inchesToMeters(59.0);
        private static final double regionMaxX = distBetweenChargingStationGrid - halfRobotLengthWithBumper;
        private static final double regionMinX = Units.inchesToMeters(54.05);
        private static final double firstRegionWidth = Units.inchesToMeters(75.185);
        private static final double secondRegionWidth = Units.inchesToMeters(65.55);
        private static final double thirdRegionWidth = Units.inchesToMeters(75.345);
        private static final double[] regionWidths = new double[] {firstRegionWidth, secondRegionWidth, thirdRegionWidth};

        public static void initGridNodes() {
            double scoringX = Units.inchesToMeters(54.05) + halfRobotLengthWithBumper;
            for (int wpiY = 0; wpiY < 9; wpiY++) {
                ScoringNode.NodeType type = (wpiY == 1 || wpiY == 4 || wpiY == 7)? ScoringNode.NodeType.CUBE : ScoringNode.NodeType.CONE;
                double yTranslation = distToFirstNodeY + (wpiY * distBetweenNodes);

                // calculating regions for blue nodes
                int currentRegion = wpiY / 3;
                double regionWidth = regionWidths[currentRegion];
                double regionMinY = 0.0;
                for (int i = 0; i <= currentRegion; i++)
                    regionMinY += regionWidths[i];
                double regionMaxY = regionMinY + regionWidth;
                RectangleRegion region = new RectangleRegion(regionMaxX, regionMaxY, regionMinX, regionMinY);

                for (int wpiX = 0; wpiX < 3; wpiX++) {
                    type = (wpiX == 2)? ScoringNode.NodeType.HYBRID : type;
                    var height = nodeHeights[wpiX];
                    double xTranslation = nodeXs[wpiX];
                    double zTranslation = height.getHeight();
                    var translation = new Translation3d(xTranslation, yTranslation, zTranslation);
                    //TODO: scoring positions may change based on height of node
                    // e.x. if its hybrid we may not want to drive all the way up (unless we do?)
                    var scoringPosition = new Pose2d(scoringX, yTranslation, Rotation2d.fromDegrees(180));

                    var node = new ScoringNode(type, height, Alliance.Blue, translation,
                            scoringPosition, region, wpiY, wpiX);
                    blueAllianceGrid[wpiY][wpiX] = node;
                    var transformedNode = ScoringNode.transformBlueToRed(node);
                    redAllianceGrid[wpiY][wpiX] = transformedNode;
//                    System.out.println(blueAllianceGrid[wpiY][wpiX]);
                }
            }
        }

        public static ScoringNode[][] getCurrentGrid() {
            return (DriverStation.getAlliance() == Alliance.Red)? redAllianceGrid : blueAllianceGrid;
        }

        public static class ScoringNode {
            public final NodeType type;
            public final NodeHeight height;
            public final Alliance alliance;
            public final Translation3d translation;
            public final Pose2d scoringPosition;
            public final RectangleRegion gridRegion;
            public final int col, row;

            public ScoringNode(NodeType type, NodeHeight height, Alliance alliance,
                               Translation3d translation, Pose2d scoringPosition, RectangleRegion gridRegion,
                               int col, int row) {
                this.type = type;
                this.height = height;
                this.alliance = alliance;
                this.translation = translation;
                this.scoringPosition = scoringPosition;
                this.gridRegion = gridRegion;
                this.col = col;
                this.row = row;
            }

            public static ScoringNode transformBlueToRed(ScoringNode node) {
                if (node.alliance == Alliance.Red) return node;
                //transform Y so that arrays are correct positioning since nodes are mirrored
                Translation3d nodeTranslation = node.translation;
                double transformedX = mirrorXPosition(nodeTranslation.getX());

                double blueMiddleNodeY = Units.inchesToMeters(108);
                double transformedY = (blueMiddleNodeY - nodeTranslation.getY()) + blueMiddleNodeY;
                Translation3d transformedTranslation = new Translation3d(transformedX,
                        transformedY, nodeTranslation.getZ());

                Pose2d transformedScoringPosition = mirrorPose(node.scoringPosition);

                var region = node.gridRegion;
                RectangleRegion transformedRegion = new RectangleRegion(
                        mirrorTranslation(region.topLeft),
                        mirrorTranslation(region.bottomRight)
                );

                return new ScoringNode(node.type, node.height, Alliance.Red,
                        transformedTranslation, transformedScoringPosition, transformedRegion,
                        node.col, node.row);
            }

            @Override
            public String toString() {
                return String.format(
                        "ScoringNode(%1$s, %2$s, %3$s," +
                        " node(x: %4$s y: %5$s z: %6$s)" +
                        " scoring pose(x: %7$s y: %8$s heading: %9$s)",
                        type, height, alliance,
                        translation.getX(), translation.getY(), translation.getZ(),
                        scoringPosition.getX(), scoringPosition.getY(), scoringPosition.getRotation().getDegrees());
            }

            @Override
            public boolean equals(Object o) {
                if (this == o) return true;
                if (o == null || getClass() != o.getClass()) return false;
                ScoringNode that = (ScoringNode) o;
                return type == that.type &&
                        height == that.height &&
                        alliance == that.alliance &&
                        translation.equals(that.translation) &&
                        scoringPosition.equals(that.scoringPosition);
            }

            @Override
            public int hashCode() {
                return Objects.hash(type, height, alliance, translation, scoringPosition);
            }

            public enum NodeType {
                CUBE, CONE, HYBRID
            }

            public enum NodeHeight {
                HIGH(Units.inchesToMeters(46.0)),
                MIDDLE(Units.inchesToMeters(34.0)),
                HYBRID(Units.inchesToMeters(0.0));

                private final double height;
                NodeHeight(double height) {
                    this.height = height;
                }
                public double getHeight() {
                    return height;
                }
            }
        }
    }

    public static double mirrorXPosition(double xToBeMirrored) {
        return (HALF_FIELDLENGTH - xToBeMirrored) + HALF_FIELDLENGTH;
    }

    public static Translation2d mirrorTranslation(Translation2d translation) {
        return new Translation2d(mirrorXPosition(translation.getX()), translation.getY());
    }

    public static Pose2d mirrorPose(Pose2d pose) {
        double xMirrored = mirrorXPosition(pose.getX());
        var rotationMirrored = pose.getRotation().times(-1.0);
        return new Pose2d(xMirrored, pose.getY(), rotationMirrored);
    }

    public interface Region {
        /** all field coords */
        boolean inRegion(Pose2d pose);
        /** all field coords */
        default boolean inRegion(Translation2d translation) {
            return inRegion(new Pose2d(translation, new Rotation2d()));
        }
        /** all field coords */
        default boolean inRegion(double x, double y) {
            return inRegion(new Translation2d(x, y));
        }
    }

    public static class RectangleRegion implements Region {
        public final Translation2d topLeft, bottomRight;

        /** field coords */
        public RectangleRegion(Translation2d topLeft, Translation2d bottomRight) {
            this.topLeft = topLeft;
            this.bottomRight = bottomRight;
        }

        /** field coords max is top left and min is bottom right*/
        public RectangleRegion(double top, double left, double bottom, double right) {
            this.topLeft = new Translation2d(top, left);
            this.bottomRight = new Translation2d(bottom, right);
        }

        @Override
        public boolean inRegion(Pose2d position) {
            return (position.getX() <= topLeft.getY() && position.getX() >= bottomRight.getX()) &&
                    (position.getY() <= topLeft.getY() && position.getY() >= bottomRight.getY());
        }
    }

    public static class PolyRectangleRegion implements Region {

        public ArrayList<RectangleRegion> rectangleRegions;

        public PolyRectangleRegion(RectangleRegion... rectangleRegions) {
            this.rectangleRegions.addAll(Arrays.asList(rectangleRegions));
        }

        @Override
        public boolean inRegion(Pose2d pose) {
            return false;
        }
    }
}
