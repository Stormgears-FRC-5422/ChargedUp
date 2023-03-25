package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotState;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Objects;

import static frc.robot.constants.Constants.*;

public final class FieldConstants {


    public static void init() {
        Grids.initGridNodes();

    }

    public final static double FIELD_LENGTH = Units.feetToMeters(54) + Units.inchesToMeters(3.25);
    public final static double FIELD_WIDTH = Units.feetToMeters(26) + Units.inchesToMeters(3.5);

    public final static double HALF_FIELD_LENGTH = FIELD_LENGTH / 2.0;

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

    public static Regions.RectangleRegion blueChargingStation = new Regions.RectangleRegion(
            Units.inchesToMeters(152), Units.inchesToMeters(115),
            Units.inchesToMeters(60), Units.inchesToMeters(189));
    public static Regions.RectangleRegion redChargingStation = blueChargingStation.getMirrored();

    public static Regions.RectangleRegion getChargingStation() {
        return (RobotState.getInstance().getCurrentAlliance() == Alliance.Red)?
                redChargingStation : blueChargingStation;
    }

    public final static class Substations {

        public final static double substationHeight = Units.inchesToMeters(39.938);

        private final static double substationAlignDist = Units.feetToMeters(3.0);
        private final static double alignDist = FIELD_LENGTH - ((ROBOT_LENGTH / 2.0 ) + BUMPER_THICKNESS + substationAlignDist);
        private final static double wallToPortalClose = Units.inchesToMeters(34.881);
        private final static double wallToPortalFar = Units.inchesToMeters(64.861);
        private final static double portalToFar = Units.inchesToMeters(24.126);

        private final static double wallToRobotPadding = FIELD_WIDTH - (ROBOT_WIDTH / 2.0);

        private final static Regions.RectangleRegion blueLeftRegion = new Regions.RectangleRegion(
                wallToRobotPadding - BUMPER_THICKNESS, alignDist,
                wallToRobotPadding - wallToPortalClose, alignDist);

        private final static Regions.RectangleRegion blueRightRegion = new Regions.RectangleRegion(
                wallToRobotPadding - wallToPortalFar, alignDist,
                wallToRobotPadding - wallToPortalFar - portalToFar, alignDist
        );

        public final static DoubleSubstation blueDoubleSubstation = new DoubleSubstation(blueLeftRegion, blueRightRegion);
        public final static DoubleSubstation redDoubleSubstation = new DoubleSubstation(blueRightRegion.getMirrored(), blueLeftRegion.getMirrored());

        public static DoubleSubstation getDoubleSubstation() {
            return (RobotState.getInstance().getCurrentAlliance() == Alliance.Red)?
                    redDoubleSubstation : blueDoubleSubstation;
        }

        public static class DoubleSubstation {
            public final Regions.RectangleRegion leftRegion, rightRegion;

            public DoubleSubstation(Regions.RectangleRegion leftRegion, Regions.RectangleRegion rightRegion) {
                this.leftRegion = leftRegion;
                this.rightRegion = rightRegion;
            }

            public Translation2d getLeftTranslation(Translation2d current) {
                return leftRegion.getClosest(current);
            }

            public Translation2d getRightTranslation(Translation2d current) {
                return rightRegion.getClosest(current);
            }
        }
    }

    /** Grid arrays [0][0] are the highest and the farthest from substation */
    public final static class Grids {
        public static final ScoringNode[][] blueAllianceGrid = new ScoringNode[9][3];
        public static final ScoringNode[][] redAllianceGrid = new ScoringNode[9][3];
        public static ScoringNode[][] getGrid() {
            return (RobotState.getInstance().getCurrentAlliance() == Alliance.Red)?
                    redAllianceGrid : blueAllianceGrid;
        }

        public static ScoringNode getOpposite(ScoringNode node) {
            int mirroredCol = 8 - node.col;
            switch (node.alliance) {
                case Blue:
                case Invalid:
                default:
                    return redAllianceGrid[mirroredCol][node.row];
                case Red:
                    return blueAllianceGrid[mirroredCol][node.row];
            }
        }

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

        // Calculate regions for every 3 by 3 of nodes
        private static final double distBetweenChargingStationGrid = Units.inchesToMeters(59.0);
        private static final double regionMinX = Units.inchesToMeters(54.05);
        private static final double regionMaxWidth = regionMinX + distBetweenChargingStationGrid;
        private static final double[] regionMaxWidths = new double[] {regionMaxWidth - halfRobotLengthWithBumper, regionMaxWidth, regionMaxWidth};
        private static final double firstRegionLength = Units.inchesToMeters(75.185);
        private static final double secondRegionLength = Units.inchesToMeters(65.55);
        private static final double thirdRegionLength = Units.inchesToMeters(75.345);
        private static final double[] regionLengths = new double[] {firstRegionLength, secondRegionLength, thirdRegionLength};

        public static void initGridNodes() {
            double scoringX = Units.inchesToMeters(54.05) + halfRobotLengthWithBumper;
            for (int wpiY = 0; wpiY < 9; wpiY++) {
                ScoringNode.NodeType type = (wpiY == 1 || wpiY == 4 || wpiY == 7)?
                        ScoringNode.NodeType.CUBE : ScoringNode.NodeType.CONE;
                double yTranslation = distToFirstNodeY + (wpiY * distBetweenNodes);

                // calculating regions for blue nodes
                int currentRegion = wpiY / 3;
                double regionMinY = 0.0;
                for (int i = 0; i < currentRegion; i++)
                    regionMinY += regionLengths[i];
                double regionWidth = regionLengths[currentRegion];
                double regionMaxY = regionMinY + regionWidth;
                double regionMaxX = regionMaxWidths[currentRegion];
                Regions.RectangleRegion region = new Regions.RectangleRegion(regionMaxY, regionMaxX, regionMinY, regionMinX);

                for (int wpiX = 0; wpiX < 3; wpiX++) {
                    type = (wpiX == 2)? ScoringNode.NodeType.HYBRID : type;
                    var height = nodeHeights[wpiX];
                    double xTranslation = nodeXs[wpiX];
                    // decrease height by thirteen inches if it is a cube
                    double zTranslation = type == ScoringNode.NodeType.CUBE?
                        height.getHeight() - Units.inchesToMeters(13.50) : height.getHeight();
                    var translation = new Translation3d(xTranslation, yTranslation, zTranslation);
                    //TODO: scoring positions may change based on height of node
                    // e.x. if its hybrid we may not want to drive all the way up (unless we do?)
                    var scoringPosition = new Pose2d(scoringX, yTranslation, Rotation2d.fromDegrees(180));

                    var node = new ScoringNode(type, height, Alliance.Blue, translation,
                            scoringPosition, region, wpiY, wpiX);
                    blueAllianceGrid[wpiY][wpiX] = node;
                    var transformedNode = ScoringNode.transformBlueToRed(node);
                    redAllianceGrid[wpiY][wpiX] = transformedNode;
//                    System.out.println(transformedNode);
                }
            }
        }


        public static class ScoringNode {
            public final NodeType type;
            public final NodeHeight height;
            public final Alliance alliance;
            public final Translation3d translation;
            public final Pose2d scoringPosition;
            public final Regions.RectangleRegion gridRegion;
            public final int col, row;

            public ScoringNode(NodeType type, NodeHeight height, Alliance alliance,
                               Translation3d translation, Pose2d scoringPosition, Regions.RectangleRegion gridRegion,
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
                        nodeTranslation.getY(), nodeTranslation.getZ());

                Pose2d transformedScoringPosition = mirrorPose(node.scoringPosition);

                var region = node.gridRegion;
                Regions.RectangleRegion transformedRegion = new Regions.RectangleRegion(region.maxY, mirrorXPosition(region.maxX),
                        region.minY, mirrorXPosition(region.minX));

                return new ScoringNode(node.type, node.height, Alliance.Red,
                        transformedTranslation, transformedScoringPosition, transformedRegion,
                        node.col, node.row);
            }

            @Override
            public String toString() {
                return "ScoringNode{" +
                        "type=" + type +
                        ", height=" + height +
                        ", alliance=" + alliance +
                        ", translation=" + translation +
                        ", scoringPosition=" + scoringPosition +
                        ", gridRegion=" + gridRegion +
                        ", col=" + col +
                        ", row=" + row +
                        '}';
            }

            @Override
            public boolean equals(Object o) {
                if (this == o) return true;
                if (!(o instanceof ScoringNode)) return false;
                ScoringNode that = (ScoringNode) o;
                return col == that.col && row == that.row && type == that.type &&
                        height == that.height && alliance == that.alliance &&
                        translation.equals(that.translation) &&
                        scoringPosition.equals(that.scoringPosition) &&
                        gridRegion.equals(that.gridRegion);
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
                double getHeight() {
                    return height;
                }
            }
        }
    }

    public static class Regions {

        public interface Region {
            /** all field coords */
            boolean contains(Pose2d pose);
            /** all field coords */
            default boolean contains(Translation2d translation) {
                return contains(new Pose2d(translation, new Rotation2d()));
            }
            /** all field coords */
            default boolean contains(double x, double y) {
                return contains(new Translation2d(x, y));
            }

            Translation2d getCenter();

            Region getMirrored();

            String toString();
        }

        public static class RectangleRegion implements Region {
            public final double maxY, minY, maxX, minX;

            public RectangleRegion(double maxY, double maxX, double minY, double minX) {
                this.maxY = maxY;
                this.minY = minY;
                this.maxX = maxX;
                this.minX = minX;
            }

            @Override
            public boolean contains(Pose2d pose) {
                return (pose.getX() <= maxX && pose.getX() >= minX) &&
                        (pose.getY() <= maxY && pose.getY() >= minY);
            }

            @Override
            public Translation2d getCenter() {
                return new Translation2d((maxX + minX) / 2.0, (maxY + minY) / 2.0);
            }

            @Override
            public RectangleRegion getMirrored() {
                return new RectangleRegion(maxY, mirrorXPosition(maxX),
                        minY, mirrorXPosition(minX));
            }

            public Translation2d getClosest(Translation2d translation) {
                double x = MathUtil.clamp(translation.getX(), minX, maxX);
                double y = MathUtil.clamp(translation.getY(), minY, maxY);
                return new Translation2d(x, y);
            }

            @Override
            public String toString() {
                return "RectangleRegion{" +
                        "maxY=" + maxY +
                        ", minY=" + minY +
                        ", maxX=" + maxX +
                        ", minX=" + minX +
                        '}';
            }
        }

        public static class PolyShapeRegion implements Region {
            public ArrayList<Region> regions = new ArrayList<>();

            public PolyShapeRegion(Region... regions) {
                Collections.addAll(this.regions, regions);
            }

            @Override
            public boolean contains(Pose2d pose) {
                for (var region : regions) {
                    if (region.contains(pose)) return true;
                }
                return false;
            }

            @Override
            public Translation2d getCenter() {
                double totalX = 0;
                double totalY = 0;
                for (var region : regions) {
                    var center = region.getCenter();
                    totalX += center.getX();
                    totalY += center.getY();
                }
                double avgX =  totalX / (double) regions.size();
                double avgY =  totalY / (double) regions.size();
                return new Translation2d(avgX, avgY);
            }

            @Override
            public PolyShapeRegion getMirrored() {
                ArrayList<Region> transformedRegions = new ArrayList<>();
                for (var region : regions) {
                    transformedRegions.add(region.getMirrored());
                }
                return new PolyShapeRegion(transformedRegions.toArray(new Region[transformedRegions.size()]));
            }

            @Override
            public String toString() {
                return "PolyShapeRegion{" +
                        "regions=" + regions +
                        '}';
            }
        }
    }

    public static double mirrorXPosition(double xToBeMirrored) {
        return FIELD_LENGTH - xToBeMirrored;
    }

    public static Translation2d mirrorTranslation(Translation2d translation) {
        return new Translation2d(mirrorXPosition(translation.getX()), translation.getY());
    }

    public static Rotation2d mirrorRotation(Rotation2d rotation) {
        return new Rotation2d(-rotation.getCos(), rotation.getSin());
    }

    public static Pose2d mirrorPose(Pose2d pose) {
        Translation2d translationMirrored = mirrorTranslation(pose.getTranslation());
        Rotation2d rotationMirrored = mirrorRotation(pose.getRotation());
        return new Pose2d(translationMirrored, rotationMirrored);
    }
}
