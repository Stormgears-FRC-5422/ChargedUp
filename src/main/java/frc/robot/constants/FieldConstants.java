package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.io.IOException;
import java.util.List;

import static frc.robot.constants.Constants.*;

public final class FieldConstants {
    public final static double FIELD_LENGTH = Units.feetToMeters(54) + Units.inchesToMeters(3.25);
    public final static double FIELD_WIDTH = Units.feetToMeters(26) + Units.inchesToMeters(3.5);

    public static final AprilTagFieldLayout APRILTAG_FIELD_LAYOUT;
    static {
        try {
            APRILTAG_FIELD_LAYOUT = AprilTagFields.k2023ChargedUp.loadAprilTagLayoutField();
        } catch (IOException e) {
            System.out.println("Could not load april tag field layout! with message " + e);
            throw new RuntimeException(e);
        }
    }

    public static List<AprilTag> APRILTAGS = APRILTAG_FIELD_LAYOUT.getTags();

    /** Grid arrays [0][0] are the highest and the farthest from substation */
    public final static class Grids {
        public static ScoringNode[][] blueAllianceGrid = new ScoringNode[9][3];
        public static ScoringNode[][] redAllianceGrid = new ScoringNode[9][3];

        private static final double distBetweenNodes = Units.inchesToMeters(22.0);
        private static final double distToFirstNodeY = Units.inchesToMeters(20.0);
        //heights of cone nodes cube nodes don't really matter maybe we can make them a bit lower than the cone nodes
        //hybrid node (last in array) is 10 inches of ground just because
        private static final double[] nodeZs = {Units.inchesToMeters(46.0), Units.inchesToMeters(34.0), Units.inchesToMeters(10.0)};
        private static final ScoringNode.NodeHeight[] nodeHeights = {
            ScoringNode.NodeHeight.HIGH, ScoringNode.NodeHeight.MIDDLE, ScoringNode.NodeHeight.HYBRID
        };
        //last one is kind of a guess (hybrid node)
        private static final double[] nodeXs = {Units.inchesToMeters(14.32), Units.inchesToMeters(31.35), Units.inchesToMeters(47.0)};

        static {
            double scoringX = Units.inchesToMeters(54.05) + (ROBOT_LENGTH / 2.0) + BUMPER_THICKNESS;
            for (int wpiY = 0; wpiY < 9; wpiY++) {
                ScoringNode.NodeType type = (wpiY == 1 || wpiY == 4 || wpiY == 7)? ScoringNode.NodeType.CUBE : ScoringNode.NodeType.CONE;
                double yTranslation = distToFirstNodeY + (wpiY * distBetweenNodes);
                for (int wpiX = 0; wpiX < 3; wpiX++) {
                    type = (wpiX == 2)? ScoringNode.NodeType.HYBRID : type;
                    double xTranslation = nodeXs[wpiX];
                    double zTranslation = nodeZs[wpiX];
                    var height = nodeHeights[wpiX];
                    var translation = new Translation3d(xTranslation, yTranslation, zTranslation);
                    //TODO: scoring positions may change based on height of node
                    // e.x. if its hybrid we may not want to drive all the way up (unless we do?)
                    var scoringPosition = new Pose2d(scoringX, yTranslation, Rotation2d.fromDegrees(180));
                    var node = new ScoringNode(type, height, Alliance.Blue, translation, scoringPosition);
                    blueAllianceGrid[wpiY][wpiX] = node;
                    var transformedNode = ScoringNode.transformBlueToRed(node);
                    redAllianceGrid[wpiY][wpiX] = transformedNode;
                    System.out.println("Blue Alliance Node: " + node);
                    System.out.println("Red Alliance Node: " + transformedNode);
                }
            }
        }

        public static Translation3d[] redAllianceNodesTranslations;

        public static class ScoringNode {
            public final NodeType type;
            public final NodeHeight height;
            public final Alliance alliance;
            public final Translation3d translation;
            public final Pose2d scoringPosition;

            public ScoringNode(NodeType type, NodeHeight height, Alliance alliance,
                               Translation3d translation, Pose2d scoringPosition) {
                this.type = type;
                this.height = height;
                this.alliance = alliance;
                this.translation = translation;
                this.scoringPosition = scoringPosition;
            }

            public static ScoringNode nodeFromTranslation(Translation3d translation) {
                for (int i = 0; i < 9; i++) {
                    for (int j = 0; j < 3; j++) {
                        if (blueAllianceGrid[i][j].translation.equals(translation)) return blueAllianceGrid[i][j];
                        if (redAllianceGrid[i][j].translation.equals(translation)) return redAllianceGrid[i][j];
                    }
                }
                System.out.println("No such node with translation: " + translation);
                return blueAllianceGrid[0][0];
            }

            public static ScoringNode transformBlueToRed(ScoringNode node) {
                if (node.alliance == Alliance.Red) return node;
                //transform Y so that arrays are correct positioning since nodes are mirrored
                double blueMiddleNodeY = Units.inchesToMeters(108);
                Translation3d nodeTranslation = node.translation;
                double transformedY = (blueMiddleNodeY - nodeTranslation.getY()) + blueMiddleNodeY;
                //mirror X across field
                double halfLengthOfField = FIELD_LENGTH / 2.0;
                //distance to field middle plus middle of field value
                double transformedX = (halfLengthOfField - nodeTranslation.getX()) + halfLengthOfField;
                Translation3d transformedTranslation = new Translation3d(transformedX,
                        transformedY, nodeTranslation.getZ());

                Pose2d scoringPosition = node.scoringPosition;
                double transformedScoringX = (halfLengthOfField - scoringPosition.getX()) + halfLengthOfField;
                Rotation2d transformedRotation = scoringPosition.getRotation().times(-1);
                Pose2d transformedScoringPosition = new Pose2d(transformedScoringX,
                        transformedY, transformedRotation);

                return new ScoringNode(node.type, node.height, Alliance.Red, transformedTranslation, transformedScoringPosition);
            }

            @Override
            public String toString() {
                return String.format("ScoringNode(type: %1$s, height: %2$s, alliance: %3$s,translation: %4$s, scoringPosition: %5$s)",
                        type, height, alliance,translation, scoringPosition);
            }
            public enum NodeType {
                CUBE, CONE, HYBRID
            }

            public enum NodeHeight {
                HIGH, MIDDLE, HYBRID
            }
        }
    }


}
