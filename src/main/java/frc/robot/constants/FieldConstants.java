package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.io.IOException;
import java.util.List;

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

    public static class Grids {
        public static ScoringNode[][] blueAllianceNodes = new ScoringNode[9][3];
        public static ScoringNode[][] blueAllianceConeNodes = new ScoringNode[6][3];
        public static ScoringNode[][] blueAllianceCubeNodes = new ScoringNode[3][3];

        public static ScoringNode[][] redAllianceNodes = new ScoringNode[9][3];
        public static ScoringNode[][] redAllianceConeNodes = new ScoringNode[6][3];
        public static ScoringNode[][] redAllianceCubeNodes = new ScoringNode[3][3];

        static {

        }

        public static Translation3d[] redAllianceNodesTranslations;

        public static class ScoringNode {
            public final GamePieceType type;
            public final Alliance alliance;
            public final Translation3d translation;
            public final Pose2d scoringPosition;

            public ScoringNode(GamePieceType type, Alliance alliance,
                               Translation3d translation, Pose2d scoringPosition) {
                this.type = type;
                this.alliance = alliance;
                this.translation = translation;
                this.scoringPosition = scoringPosition;
            }

            public static ScoringNode transformToAlliance(ScoringNode node, DriverStation.Alliance alliance) {
                if (node.alliance == alliance) return node;

                double halfLengthOfField = FIELD_LENGTH / 2.0;
                //distance to field middle plus middle of field value
                double transformedX = (halfLengthOfField - node.translation.getX()) + halfLengthOfField;
                Translation3d transformedTranslation = new Translation3d(transformedX,
                        node.translation.getY(), node.translation.getY());

                double transformedScoringX = (halfLengthOfField - node.scoringPosition.getX()) + halfLengthOfField;
                Rotation2d transformedRotation = node.scoringPosition.getRotation().times(-1);
                Pose2d transformedScoringPosition = new Pose2d(transformedScoringX,
                        node.scoringPosition.getY(), transformedRotation);

                return new ScoringNode(node.type, node.alliance, transformedTranslation, transformedScoringPosition);
            }
        }
    }

    public enum GamePieceType {
        CUBE, CONE
    }
}
