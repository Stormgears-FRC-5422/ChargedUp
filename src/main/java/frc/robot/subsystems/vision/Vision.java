package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotState;
import frc.utils.data.StormStruct;
import frc.utils.subsystemUtils.StormSubsystemBase;

import java.util.Vector;

public class Vision extends StormSubsystemBase {

    private int logCounter = 0;
    private final StormStruct m_struct;
    private final Vector<AprilTagData> currentAprilTags = new Vector<>();
    private boolean visionReady = false;

    private NetworkTableInstance inst = NetworkTableInstance.getDefault();
    private NetworkTable table = inst.getTable("vision-data");
    private IntegerPublisher xPub = table.getIntegerTopic("vision_mode").publish();

    public Vision() {
        var ntInst = NetworkTableInstance.getDefault();
        m_struct = new StormStruct(ntInst, "vision-data", "tag_data");
    }

    @Override
    public void stormPeriodic() {
        logCounter ++;
        // grab current info list which is a vector of a hashmap which contains
        // a bunch of stuff for each april tag currently in view
        var infoList = m_struct.get_data("april_tag");
        // exit if there is none
        visionReady = false;
        if (infoList.size() < 1) return;
        visionReady = true;
        // clear the april tag vector
        currentAprilTags.clear();
        for (var info : infoList) {
//            System.out.println("yaw from vision" + info.get("yaw"));
            var aprilTagData = new AprilTagData(
                    info.get("id").intValue(),
                    info.get("distance"),
                    info.get("yaw"),
                    info.get("leftright")
            );
            if (logCounter % 25 == 0)
                System.out.println(aprilTagData);
            currentAprilTags.add(aprilTagData);
        }
        // convert timestamp to seconds
        double timeSeconds = infoList.get(0).get("timestamp") / Math.pow(10, 6);
        if (logCounter % 25 == 0) {
//            System.out.println("timestamp: " + timeSeconds);
//            System.out.println(AprilTagPoseEstimationStrategy.fromAprilTagData(currentAprilTags, new Rotation2d()));
        }
        RobotState.getInstance().setVisionData(timeSeconds, currentAprilTags);
    }

    public static class AprilTagData {
        public final int id;
        public final double dist, yawDegrees, offCenterDegrees;

        public AprilTagData(int id, double dist, double yawDegrees, double offCenterDegrees) {
            this.id = id;
            this.dist = dist;
            this.yawDegrees = yawDegrees;
            this.offCenterDegrees = offCenterDegrees;
        }

        @Override
        public String toString() {
            return "{" +
                    "id=" + id +
                    ", dist=" + dist +
                    ", yawDegrees=" + yawDegrees +
                    ", offCenterDegrees=" + offCenterDegrees +
                    '}';
        }
    }
    public void setMode(int x){
        System.out.println("Before");
        xPub.set(x);
        System.out.println("After");
    }

    public boolean getAprilTagDetected() {
        return visionReady;
    }
}
