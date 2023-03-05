package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotState;
import frc.utils.data.StormStruct;
import frc.utils.subsystemUtils.StormSubsystemBase;

import java.util.Vector;

public class Vision extends StormSubsystemBase {

    private final StormStruct m_struct;
    private final Vector<AprilTagData> currentAprilTags = new Vector<>();

    public Vision() {
        var ntInst = NetworkTableInstance.getDefault();
        m_struct = new StormStruct(ntInst, "vision-data", "tag-data");
    }

    @Override
    public void stormPeriodic() {
        currentAprilTags.clear();
        // grab current info list which is a vector of a hashmap which contains
        // a bunch of stuff for each april tag currently in view
        var infoList = m_struct.get_data("april tag");
        // exit if there is none
        if (infoList.size() < 1) return;
        for (var info : infoList) {
            var aprilTagData = new AprilTagData(
                    info.get("id").intValue(),
                    info.get("distance"),
                    info.get("yaw"),
                    info.get("off-center")
            );
            currentAprilTags.add(aprilTagData);
        }
        // convert timestamp to seconds
        double timeSeconds = infoList.get(0).get("timestamp") / 0.01;
        RobotState.getInstance().addVisionData(timeSeconds, currentAprilTags);
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
    }
}
