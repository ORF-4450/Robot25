package Team4450.Robot25.utility;

import java.util.HashMap;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AprilTagMap {

    public static HashMap<Integer, Pose2d> aprilTagToPoseMap = new HashMap<>();

static {
    // april id, location to go to
    // red side
    // aprilTagToPoseMap.put(1, new Pose2d(16.7, 0.66, new Rotation2d(Math.toRadians(126))));
    // aprilTagToPoseMap.put(2, new Pose2d(16.7, 7.40, new Rotation2d(Math.toRadians(-126))));
    // aprilTagToPoseMap.put(3, new Pose2d(11.56, 8.06, new Rotation2d(Math.toRadians(-90))));
    // aprilTagToPoseMap.put(4, new Pose2d(9.28, 6.14, new Rotation2d(Math.toRadians(0))));
    // aprilTagToPoseMap.put(5, new Pose2d(9.28, 1.91, new Rotation2d(Math.toRadians(0))));
    aprilTagToPoseMap.put(6, new Pose2d(13.47, 3.31, new Rotation2d(Math.toRadians(-60))));
    aprilTagToPoseMap.put(7, new Pose2d(13.89, 4.03, new Rotation2d(Math.toRadians(0))));
    aprilTagToPoseMap.put(8, new Pose2d(13.43, 4.75, new Rotation2d(Math.toRadians(60))));
    aprilTagToPoseMap.put(9, new Pose2d(12.64, 4.75, new Rotation2d(Math.toRadians(120))));
    aprilTagToPoseMap.put(10, new Pose2d(12.23, 4.03, new Rotation2d(Math.toRadians(180))));
    aprilTagToPoseMap.put(11, new Pose2d(12.64, 3.31, new Rotation2d(Math.toRadians(-120))));
    // blue side 
    // aprilTagToPoseMap.put(12, new Pose2d(0.85, 0.66, new Rotation2d(Math.toRadians(54))));
    // aprilTagToPoseMap.put(13, new Pose2d(0.85, 7.40, new Rotation2d(Math.toRadians(-54))));
    // aprilTagToPoseMap.put(14, new Pose2d(8.27, 6.14, new Rotation2d(Math.toRadians(180))));
    // aprilTagToPoseMap.put(15, new Pose2d(8.27, 1.91, new Rotation2d(Math.toRadians(180))));
    // aprilTagToPoseMap.put(16, new Pose2d(5.99, 0.00, new Rotation2d(Math.toRadians(90))));
    aprilTagToPoseMap.put(17, new Pose2d(4.07, 3.31, new Rotation2d(Math.toRadians(-120))));
    aprilTagToPoseMap.put(18, new Pose2d(3.66, 4.03, new Rotation2d(Math.toRadians(180))));
    aprilTagToPoseMap.put(19, new Pose2d(4.07, 4.75, new Rotation2d(Math.toRadians(120))));
    aprilTagToPoseMap.put(20, new Pose2d(4.90, 4.75, new Rotation2d(Math.toRadians(60))));
    aprilTagToPoseMap.put(21, new Pose2d(5.32, 4.03, new Rotation2d(Math.toRadians(0))));
    aprilTagToPoseMap.put(22, new Pose2d(4.90, 3.31, new Rotation2d(Math.toRadians(-60))));
    }
}