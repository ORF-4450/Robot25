package Team4450.Robot25.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Lib.Util;
import Team4450.Robot25.AdvantageScope;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A class that wraps a single camera connected to the
 * PhotonVision system running on a coprocessor
 * and adds utility functions to support using vision.
 * Note: Communication between this class and the PV on the coprocessor
 * is handled through the Network Tables and the tables are wrapped by
 * by PhotonLib.
 */
public class PhotonVision extends SubsystemBase
{
    private PhotonCamera            camera;
    private PhotonPipelineResult    latestResult;

    private VisionLEDMode           ledMode = VisionLEDMode.kOff;

    private VisionSystemSim         visionSim;
    private PhotonCameraSim         cameraSim;

    private Field2d                 field = new Field2d();

    // change the field layout for other years!
    private final AprilTagFields    fields = AprilTagFields.k2025ReefscapeWelded; 
    private AprilTagFieldLayout     fieldLayout;
    private PhotonPoseEstimator     poseEstimator;

    private Transform3d             robotToCam;
    private PipelineType            pipelineType;

    public static enum PipelineType {APRILTAG_TRACKING, OBJECT_TRACKING, POSE_ESTIMATION};

    /**
     * Create an instance of PhotonVision class for a camera with a default transform. (One per camera)
     * @param cameraName the name in PhotonVision used for the camera like HD_USB_Camera
     *                   (likely from manufacturer, best not to change it to avoid conflict issues -cole)
     * @param pipelineType the PipelineType of what it's going to be used for
     */
    public PhotonVision(String cameraName, PipelineType pipelineType) {
        this(cameraName, pipelineType, new Transform3d());
    }

    public Transform3d getRobotToCam() {
        return robotToCam;
    }

    /**
     * Create an instance of PhotonVision class for a camera with a default transform. (One per camera)
     * @param cameraName the name in PhotonVision used for the camera like HD_USB_Camera
     *                   (likely from manufacturer, best not to change it to avoid conflict issues -cole)
     * @param pipelineType the PipelineType of what it's going to be used for
     * @param robotToCam a Tranformation3d of the camera relative to the bottom center of the robot (off floor).
     */
	public PhotonVision(String cameraName, PipelineType pipelineType, Transform3d robotToCam) {
        this.pipelineType = pipelineType;
        this.robotToCam = robotToCam;
        this.camera = new PhotonCamera(cameraName);
        fieldLayout = AprilTagFieldLayout.loadField(fields);
        camera = new PhotonCamera(cameraName);
        fieldLayout = AprilTagFieldLayout.loadField(fields);
        if (pipelineType == PipelineType.POSE_ESTIMATION) {
            // setup the AprilTag pose etimator.
            poseEstimator = new PhotonPoseEstimator(
                fieldLayout, // feed in the current year's field layout
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, // best one as far as we can tell
                //camera,
                robotToCam
            );
        }
		Util.consoleLog("PhotonVision (%s) created!", cameraName);
        SmartDashboard.putData(field);
	}

    /**
     * Select camera's image processing pipeline.
     * @param index Zero based number of desired pipeline.
     */
    public void selectPipeline(int index)
    {
        Util.consoleLog("%d", index);

        camera.setPipelineIndex(index);
    }

    /**
     * Whether the camera is used for pose estimation or just normal apriltags
     * @return true if apriltag/pose est., false if object detection or reflective tape or other
     */
    public boolean isAprilTag() {
        return pipelineType != PipelineType.OBJECT_TRACKING;
    }

    public Optional<PhotonPipelineResult> getLatestResult() {
        List<PhotonPipelineResult> targets = camera.getAllUnreadResults();
        if (targets.isEmpty()) {
            if (latestResult == null) {
                return Optional.empty();
            }
        } else {
            // If there is a number of targets grab that latest one.
            // Latest one will be the last because it is an FIFO list.
            latestResult = targets.get(targets.size() - 1);
        }
        return Optional.of(latestResult);
    }

    public boolean hasTargets()
    {
        getLatestResult();

        return latestResult.hasTargets();
    }
    public PhotonTrackedTarget getClosestTarget() {
        PhotonTrackedTarget closest; // will hold the current closest for replacement or return

        if (latestResult != null && latestResult.hasTargets()) {
            List<PhotonTrackedTarget> targets = latestResult.getTargets();
            closest = targets.get(0); // start with first target

            for (int i = 0; i < targets.size(); i++) {
                if (targets.get(i).getArea() > closest.getArea()) // compare picth to closest
                    // if it's closer that closest, replace closest with it!
                    closest = targets.get(i);
            }
            return closest;
        }
        else
            return null;
    }

    /**
     * Returns an Optional value of the robot's estimated 
     * field-centric pose given current tags that it sees.
     * (and also the timestamp)
     * 
     * @return The Optional estimated pose (empty optional means no pose or uncertain/bad pose).
     */
    public Optional<EstimatedRobotPose> getEstimatedPose() {
        PhotonPipelineResult photonResultToUse;

        List<PhotonPipelineResult> targets = camera.getAllUnreadResults();
        if (targets.isEmpty()) {
            if (latestResult == null) {
                return Optional.empty();
            }
        } else {
            // If there is a number of targets grab that latest one.
            // Latest one will be the last because it is an FIFO list.
            latestResult = targets.get(targets.size() - 1);
        }
        photonResultToUse = latestResult;

        Optional<EstimatedRobotPose> estimatedPoseOptional = poseEstimator.update(photonResultToUse);

        if (estimatedPoseOptional.isPresent()) {
            EstimatedRobotPose estimatedPose = estimatedPoseOptional.get();
            Pose3d pose = estimatedPose.estimatedPose;

            // pose2d to pose3d (ignore the Z axis which is height off ground)
            Pose2d pose2d = new Pose2d(pose.getX(), pose.getY(), new Rotation2d(pose.getRotation().getAngle()));

            poseEstimator.setLastPose(pose);
            // update the field2d object in NetworkTables to visualize where the camera thinks it's at
            field.setRobotPose(pose2d);

            // logic for checking if pose is valid would go here:
            // for example:
            ArrayList<Pose3d> usedTagPoses = new ArrayList<Pose3d>();


            for (int i = 0; i < estimatedPose.targetsUsed.size(); i++) {
                int id = estimatedPose.targetsUsed.get(i).getFiducialId();
                // if a target was used with ID > 16 then return no estimated pose
                if (id > 16) {
                    return Optional.empty();
                }


                Optional<Pose3d> tagPose = fieldLayout.getTagPose(id);

                if (tagPose.isPresent())
                    usedTagPoses.add(tagPose.get());
            }

            return Optional.of(estimatedPose);
        } else {
            return Optional.empty();
        }
    }
}