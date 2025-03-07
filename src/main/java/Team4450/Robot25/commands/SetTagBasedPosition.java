package Team4450.Robot25.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Lib.Util;
import Team4450.Robot25.utility.AprilTagMap;
import Team4450.Robot25.Constants;
import Team4450.Robot25.subsystems.DriveBase;
import Team4450.Robot25.subsystems.PhotonVision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;



/**
   * This function uses the current robot position estimate that is build from odometry and apriltags to go to a location on the field.
   * Will return imidiatly when the location is reached.
   *
   * 
   * @return Void
   */

public class SetTagBasedPosition extends Command {
    DriveBase robotDrive;
    PhotonVision photonVision;
    private int side;
    private boolean algaeRemove;
    private boolean isFinished;
    /**
     * @param robotDrive the drive subsystem
     */

    public SetTagBasedPosition (DriveBase robotDrive, PhotonVision photonVision, int side, boolean algaeRemove) {
        this.robotDrive = robotDrive;
        this.photonVision = photonVision;
        this.side = side; // If -1 score on left side, If 0 align with middle, If 1 score on right side
        this.algaeRemove = algaeRemove;
    }

    public void initialize () {
        Util.consoleLog();

        robotDrive.enableTracking();
        robotDrive.enableTrackingSlowMode();
        
        SmartDashboard.putString("SetTagBasedPostion", "Tag Tracking Initialized");
    }

    @Override
    public void execute() {
        PhotonTrackedTarget target = photonVision.getLatestResult().getBestTarget();

        if (target != null && photonVision.hasTargets()) { // Return early with no targets

            Pose2d aprilTagPose = AprilTagMap.aprilTagToPoseMap.get(target.getFiducialId()); // Get april tag number
            if (aprilTagPose != null) { // Quick null check
                // Deal with going left or right for coral scoring
                Translation2d robotOffset = new Translation2d(0, 0);
                if (side == -1) { // Score Left
                    robotOffset = new Translation2d(Constants.robotCoralLongitudinalScoringDistance, Constants.robotCoralLateralScoringOffset);
                } else if(side == 1) { // Score Right
                    robotOffset = new Translation2d(Constants.robotCoralLongitudinalScoringDistance, -Constants.robotCoralLateralScoringOffset);
                }
                else if(side == 0){ // Align with middle
                    robotOffset = new Translation2d(Constants.robotCoralLongitudinalScoringDistance, 0);
                }
                // Matrix multiplication to rotate based on where on the reef the target is.
                Translation2d robotTargetPose = aprilTagPose.getTranslation().plus(robotOffset.rotateBy(aprilTagPose.getRotation().unaryMinus()));

                if (algaeRemove) { // If algae remove rotate 180
                    robotDrive.setTargetPose(new Pose2d(robotTargetPose, new Rotation2d(Math.toRadians(aprilTagPose.getRotation().getDegrees() - Math.toDegrees(Constants.CAMERA_TAG_TRANSFORM.getRotation().getAngle()) - 180)))); 
                } else {
                    robotDrive.setTargetPose(new Pose2d(robotTargetPose, new Rotation2d(Math.toRadians(aprilTagPose.getRotation().getDegrees() - Math.toDegrees(Constants.CAMERA_TAG_TRANSFORM.getRotation().getAngle()) - 90)))); 
                }
                isFinished = true; // Position has been set, return.
            } else {
                return;
            }
        } else {
            // Set warning on smartdashboard instead? (Upgrade)
            // Util.consoleLog("NO TARGET FOUND");
            return;
        }

        if (robotDrive.getTargetPose().getX() == 0 || robotDrive.getTargetPose().getY() == 0) {
            // Smartdashboard warning on target assignment (Upgrade)
            // Util.consoleLog("NO TARGET ASSIGNED");
            return;
        }
    }

    public boolean isFinished() {
        if (isFinished) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);

        // Telemetry for correct exit.
        Util.consoleLog("Correctly Set Tag Based Position");

        robotDrive.setTrackingRotation(Double.NaN);
        robotDrive.disableTracking();
        robotDrive.disableTrackingSlowMode();
        robotDrive.clearPPRotationOverride();

        SmartDashboard.putString("SetTagBasedPostion", "Tag Tracking Ended");
    }
}
