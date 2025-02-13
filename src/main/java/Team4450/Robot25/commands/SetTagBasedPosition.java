package Team4450.Robot25.commands;

import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Lib.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import Team4450.Robot25.utility.AprilTagMap;
import Team4450.Robot25.Constants;
import Team4450.Robot25.subsystems.DriveBase;
import Team4450.Robot25.subsystems.PhotonVision;


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
    private boolean scoreLeft;
    /**
     * @param robotDrive the drive subsystem
     */

    public SetTagBasedPosition (DriveBase robotDrive, PhotonVision photonVision, boolean scoreLeft) {
        this.robotDrive = robotDrive;
        this.photonVision = photonVision;
        this.scoreLeft = scoreLeft; // If true score on left side else score on right side
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

        if (target != null && photonVision.hasTargets()) {
            // Set status on smartdashboard instead? (upgrade)
            Util.consoleLog("TARGET FOUND");

            Pose2d aprilTagPose = AprilTagMap.aprilTagToPoseMap.get(target.getFiducialId());
            if (aprilTagPose != null) {
                // Offset pose by robot dist
                // Offset by left or right dist
                Translation2d robotOffset = new Translation2d(0, 0);
                if (scoreLeft) { // Score Left
                    robotOffset = new Translation2d(Constants.robotCoralLongitudinalScoringDistance, Constants.robotCoralLateralScoringOffset);
                } else { // Score Right
                    robotOffset = new Translation2d(Constants.robotCoralLongitudinalScoringDistance, -Constants.robotCoralLateralScoringOffset);
                }
                Translation2d robotTargetPose = aprilTagPose.getTranslation().plus(robotOffset.rotateBy(aprilTagPose.getRotation().unaryMinus()));
                robotDrive.setTargetPose(new Pose2d(robotTargetPose, new Rotation2d(target.getYaw())));
                Util.consoleLog("APRIL TAG POSE: " + String.valueOf(aprilTagPose));
                Util.consoleLog("ROBOT OFFSET: " + String.valueOf(robotOffset));
                Util.consoleLog("TARGET POSE: " + String.valueOf(robotTargetPose));
                // Util.consoleLog("TARGET", String.valueOf(target));
            } else {
                // Set warning on smartdashboard instead? (Upgrade)
                Util.consoleLog("Target not on reef");
                return;
            }
            Util.consoleLog(String.valueOf(AprilTagMap.aprilTagToPoseMap.get(target.getFiducialId())));
        } else {
            // Set warning on smartdashboard instead? (Upgrade)
            Util.consoleLog("NO TARGET FOUND");
            return;
        }

        if (robotDrive.getTargetPose().getX() == 0 || robotDrive.getTargetPose().getY() == 0) {
            // Smartdashboard warning on target assignment (Upgrade)
            Util.consoleLog("NO TARGET ASSIGNED");
            return;
        }
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        Util.consoleLog();

        robotDrive.setTrackingRotation(Double.NaN);
        robotDrive.disableTracking();
        robotDrive.disableTrackingSlowMode();
        robotDrive.clearPPRotationOverride();

        SmartDashboard.putString("SetTagBasedPostion", "Tag Tracking Ended");

    }
}