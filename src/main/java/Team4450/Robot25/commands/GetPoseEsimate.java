package Team4450.Robot25.commands;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import Team4450.Robot25.subsystems.PhotonVision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import Team4450.Robot25.subsystems.DriveBase;


/**
 * This command points the robot to the yaw value from the AprilTag,
 * overriding the controller joystick input and allowing
 * rotation to be commanded seperately from translation.
 */

public class GetPoseEsimate extends Command {
    DriveBase robotDrive;
    PhotonVision photonVision;
    private boolean alsoDrive;
    private boolean initialFieldRel;
    /**
     * @param robotDrive the drive subsystem
     */

    public GetPoseEsimate (DriveBase robotDrive, PhotonVision photonVision, boolean alsoDrive, boolean initialFieldRel) {
        this.robotDrive = robotDrive;
        this.photonVision = photonVision;
        this.alsoDrive = alsoDrive;

        if (alsoDrive) addRequirements(robotDrive);
    }

    public void initialize (){
        Util.consoleLog();

        // store the initial field relative state to reset it later.
        initialFieldRel = robotDrive.getFieldRelative();
        
        if(initialFieldRel)
            robotDrive.toggleFieldRelative();
        robotDrive.enableTracking();
        robotDrive.enableTrackingSlowMode();

        SmartDashboard.putString("GetPoseEsimate", "Pose Estimator Ended");
    }

    @Override
    public void execute() {
      // logic for chosing "closest" target in PV subsystem
    
        Optional<EstimatedRobotPose> target = photonVision.getEstimatedPose();

        // PhotonTrackedTarget target = photonVision.getClosestTarget();
        
        if (target == null || !target.isPresent()) {
            robotDrive.setTrackingRotation(Double.NaN); // temporarily disable tracking
            robotDrive.clearPPRotationOverride();
            return;
        }

        // What about being on the red or blue side
        Util.consoleLog(String.valueOf(target.get().estimatedPose.getX()));
        Util.consoleLog(String.valueOf(target.get().estimatedPose.getY()));
    }
    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        
        if (alsoDrive) robotDrive.drive(0, 0, 0, false);
        
        if (initialFieldRel) robotDrive.toggleFieldRelative(); // restore beginning state
        
        robotDrive.setTrackingRotation(Double.NaN);
        robotDrive.disableTracking();
        robotDrive.disableTrackingSlowMode();
        robotDrive.clearPPRotationOverride();

        SmartDashboard.putString("GetPoseEsimate", "Pose Estimator Ended");

    }
}
