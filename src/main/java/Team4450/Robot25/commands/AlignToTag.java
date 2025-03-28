package Team4450.Robot25.commands;

import Team4450.Lib.Util;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import Team4450.Robot25.subsystems.PhotonVision;

import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Robot25.subsystems.DriveBase;


/**
 * This command points the robot to the yaw value from the AprilTag,
 * overriding the controller joystick input and allowing
 * rotation to be commanded seperately from translation.
 */

public class AlignToTag extends Command {
    PIDController rotationController = new PIDController(0.035, 0, 0); // for rotating drivebase
    DriveBase robotDrive;
    PhotonVision photonVision;
    private boolean alsoDrive;
    private boolean initialFieldRel;
    private boolean finished;
    /**
     * @param robotDrive the drive subsystem
     */
    public AlignToTag (DriveBase robotDrive, PhotonVision photonVision, boolean alsoDrive, boolean initialFieldRel) {
        this.robotDrive = robotDrive;
        this.photonVision = photonVision;
        this.alsoDrive = alsoDrive;

        if (alsoDrive) addRequirements(robotDrive);

        SendableRegistry.addLW(rotationController, "AlignToTag Rotation PID");
    }

    public void initialize (){
        Util.consoleLog();

        // store the initial field relative state to reset it later.
        initialFieldRel = robotDrive.getFieldRelative();
        
        finished = false;
        if(initialFieldRel)
            robotDrive.toggleFieldRelative();
        robotDrive.enableTracking();
        robotDrive.enableTrackingSlowMode();
        
        rotationController.setSetpoint(0);
        rotationController.setTolerance(0.5);

        SmartDashboard.putString("AlignToTag", "Tag Tracking Initialized");
    }

    @Override
    public void execute() {
      // logic for chosing "closest" target in PV subsystem
      PhotonTrackedTarget target = photonVision.getClosestTarget();

      if (target == null) {
        robotDrive.setTrackingRotation(Double.NaN); // temporarily disable tracking
        robotDrive.clearPPRotationOverride();
        return;
    }

        double targetYaw = target.getYaw();
        double rotation = rotationController.calculate(targetYaw); // attempt to minimize


        Util.consoleLog("in[yaw=%f, pitch=%f] out[rot=%f]", target.getYaw(), target.getPitch(), rotation);

        if (alsoDrive) {
            robotDrive.driveRobotRelative(0, 0, rotation);
            

        } else {
            robotDrive.setTrackingRotation(rotation);
        }

        if(rotation < 0.07){
            finished = true;
        }
    }

    public boolean isFinished(){
        if(finished){
            return true;
        }
        
        else{
            return false;
        }

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

        SmartDashboard.putString("AlignToTag", "Tag Tracking Ended");

    }
}