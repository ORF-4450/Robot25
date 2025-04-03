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

public class DriveToAlgaeTag extends Command {
    PIDController rotationController = new PIDController(0.03, 0, 0); // for rotating drivebase
    PIDController translationController = new PIDController(0.08, 0.005, 0); // for moving drivebase in X,Y plane
    DriveBase robotDrive;
    PhotonVision photonVision;
    private boolean alsoDrive;
    private boolean initialFieldRel;
    /**
     * @param robotDrive the drive subsystem
     */
    public DriveToAlgaeTag (DriveBase robotDrive, PhotonVision photonVision, boolean alsoDrive, boolean initialFieldRel) {
        this.robotDrive = robotDrive;
        this.photonVision = photonVision;
        this.alsoDrive = alsoDrive;

        // if (alsoDrive) addRequirements(robotDrive);

        SendableRegistry.addLW(translationController, "DriveToAlgaeTag Translation PID");
        SendableRegistry.addLW(rotationController, "DriveToAlgaeTag Rotation PID");
    }

    public void initialize (){
        Util.consoleLog();

        // store the initial field relative state to reset it later.
        initialFieldRel = robotDrive.getFieldRelative();
        
        if(initialFieldRel)
            robotDrive.toggleFieldRelative();
        robotDrive.enableTracking();
        robotDrive.enableTrackingSlowMode();
        
        rotationController.setSetpoint(0);
        rotationController.setTolerance(0.5);

        translationController.setSetpoint(-15); // target should be at -15 pitch
        translationController.setTolerance(0.5);

        SmartDashboard.putString("DriveToAlgaeTag", "Tag Tracking Initialized");
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
        double targetPitch = target.getPitch();

        double rotation = rotationController.calculate(targetYaw); // attempt to minimize
        double movement = translationController.calculate(targetPitch); // attempt to minimize


        Util.consoleLog("in[yaw=%f, pitch=%f] out[rot=%f, mov=%f]", target.getYaw(), target.getPitch(), rotation, movement);

        if (alsoDrive) {
            robotDrive.driveRobotRelative(0, -movement, rotation);

        } else {
            robotDrive.setTrackingRotation(rotation);
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

        SmartDashboard.putString("DriveToAlgaeTag", "Tag Tracking Ended");

    }
}