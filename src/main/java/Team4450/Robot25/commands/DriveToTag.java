package Team4450.Robot25.commands;

import Team4450.Lib.Util;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import Team4450.Robot25.subsystems.PhotonVision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import Team4450.Robot25.Constants;
import Team4450.Robot25.subsystems.DriveBase;


/**
 * This command points the robot to the yaw value from the AprilTag,
 * overriding the controller joystick input and allowing
 * rotation to be commanded seperately from translation.
 */

public class DriveToTag extends Command {
    PIDController rotationController = new PIDController(0.02, 0.1, 0); // for rotating drivebase
    PIDController translationControllerX = new PIDController(0.5, 0.2, 0); // for moving drivebase in X,Y plane
    PIDController translationControllerY = new PIDController(0.5, 0.2, 0); // for moving drivebase in X,Y plane
    DriveBase robotDrive;
    PhotonVision photonVision;
    private double targetRobotX;
    private double targetRobotY;
    private double targetRobotRot;
    private boolean alsoDrive;
    private boolean initialFieldRel;
    /**
     * @param robotDrive the drive subsystem
     */

    public DriveToTag (DriveBase robotDrive, PhotonVision photonVision, boolean alsoDrive, boolean initialFieldRel, double targetRobotX, double targetRobotY, double targetRobotRot) {
        this.robotDrive = robotDrive;
        this.photonVision = photonVision;
        this.alsoDrive = alsoDrive;
        this.targetRobotX = targetRobotX;
        this.targetRobotY = targetRobotY;
        this.targetRobotRot = targetRobotRot;

        if (alsoDrive) addRequirements(robotDrive);

        SendableRegistry.addLW(translationControllerX, "DriveToTag Translation PID");
        SendableRegistry.addLW(translationControllerY, "DriveToTag Translation PID");
        SendableRegistry.addLW(rotationController, "DriveToTag Rotation PID");
    }

    public void initialize (){
        Util.consoleLog();

        // store the initial field relative state to reset it later.
        initialFieldRel = robotDrive.getFieldRelative();
        
        if(initialFieldRel)
            robotDrive.toggleFieldRelative();
        robotDrive.enableTracking();
        robotDrive.enableTrackingSlowMode();
        
        rotationController.setSetpoint(targetRobotRot);
        rotationController.setTolerance(0.5);

        // translationControllerX.setSetpoint(-15); // target should be at -15 pitch
        translationControllerX.setSetpoint(targetRobotX - Constants.xCameraOffset);
        translationControllerX.setTolerance(0.5);

        translationControllerY.setSetpoint(targetRobotY - Constants.yCameraOffset);
        translationControllerY.setTolerance(0.5);

        SmartDashboard.putString("DriveToTag", "Tag Tracking Initialized");
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

        // what about being on the red or blue side
        Util.consoleLog(String.valueOf(target.get().estimatedPose.getX()));
        Util.consoleLog(String.valueOf(target.get().estimatedPose.getY()));
        // Util.consoleLog(String.valueOf(target.get().estimatedPose.getY()));
        double movementX;
        double movementY;
        double rotation;

        double toleranceX = 0.1;
        double toleranceY = 0.1;
        double toleranceRot = 1;

        if (target.get().estimatedPose.getX() < targetRobotX - Constants.xCameraOffset - toleranceX || target.get().estimatedPose.getX() > targetRobotX - Constants.xCameraOffset + toleranceX) {
            movementX = translationControllerX.calculate(target.get().estimatedPose.getX());
        } else {
            movementX = 0;
        }
        if (target.get().estimatedPose.getY() < targetRobotY - Constants.yCameraOffset - toleranceY || target.get().estimatedPose.getY() > targetRobotY - Constants.yCameraOffset + toleranceY) {
            movementY = translationControllerY.calculate(target.get().estimatedPose.getY());
        } else {
            movementY = 0;
        }
        if (robotDrive.getGyroYaw() < targetRobotRot - Math.toRadians(toleranceRot) || robotDrive.getGyroYaw() > targetRobotRot + Math.toRadians(toleranceRot)) {
            rotation = rotationController.calculate(robotDrive.getGyroYaw());
        } else {
            rotation = 0;
        }

        // double rotation = rotationController.calculate(target.getYaw()); // attempt to minimize
        // double movementX = translationControllerX.calculate(target.getPitch()); // attempt to minimize

        // Util.consoleLog("in[yaw=%f, pitch=%f] out[rot=%f, mov=%f]", target.getYaw(), target.getPitch(), rotation, movementX);

        // if (alsoDrive) {
        //     robotDrive.driveRobotRelative(-movementX, 0, rotation);
        // } else {
        //     robotDrive.setTrackingRotation(rotation);
        // }

        if (alsoDrive) {
            robotDrive.driveRobotRelative(movementX, movementY, rotation);
            // robotDrive.driveRobotRelative(movementX, 0, 0); // testing just x
        } else {
            robotDrive.setTrackingRotation(0);
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

        SmartDashboard.putString("DriveToTag", "Tag Tracking Ended");

    }
}
