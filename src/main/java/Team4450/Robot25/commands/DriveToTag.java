package Team4450.Robot25.commands;

import Team4450.Lib.Util;
import Team4450.Robot25.Constants;
import Team4450.Robot25.subsystems.DriveBase;
import Team4450.Robot25.subsystems.PhotonVision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import org.opencv.ml.EM;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;



/**
 * This command points the robot to the yaw value from the AprilTag,
 * overriding the controller joystick input and allowing
 * rotation to be commanded seperately from translation.
 */

public class DriveToTag extends Command {
    PIDController rotationController = new PIDController(0.02, 0, 0); // for rotating drivebase
    PIDController translationControllerX = new PIDController(0.4, 0.1, 0); // for moving drivebase in X,Y plane
    PIDController translationControllerY = new PIDController(0.4, 0.1, 0); // for moving drivebase in X,Y plane
    DriveBase robotDrive;
    PhotonVision photonVision;
    private double targetRobotX;
    private double targetRobotY;
    private double targetRobotRot;
    private boolean alsoDrive;
    private boolean initialFieldRel;
    private double currentX;
    private double currentY;
    private double lastTargetX;
    private double lastTargetY;
    private boolean firstLoop;
    private int iCount;
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
        this.firstLoop = true;

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
    
        Optional<EstimatedRobotPose> ttarget = photonVision.getEstimatedPose();

        if (ttarget == null || !ttarget.isPresent()) {
            robotDrive.setTrackingRotation(Double.NaN); // temporarily disable tracking
            robotDrive.clearPPRotationOverride();
            return;
        }

        if (iCount < 7 && !firstLoop) {
            currentX = (ttarget.get().estimatedPose.getX() + lastTargetX) / 2;
            currentY = (ttarget.get().estimatedPose.getY() + lastTargetY) / 2;
            iCount += 1;
        } else {
            iCount = 0;
            firstLoop = true;
        }
     
        double movementX;
        double movementY;
        double rotation;

        double toleranceX = 0.15;
        double toleranceY = 0.15;
        double toleranceRot = 1;

        if (currentX < targetRobotX - Constants.xCameraOffset - toleranceX || currentX > targetRobotX - Constants.xCameraOffset + toleranceX) {
            movementX = translationControllerX.calculate(currentX);
        } else {
            movementX = 0;
        }
        if (currentY < targetRobotY - Constants.yCameraOffset - toleranceY || currentY > targetRobotY - Constants.yCameraOffset + toleranceY) {
            movementY = translationControllerY.calculate(currentY);
        } else {
            movementY = 0;
        }
        if (robotDrive.getGyroYaw() < targetRobotRot - Math.toRadians(toleranceRot) || robotDrive.getGyroYaw() > targetRobotRot + Math.toRadians(toleranceRot)) {
            rotation = rotationController.calculate(robotDrive.getGyroYaw());
        } else {
            rotation = 0;
        }

        if (movementX == 0 && movementY == 0 && rotation == 0) {
            return;
        }

        if (alsoDrive) {
            robotDrive.driveRobotRelative(movementX, movementY, rotation);
            firstLoop = false;
            lastTargetX = currentX;
            lastTargetY = currentY;
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
