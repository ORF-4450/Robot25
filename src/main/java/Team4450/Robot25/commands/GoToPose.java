package Team4450.Robot25.commands;

import Team4450.Lib.Util;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import Team4450.Robot25.subsystems.DriveBase;

/**
   * This function uses the current robot position estimate that is build from odometry and apriltags to go to a location on the field.
   * Will return imidiatly when the location is reached.
   *
   * 
   * @return Void
   */

public class GoToPose extends Command {
    PIDController rotationController = new PIDController(0.02, 0, 0); // for rotating drivebase
    PIDController translationControllerX = new PIDController(0.1, 0.1, 0); // for moving drivebase in X,Y plane
    PIDController translationControllerY = new PIDController(0.1, 0.1, 0); // for moving drivebase in X,Y plane
    DriveBase robotDrive;
    private boolean alsoDrive;
    private boolean initialFieldRel;
    private boolean isFinished;
    private double toleranceX = 0.05;
    private double toleranceY = 0.05;
    private double toleranceRot = 1;
    /**
     * @param robotDrive the drive subsystem
     */

    public GoToPose (DriveBase robotDrive, boolean alsoDrive, boolean initialFieldRel) {
        this.robotDrive = robotDrive;
        this.alsoDrive = alsoDrive;
        isFinished = false;

        if (alsoDrive) addRequirements(robotDrive);

        SendableRegistry.addLW(translationControllerX, "GoToPose Translation PID");
        SendableRegistry.addLW(translationControllerY, "GoToPose Translation PID");
        SendableRegistry.addLW(rotationController, "GoToPose Rotation PID");
    }

    public void initialize () {
        Util.consoleLog();

        

        // store the initial field relative state to reset it later.
        initialFieldRel = robotDrive.getFieldRelative();
        
        if(initialFieldRel)
            robotDrive.toggleFieldRelative();
        robotDrive.enableTracking();
        robotDrive.enableTrackingSlowMode();
        
        rotationController.setSetpoint(robotDrive.getTargetPose().getRotation().getRadians());
        rotationController.setTolerance(toleranceRot);

        // translationControllerX.setSetpoint(-15); // target should be at -15 pitch
        translationControllerX.setSetpoint(robotDrive.getTargetPose().getX());
        translationControllerX.setTolerance(toleranceX);

        translationControllerY.setSetpoint(robotDrive.getTargetPose().getY());
        translationControllerY.setTolerance(toleranceY);

        SmartDashboard.putString("GoToPose", "Tag Tracking Initialized");
    }

    @Override
    public void execute() {
        if (isFinished()) {
            end(false);
            return;
        }

        if (robotDrive.getTargetPose().getX() == 0 || robotDrive.getTargetPose().getY() == 0) {
            // Smartdashboard warning on target assignment (Upgrade)
            Util.consoleLog("NO TARGET ASSIGNED");
            return;
        }
        
        if (robotDrive.getTargetPose() == null || (robotDrive.getTargetPose().getX() == 0 && robotDrive.getTargetPose().getY() == 0)) {
            robotDrive.setTrackingRotation(Double.NaN); // temporarily disable tracking
            robotDrive.clearPPRotationOverride();
            return;
        }

        double movementX;
        double movementY;
        double rotation;

        Util.consoleLog(robotDrive.getTargetPose().toString());
        Util.consoleLog(String.valueOf(robotDrive.getPose().getX()));

        if (robotDrive.getPose().getX() < robotDrive.getTargetPose().getX() - toleranceX || robotDrive.getPose().getX() > robotDrive.getTargetPose().getX() + toleranceX) {
            movementX = translationControllerX.calculate(robotDrive.getPose().getX());
        } else {
            movementX = 0;
        }
        if (robotDrive.getPose().getY() < robotDrive.getTargetPose().getY() - toleranceY || robotDrive.getPose().getY() > robotDrive.getTargetPose().getY() + toleranceY) {
            movementY = translationControllerY.calculate(robotDrive.getPose().getY());
        } else {
            movementY = 0;
        }
        if (robotDrive.getGyroYaw() < robotDrive.getTargetPose().getRotation().getRadians() - Math.toRadians(toleranceRot) || robotDrive.getGyroYaw() > robotDrive.getTargetPose().getRotation().getRadians() + Math.toRadians(toleranceRot)) {
            rotation = rotationController.calculate(robotDrive.getGyroYaw());
        } else {
            rotation = 0;
        }

        if (movementX == 0 && movementY == 0 && rotation == 0) {
            isFinished = true;
            end(false);
            return;
        }

        if (alsoDrive) {
            robotDrive.driveRobotRelative(movementX, movementY, rotation);
        } else {
            robotDrive.setTrackingRotation(0);
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
        Util.consoleLog();

        robotDrive.setTargetPose(new Pose2d(0, 0, new Rotation2d(0)));
        
        if (alsoDrive) robotDrive.drive(0, 0, 0, false);
        
        if (initialFieldRel) robotDrive.toggleFieldRelative(); // restore beginning state
        
        robotDrive.setTrackingRotation(Double.NaN);
        robotDrive.disableTracking();
        robotDrive.disableTrackingSlowMode();
        robotDrive.clearPPRotationOverride();

        SmartDashboard.putString("GoToPose", "Tag Tracking Ended");

    }
}
