package Team4450.Robot25.commands;

import Team4450.Lib.Util;
import Team4450.Robot25.subsystems.DriveBase;
import Team4450.Robot25.subsystems.PhotonVision;
import Team4450.Robot25.utility.AprilTagMap;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This function uses the current robot position estimate that is built from odometry and apriltags to go to a location on the field.
 * Will return immediately when the location is reached.
 *
 * @return Void
 */
public class GoToTag extends Command {
    PIDController translationControllerX = new PIDController(0.35, 0, 0); // for moving drivebase in X,Y plane
    PIDController translationControllerY = new PIDController(0.35, 0, 0); // for moving drivebase in X,Y plane
    DriveBase robotDrive;
    PhotonVision photonVision;
    private boolean alsoDrive;
    private boolean initialFieldRel;
    private boolean isFinished;
    private double toleranceX = 0.01;
    private double toleranceY = 0.01;
    private Pose2d targetPose;

    /**
     * @param robotDrive the drive subsystem
     */
    public GoToTag(DriveBase robotDrive, boolean alsoDrive, boolean initialFieldRel, PhotonVision photonVision) {
        this.robotDrive = robotDrive;
        this.alsoDrive = alsoDrive;
        this.photonVision = photonVision;
        isFinished = false;

        if (alsoDrive) addRequirements(robotDrive);

        SendableRegistry.addLW(translationControllerX, "GoToTag Translation PID");
        SendableRegistry.addLW(translationControllerY, "GoToTag Translation PID");
    }

    public void initialize() {
        Util.consoleLog();
        isFinished = false;

        // store the initial field relative state to reset it later.
        initialFieldRel = robotDrive.getFieldRelative();

        if (initialFieldRel)
            robotDrive.toggleFieldRelative();
        robotDrive.enableTracking();
        robotDrive.enableTrackingSlowMode();

        SmartDashboard.putString("GoToTag", "Tag Tracking Initialized");

        // int targetID = photonVision.getClosestTarget().getFiducialId();
        targetPose = robotDrive.getTargetPose();
    }   

    @Override
    public void execute() {
        Pose2d currentPose = robotDrive.getPose();
        double errorX = targetPose.getX() - currentPose.getX();
        double errorY = targetPose.getY() - currentPose.getY();

        // double outputX = translationControllerX.calculate(currentPose.getX(), targetPose.getX());
        // double outputY = translationControllerY.calculate(currentPose.getY(), targetPose.getY());

        double outputX = targetPose.getX() - currentPose.getX();
        double outputY = targetPose.getY() - currentPose.getY();
        
        SmartDashboard.putNumber("Output X Speed", outputX);
        SmartDashboard.putNumber("Output Y Speed", outputY);

        if(outputX > 0.5) outputX = 0.5;
        if(outputY > 0.5) outputY = 0.5;
        if(outputX < -0.5) outputX = -0.5;
        if(outputY < -0.5) outputY = -0.5;

        

        if(robotDrive.getRotatedToTargetPose() == true){
            robotDrive.drive(outputX, outputY, 0, false);
        }

        if (Math.abs(errorX) < toleranceX && Math.abs(errorY) < toleranceY) {
            isFinished = true;
        }
    }

    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog();
        robotDrive.drive(0, 0, 0, false);
        robotDrive.disableTracking();
        robotDrive.disableTrackingSlowMode();
        if (initialFieldRel)
            robotDrive.toggleFieldRelative();
    }
}
