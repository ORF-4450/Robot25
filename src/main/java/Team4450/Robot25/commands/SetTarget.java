package Team4450.Robot25.commands;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import Team4450.Robot25.subsystems.DriveBase;
import Team4450.Robot25.subsystems.PhotonVision;


/**
 * This command points the robot to the yaw value from the AprilTag,
 * overriding the controller joystick input and allowing
 * rotation to be commanded seperately from translation.
 */

public class SetTarget extends Command {
    DriveBase robotDrive;
    private PhotonVision photonVision;
    private boolean finished;
    /**
     * @param robotDrive the drive subsystem
     */

    public SetTarget (DriveBase robotDrive, PhotonVision photonVision) {
        this.robotDrive = robotDrive;
        this.photonVision = photonVision;

    }

    public void initialize() {
        SmartDashboard.putString("SetTargetPose", "Started");
    }

    @Override
    public void execute() {
        if (!finished) {
        robotDrive.setTargetPitch(photonVision.getClosestTarget().getPitch());
        robotDrive.setTargetYaw(photonVision.getClosestTarget().getYaw());
        finished = true;
        }
        return;
    }

    public boolean isFinished() {
        if (finished) {
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        finished = false;
        Util.consoleLog("interrupted=%b", interrupted);

        SmartDashboard.putString("Set Target", "Ended");

    }
}
