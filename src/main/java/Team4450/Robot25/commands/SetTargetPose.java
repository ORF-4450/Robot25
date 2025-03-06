package Team4450.Robot25.commands;

import Team4450.Lib.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import Team4450.Robot25.subsystems.DriveBase;


/**
 * This command points the robot to the yaw value from the AprilTag,
 * overriding the controller joystick input and allowing
 * rotation to be commanded seperately from translation.
 */

public class SetTargetPose extends Command {
    DriveBase robotDrive;
    private Pose2d targetPose;
    private boolean finished;
    /**
     * @param robotDrive the drive subsystem
     */

    public SetTargetPose (DriveBase robotDrive, Pose2d targetPose) {
        this.robotDrive = robotDrive;
        this.targetPose = targetPose;

    }

    public void initialize() {
        SmartDashboard.putString("SetTargetPose", "Started");
    }

    @Override
    public void execute() {
        if (!finished) {
        robotDrive.setTargetPose(targetPose);
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

        SmartDashboard.putString("SetTargetPose", "Ended");

    }
}
