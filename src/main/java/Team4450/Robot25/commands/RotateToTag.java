package Team4450.Robot25.commands;

import static Team4450.Robot25.Constants.robot;

import java.util.Map;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import Team4450.Lib.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import Team4450.Robot25.AdvantageScope;
import Team4450.Robot25.Robot;
import Team4450.Robot25.subsystems.DriveBase;
import Team4450.Robot25.subsystems.PhotonVision;

/**
 * This class runs as the default command of a PhotonVision
 * subsystem and regularly updates the SwerveDrivePoseEstimator
 * object with timestamped vision poses. The pose estimator then
 * merges these observed poses with what the actual odometry on
 * the robot is doing and produces a smoothed "true" pose.
 * 
 * Also, this class updates the visionSim object with the robots
 * pose so that the visionSim can accurately calculate targets
 * in the simulator.
 */
public class RotateToTag extends Command {
    private DriveBase robotDrive;
    private double targetAngle;
    private Rotation2d targetRotation2D;
    private PhotonVision photonVision;
    private PIDController pidController = new PIDController(0.02, 0.0, 0);

    private static final Map<Integer, Double> tagIDToAngle = Map.ofEntries(
            Map.entry(21, 0.0),
            Map.entry(7, 0.0),
            Map.entry(22, 60.0),
            Map.entry(6, 60.0),
            Map.entry(17, 120.0),
            Map.entry(11, 120.0),
            Map.entry(18, 180.0),
            Map.entry(10, 180.0),
            Map.entry(19, 240.0),
            Map.entry(9, 240.0),
            Map.entry(20, 300.0),
            Map.entry(8, 300.0));

    /**
     * @param photonVision the PhotonVision subsystem in use
     * @param robotDrive the drive base
     */
    public RotateToTag(PhotonVision photonVision, DriveBase robotDrive) {
        this.photonVision = photonVision;
        this.robotDrive = robotDrive;

        // require camera subsystem because defaultcommand
        addRequirements(photonVision, robotDrive);
    }
    
    @Override
    public void initialize() {
        Util.consoleLog();
        pidController.reset();
        pidController.setTolerance(1);
        pidController.enableContinuousInput(-180, 180);
        robotDrive.enableTracking();

        targetAngle = robotDrive.getAngle();

        int targetID = photonVision.getClosestTarget().getFiducialId();

        if(photonVision.hasTargets() && tagIDToAngle.containsKey(targetID)) {
            targetAngle = tagIDToAngle.get(targetID);
        }
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == Alliance.Red) {
            if (targetAngle <= 0.0) {
                targetAngle = targetAngle + 180.0;
            } else {
                targetAngle = targetAngle - 180.0;
            }
          }
        }

    @Override
    public void execute() {
        double currentAngle = robotDrive.getAngle(); //coral side
        double output = pidController.calculate(currentAngle, targetAngle);
        robotDrive.drive(0, 0, output, false);
    }

    @Override
    public boolean isFinished() {
        return pidController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog();
        robotDrive.drive(0, 0, 0, false);
        robotDrive.disableTracking();
    }
}
