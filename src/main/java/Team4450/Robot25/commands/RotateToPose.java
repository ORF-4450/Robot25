// package Team4450.Robot25.commands;

// import Team4450.Lib.Util;
// import Team4450.Robot25.subsystems.DriveBase;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;

// /**
//    * This function uses the current robot position estimate that is build from odometry and apriltags to go to a location on the field.
//    * Will return imidiatly when the location is reached.
//    *
//    * 
//    * @return Void
//    */

// public class RotateToPose extends Command {
//     DriveBase robotDrive;
//     private boolean alsoDrive;
//     private boolean initialFieldRel;
//     private boolean isFinished;
//     private double toleranceRot = 4;
//     private double sign;
//     /**
//      * @param robotDrive the drive subsystem
//      */

//     public RotateToPose (DriveBase robotDrive, boolean alsoDrive, boolean initialFieldRel) {
//         this.robotDrive = robotDrive;
//         this.alsoDrive = alsoDrive;
//         isFinished = false;
//         sign = 0;

//         if (alsoDrive) addRequirements(robotDrive);
//     }

//     public void initialize () {
//         Util.consoleLog("Init");
//         isFinished = false;
//         Util.consoleLog();

//         // store the initial field relative state to reset it later.
//         initialFieldRel = robotDrive.getFieldRelative();
        
//         if(initialFieldRel)
//             robotDrive.toggleFieldRelative();
//         robotDrive.enableTracking();
//         robotDrive.enableTrackingSlowMode();

//         // double netRotation = robotDrive.getHeading() - robotDrive.getTargetPose().getRotation().getDegrees();
//         // if (netRotation < 0) {
//         //     netRotation = netRotation + 360;
//         // }
//         // if (netRotation > 180) {
//         //     sign = -1;
//         // } else {
//         //     sign = 1;
//         // }
//         if (robotDrive.getHeading() > robotDrive.getTargetPose().getRotation().getDegrees() && Math.abs(robotDrive.getHeading() - robotDrive.getTargetPose().getRotation().getDegrees()) < 180) {
//             sign = 1;
//         } else if (robotDrive.getHeading() > robotDrive.getTargetPose().getRotation().getDegrees() && Math.abs(robotDrive.getHeading() - robotDrive.getTargetPose().getRotation().getDegrees()) > 180) {
//             sign = -1;
//         } else if (robotDrive.getHeading() < robotDrive.getTargetPose().getRotation().getDegrees() && Math.abs(robotDrive.getHeading() - robotDrive.getTargetPose().getRotation().getDegrees()) < 180) {
//             sign = 1;
//         } else if (robotDrive.getHeading() < robotDrive.getTargetPose().getRotation().getDegrees() && Math.abs(robotDrive.getHeading() - robotDrive.getTargetPose().getRotation().getDegrees()) > 180) {
//             sign = -1;
//         }
                
//         SmartDashboard.putString("RotateToPose", "Tag Tracking Initialized");
//     }

//     @Override
//     public void execute() {
//         if (robotDrive.getRotatedToTargetPose()) {
//             end(true);
//         }
//         if (isFinished()) {
//             end(false);
//             return;
//         }

//         if (robotDrive.getTargetPose().getX() == 0 && robotDrive.getTargetPose().getY() == 0) {
//             isFinished = true;
//         }

//         double rotation = 0;
//         if (robotDrive.getHeading() > robotDrive.getTargetPose().getRotation().getDegrees() + toleranceRot) {
//             rotation = 0.50;
//         } else if (robotDrive.getHeading() < robotDrive.getTargetPose().getRotation().getDegrees() - toleranceRot) {
//             rotation = -0.50;
//         } else {
//             rotation = 0;
//             robotDrive.setRotatedToTargetPose(true);
//             isFinished = true;
//         }
//         rotation = rotation * sign;
//         if (alsoDrive) {
//             robotDrive.driveFieldRelative(0, 0, rotation);
//         } else {
//             robotDrive.setTrackingRotation(0);
//         }
//     }

//     public boolean isFinished() {
//         if (isFinished) {
//             return true;
//         }
//         return false;
//     }

//     @Override
//     public void end(boolean interrupted) {
//         Util.consoleLog("interrupted=%b", interrupted);
//         Util.consoleLog();
        
//         if (alsoDrive) robotDrive.drive(0, 0, 0, false);
        
//         if (initialFieldRel) robotDrive.toggleFieldRelative(); // restore beginning state
        
//         robotDrive.setTrackingRotation(Double.NaN);
//         robotDrive.disableTracking();
//         robotDrive.disableTrackingSlowMode();
//         robotDrive.clearPPRotationOverride();

//         SmartDashboard.putString("RotateToPose", "Tag Tracking Ended");

//     }
// }
