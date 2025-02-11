
package Team4450.Robot25;

import java.util.HashMap;
import java.util.Properties;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
	public static String		PROGRAM_NAME = "ORF25-01.30.25VR";

	public static Robot			robot;

	public static Properties	robotProperties;
	  
	public static boolean		isClone = false, isComp = false, tracing = false;
	    	
	public static DriverStation.Alliance	 alliance;
	public static int                        location, matchNumber;
	public static String					 eventName, gameMessage;
	    
    public static String                     functionMarker = "-".repeat(30);

	// Non-drive base motor controller port assignments
    public static final int     CORAL_MANIPULATOR = 9;
    public static final int     ALGAE_MANIPULATOR = 10;

    //ELEVATOR:
    public static final int     ELEVATOR_LEFT = 11;
    public static final int     ELEVATOR_RIGHT = 12;

    //(NOTES) ELEVATOR_WINCH_FACTOR is a conversion factor from motor rotations to meters of height change.
    //ELEVATOR_WINCH_FACTOR is multiplied by native rotations of motor shaft 
    // to get height change in MAXSpline shaft since startup or last encoder reset.
    // math explanation:
    // ratio is (1.0 / (1014.0 / 55.0)) spool rots for every turn of shaft
    // * 2pi for radians traveled/angular displacement * spool radius in meters to get linear displacement
    // 1.25 inch radius is 0.03175 meters (source: looked it up)
    // idk why it has to be negative, probably the gears swap rotation, not a big deal tho
    public static final double  ELEVATOR_WINCH_FACTOR = (-1.0 / (1014.0 / 55.0)) * (2 * Math.PI) * 0.03175; //NEEDS TO BE CHANGED TO ACTUAL VALUE

    // Pneumatic valve controller port assignments.
	public static final int		COMPRESSOR = 0;
	public static final int		CORAL_PIVOT = 0;		
	public static final int		ALGAE_EXTEND = 2;		
	public static final int		ALGAE_PIVOT = 4;    

    
    public static final double INTAKE_SPEED = 0.90;
    
    // CAMERAS 

    public static Transform3d   CAMERA_TAG_TRANSFORM = new Transform3d(
        new Translation3d(0, 0.32, 0.28), // change last value to height in METERS of lens
        new Rotation3d(0, 0, Math.toRadians(180)) // keep the 180, the -10 is the camera angle (negative!)
    );

    // Find positions
    // map of positions for the robot to go to based on which april tag it sees
    public static HashMap<Integer, Pose2d> aprilTagToPoseMap = new HashMap<>(); static {
        // april id, location to go to
        // red side
        // aprilTagToPoseMap.put(1, new Pose2d(16.7, 0.66, new Rotation2d(Math.toRadians(126))));
        // aprilTagToPoseMap.put(2, new Pose2d(16.7, 7.40, new Rotation2d(Math.toRadians(-126))));
        // aprilTagToPoseMap.put(3, new Pose2d(11.56, 8.06, new Rotation2d(Math.toRadians(-90))));
        // aprilTagToPoseMap.put(4, new Pose2d(9.28, 6.14, new Rotation2d(Math.toRadians(0))));
        // aprilTagToPoseMap.put(5, new Pose2d(9.28, 1.91, new Rotation2d(Math.toRadians(0))));
        aprilTagToPoseMap.put(6, new Pose2d(13.47, 3.31, new Rotation2d(Math.toRadians(-60))));
        aprilTagToPoseMap.put(7, new Pose2d(13.89, 4.03, new Rotation2d(Math.toRadians(0))));
        aprilTagToPoseMap.put(8, new Pose2d(13.43, 4.75, new Rotation2d(Math.toRadians(60))));
        aprilTagToPoseMap.put(9, new Pose2d(12.64, 4.75, new Rotation2d(Math.toRadians(120))));
        aprilTagToPoseMap.put(10, new Pose2d(12.23, 4.03, new Rotation2d(Math.toRadians(180))));
        aprilTagToPoseMap.put(11, new Pose2d(12.64, 3.31, new Rotation2d(Math.toRadians(-120))));
        // blue side 
        // aprilTagToPoseMap.put(12, new Pose2d(0.85, 0.66, new Rotation2d(Math.toRadians(54))));
        // aprilTagToPoseMap.put(13, new Pose2d(0.85, 7.40, new Rotation2d(Math.toRadians(-54))));
        // aprilTagToPoseMap.put(14, new Pose2d(8.27, 6.14, new Rotation2d(Math.toRadians(180))));
        // aprilTagToPoseMap.put(15, new Pose2d(8.27, 1.91, new Rotation2d(Math.toRadians(180))));
        // aprilTagToPoseMap.put(16, new Pose2d(5.99, 0.00, new Rotation2d(Math.toRadians(90))));
        aprilTagToPoseMap.put(17, new Pose2d(4.07, 3.31, new Rotation2d(Math.toRadians(-120))));
        aprilTagToPoseMap.put(18, new Pose2d(3.66, 4.03, new Rotation2d(Math.toRadians(180))));
        aprilTagToPoseMap.put(19, new Pose2d(4.07, 4.75, new Rotation2d(Math.toRadians(120))));
        aprilTagToPoseMap.put(20, new Pose2d(4.90, 4.75, new Rotation2d(Math.toRadians(60))));
        aprilTagToPoseMap.put(21, new Pose2d(5.32, 4.03, new Rotation2d(Math.toRadians(0))));
        aprilTagToPoseMap.put(22, new Pose2d(4.90, 3.31, new Rotation2d(Math.toRadians(-60))));
    }

    public static double robotCoralLongitudinalScoringDistance = 0.3; // 0.3 meters distance from the tag for scoring coral.
    public static double robotCoralLateralScoringOffset = 0.2; // Added to the target position if scoring left and subtracted if scoring right.

    // Find offsets

    public static double xCameraOffset = 0;
    public static double yCameraOffset = 0;

    // the names of the cameras in the PhotonVision software
    public static String        CAMERA_TAG = "HD_USB_Camera";

    public static final int     REV_PDB = 20;
    public static final int     CTRE_CANDLE = 21;
	
	// GamePad port assignments.
	public static final int		DRIVER_PAD = 0, UTILITY_PAD = 1;
    public static final double  DRIVE_DEADBAND = 0.1, ROTATION_DEADBAND = .1;

	// Pneumatic valve controller port assignments.
	//public static final int		COMPRESSOR = 0;
	  
	// Analog Input port assignments.
	
	// LCD display line number constants showing class where the line is set.
	public static final int		LCD_1 = 1;	    // Robot, Auto Commands.
	public static final int		LCD_2 = 2;	    // Swerve Drive command.
	public static final int		LCD_3 = 3;	    // ShuffleBoard subsystem.
	public static final int		LCD_4 = 4;	    // ShuffleBoard subsystem.
	public static final int		LCD_5 = 5;	    // Autonomous commands.
	public static final int		LCD_6 = 6;	    // ShuffleBoard subsystem.
	public static final int		LCD_7 = 7;	    // ShuffleBoard subsystem.
	public static final int		LCD_8 = 8;	    // ShuffleBoard subsystem.
	public static final int		LCD_9 = 9;	    // ShuffleBoard subsystem.
	public static final int		LCD_10 = 10;	// ShuffleBoard subsystem.

    public static final class DriveConstants {
        // Driving Parameters - These are the maximum capable speeds of
        // the robot.

        public static final double kMaxSpeedMetersPerSecond = 4.92;  // 1.0; Speed limited for demos.
        //public static final double kMaxSpeedMetersPerSecond = ModuleConstants.kDriveWheelFreeSpeedRps; // max speed
        public static final double kMaxAngularSpeed = 1.5 * (2 * Math.PI); // radians per second (1.5 rots / sec)
        public static final double kSlowModeFactor = .30; // 50% of normal.
        public static final double kRotSlowModeFactor = .20; // 20% of normal.
        
        //TrackingMode Speed:
        public static final double kTrackingModeFactor = 0.01;
        public static final double kRotTrackingModeFactor = 0.20;

        // these were 1.2, 1.8, 2.0 in REV base code. Controls drivebase slew limiting.
        public static final double kDirectionSlewRate = Double.POSITIVE_INFINITY; // radians per second.
        public static final double kMagnitudeSlewRate = 1; // percent per second (1 = 100%).
        public static final double kRotationalSlewRate = Double.POSITIVE_INFINITY; // percent per second (1 = 100%).

        // Chassis configuration:

        // Distance between centers of right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(23.5);

        // Distance between front and back wheel centers
        public static final double kWheelBase = Units.inchesToMeters(23.5);

        // Drive base radius in meters. Distance from robot center to furthest module.
        public static final double kDriveBaseRadius = .42;

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        // Angular offsets of the modules relative to the chassis in radians at
        // alignment/start up.
        public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
        public static final double kFrontRightChassisAngularOffset = 0;
        public static final double kBackLeftChassisAngularOffset = Math.PI;
        public static final double kBackRightChassisAngularOffset = Math.PI / 2;

        // SPARK MAX CAN IDs
        public static final int kFrontLeftDrivingCanId = 1;
        public static final int kFrontLeftTurningCanId = 2;

        public static final int kFrontRightDrivingCanId = 3;
        public static final int kFrontRightTurningCanId = 4;

        public static final int kRearLeftDrivingCanId = 5;
        public static final int kRearLeftTurningCanId = 6;

        public static final int kRearRightDrivingCanId = 7;
        public static final int kRearRightTurningCanId = 8;

        public static final boolean kGyroReversed = false;

        // Default starting field position in meters for pose tracking. 2024 field. 
        // public static final Pose2d	DEFAULT_STARTING_POSE = new Pose2d(.697, 7.153, Rotation2d.fromDegrees(0));
        public static final Pose2d	DEFAULT_STARTING_POSE = new Pose2d(7.473, .559, Rotation2d.fromDegrees(0));
        //public static final Pose2d	DEFAULT_STARTING_POSE = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    }

    public static final class ModuleConstants {
        // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
        // This changes the drive speed of the module (a pinion gear with more teeth will result in a
        // robot that drives faster).
        public static final int kDrivingMotorPinionTeeth = 12;

        // Invert the turning encoder, since the output shaft rotates in the opposite direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTurningEncoderInverted = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kVortexFreeSpeedRpm / 60;
        public static final double kWheelDiameterMeters = 0.0762;
        public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
        public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
        public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
            / kDrivingMotorReduction;

        public static final double kDrivingEncoderPositionFactor = (kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction; // meters

        public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
            / kDrivingMotorReduction) / 60.0; // meters per second

        public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
        public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
        public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

        public static final double kDrivingP = 0.04;
        public static final double kDrivingI = 0;
        public static final double kDrivingD = 1.0; // High to mitigate rotational drift.
        public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
        public static final double kDrivingMinOutput = -1;
        public static final double kDrivingMaxOutput = 1;

        public static final double kTurningP = 1;
        public static final double kTurningI = 0;
        public static final double kTurningD = 0;
        public static final double kTurningFF = 0;
        public static final double kTurningMinOutput = -1;
        public static final double kTurningMaxOutput = 1;

        public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
        public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

        public static final int kDrivingMotorCurrentLimit = 50; // amps
        public static final int kTurningMotorCurrentLimit = 20; // amps
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 4.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        public static final double kHolonomicPathFollowerP = 5.0;
        
        // Constraint for the motion profiled robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class NeoMotorConstants {
        public static final double kNeoFreeSpeedRpm = 5676;
        public static final double kVortexFreeSpeedRpm = 6784;
    }

  //-------------------- No student code above this line ------------------------------------------------------

}
