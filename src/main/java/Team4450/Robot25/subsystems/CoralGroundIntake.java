// package Team4450.Robot25.subsystems;

// import static Team4450.Robot25.Constants.CORAL_GROUND_PIVOT;
// import static Team4450.Robot25.Constants.CORAL_GROUND_INTAKE;
// import static Team4450.Robot25.Constants.CORAL_GROUND_FEED;
// import static Team4450.Robot25.Constants.CORAL_GROUND_PIVOT_FACTOR;

// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkLimitSwitch;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.LimitSwitchConfig.Type;
// import com.revrobotics.spark.SparkLowLevel.MotorType;

// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.wpilibj.RobotBase;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class CoralGroundIntake extends SubsystemBase {
    
//     private SparkMax motorPivot = new SparkMax(CORAL_GROUND_PIVOT, MotorType.kBrushless);
//     private TalonFX motorIntake = new TalonFX(CORAL_GROUND_INTAKE);
//     private TalonFX motorFeed = new TalonFX(CORAL_GROUND_FEED);

//     private RelativeEncoder pivotEncoder;
//     private ProfiledPIDController pivotPID;

//     private final double PIVOT_TOLERANCE = 1;
//     private final double PIVOT_START = 90;

//     private double goal = PIVOT_START;

//     private final double pivotFactor = CORAL_GROUND_PIVOT_FACTOR;

//     public CoralGroundIntake(){
//         pivotEncoder = motorPivot.getEncoder();

//         pivotPID = new ProfiledPIDController(0.12, 0, 0,
//             new Constraints(angleToEncoderCounts(360 *1), angleToEncoderCounts(4*360)) // max velocity(/s), max accel(/s)
//         );

//         pivotPID.setTolerance(PIVOT_TOLERANCE); // encoder counts not degrees for this one

        

//     }


//     public void startIntake(){
//         setAngle(0);
//         motorIntake.set(1);
        
//     }

//     public void feedToManipulator(){
//         motorFeed.set(1);
//     }

//     /**
//      * Sets the intake assembly to a given angle
//      * @param angle the angle in degrees
//      */
//     public void setAngle(double angle) {
//         goal = angle;
//     }

//     public double getAngle() {
//         double roughAngle = pivotEncoder.getPosition() * pivotFactor; // convert to degrees
//         return roughAngle;
//     }

//     /**
//      * remove setpoint control, causing pivot to become limp and
//      * react to external forces with no braking or anything.
//      */
//     public void unlockPosition() {
//         goal = Double.NaN; // when setpoint NaN it doesn't do it
//     }


//     /**
//      * set the current setpoint/goal to the current position,
//      * essentially "locking" the pivot in place
//      */
//     public void lockPosition() {
//         goal = getAngle();
//         pivotPID.reset(angleToEncoderCounts(goal));
//     }

//      /**
//      * given an angle, return the encoder counts
//      * @param angle angle of ground intake position: 0 is nominal angle in degrees
//      * @return the raw encoder position
//      */
//     private double angleToEncoderCounts(double angle) {
//         return angle / pivotFactor;
//     }
// }
