package Team4450.Robot25.subsystems;

import static Team4450.Robot25.Constants.ELEVATOR_LEFT;
import static Team4450.Robot25.Constants.ELEVATOR_RIGHT;
import static Team4450.Robot25.Constants.ELEVATOR_WINCH_FACTOR;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import Team4450.Lib.Util;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Elevator Subsystem that shouldn't be used on it's own, but rather as a part of the ElevatedManipulator Subsystem
public class Elevator extends SubsystemBase {
    private SparkFlex motorFollower = new SparkFlex(ELEVATOR_LEFT, MotorType.kBrushless);
    private SparkFlex motorMain = new SparkFlex(ELEVATOR_RIGHT, MotorType.kBrushless);
    private SparkFlexConfig mainConfig = new SparkFlexConfig();
    private SparkFlexConfig followerConfig = new SparkFlexConfig();

    //We use a ProfiledPIDController for acceleration and deceleration control
    private ProfiledPIDController mainPID;
    // private ProfiledPIDController slowPID;

    private RelativeEncoder mainEncoder;
    private RelativeEncoder followerEncoder;

    private final double TOLERANCE_ROTATIONS = 1.0;
    private final double START_ROTATIONS = 0.00 / ELEVATOR_WINCH_FACTOR; //NEEDS TO BE CHANGED TO ACTUAL VALUE

    private double targetPosition = Double.NaN; //in units of Rotations
    private boolean isManualControl = false;
    private boolean isSlow = false;
    public boolean limiter = false;

    private DriveBase driveBase;

    public Elevator(DriveBase driveBase) {
        Util.consoleLog();

        this.driveBase = driveBase;

        //Invert the follower motor: true
        followerConfig.follow(motorMain, true); // follow the main motor
        followerConfig.idleMode(IdleMode.kBrake);
        motorFollower.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        mainConfig.idleMode(IdleMode.kBrake);
        motorMain.configure(mainConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        mainEncoder = motorMain.getEncoder();
        followerEncoder = motorFollower.getEncoder();

        resetEncoders();

        mainPID = new ProfiledPIDController(0.12, 0, 0, new Constraints(
                (3.25 / -ELEVATOR_WINCH_FACTOR), 6 / -ELEVATOR_WINCH_FACTOR // velocity / acceleration
            ));

        // slowPID = new ProfiledPIDController(0.12, 0, 0, new Constraints(
        //         (2.5 / -ELEVATOR_WINCH_FACTOR), 8 / -ELEVATOR_WINCH_FACTOR // velocity / acceleration
        //     ));

        // PID constants, but also the motion profiling constraints
    
        // mainPID = new ProfiledPIDController(0.12, 0, 0, new Constraints(
        //     (1.625 / -ELEVATOR_WINCH_FACTOR), 8 / -ELEVATOR_WINCH_FACTOR // velocity / acceleration
        // ));
        
        SmartDashboard.putData("winch_pid", mainPID);
        mainPID.setTolerance(TOLERANCE_ROTATIONS);
    }

    public void periodic(){
        SmartDashboard.putNumber("winch_measured", mainEncoder.getPosition());
        SmartDashboard.putNumber("winch_1_m", mainEncoder.getPosition()* ELEVATOR_WINCH_FACTOR);
        SmartDashboard.putNumber("winch_2_m", followerEncoder.getPosition()* ELEVATOR_WINCH_FACTOR);
        SmartDashboard.putNumber("winch_setpoint", targetPosition);

        if (Double.isNaN(targetPosition)) return;

        //SOFT LIMITS
        if (targetPosition < -59) 
            targetPosition = -59;
        if (targetPosition > 0) 
            targetPosition = 0; 

        
        
        // Calculate the distance to the target
        double distanceToTarget = Math.abs(targetPosition - mainEncoder.getPosition());
        double slowDownThreshold = 0.2 * targetPosition; // 20% of the target position

        // Adjust motor output based on distance to target
        double motorOutput;
        
        //Main PID/Profile Loop which is used to control the elevator, and uses targetPosition 
        //which has units of rotations. 
        // double nonclamped = 0.0; // Initialize nonclamped with a default value


        // if (targetPosition > this.getElevatorPosition()) {
        //     mainPID.setGoal(targetPosition);
        //     nonclamped = mainPID.calculate(mainEncoder.getPosition());
        // } else if (targetPosition < this.getElevatorPosition()) {
        //     slowPID.setGoal(targetPosition);
        //     nonclamped = slowPID.calculate(mainEncoder.getPosition());
        // }
        mainPID.setGoal(targetPosition);
        double nonclamped = mainPID.calculate(mainEncoder.getPosition());

        if(isSlow == true){
            motorOutput = Util.clampValue(nonclamped, 0.30);
        }

        if (distanceToTarget <= slowDownThreshold && isManualControl == false) {
            double slowDownFactor = 0.1; // Adjust this factor as needed
            motorOutput = Util.clampValue(nonclamped * slowDownFactor, 0.15);
            SmartDashboard.putString("Elevator Position Phase", "Slowing Down Elevator");
        } else {
            motorOutput = Util.clampValue(nonclamped, 0.80);
            // motorOutput = Util.clampValue(nonclamped, 0.40);

        }
        

        SmartDashboard.putNumber("Elevator Speed", motorOutput);
        motorMain.set(motorOutput);
        SmartDashboard.putNumber("Elevator Amperage", motorMain.getOutputCurrent());

        
        // if (!driveBase.slowModeEnabled && this.getElevatorHeight() == 0.99) {
        //     driveBase.speedLimiter = 0.45;
        //     driveBase.rotSpeedLimiter = 0.65;
        //     Util.consoleLog("%.2f %.2f", driveBase.speedLimiter, driveBase.rotSpeedLimiter);
        // }

        // height 0.59 L2 1 drive speed and rotation speed
        // height 0.99 L3 0.48 drive speed and 0.68 rotation speed
        // height 1.59 L4 0.2 drive speed and 0.4 rotation speed
        if(limiter == true){
            if (!driveBase.slowModeEnabled) {
            driveBase.speedLimiter = Math.pow(2, -(3.1 * this.getElevatorHeight() - 0.65));
            driveBase.rotSpeedLimiter = Math.pow(2, -(3.1 * this.getElevatorHeight() - 0.65)) + 0.2;
            if (driveBase.speedLimiter > 1) {
                driveBase.speedLimiter = 1;
            }
            if (driveBase.rotSpeedLimiter > 1) {
                driveBase.rotSpeedLimiter = 1;
            }
            if (driveBase.speedLimiter < 0.2) {
                driveBase.speedLimiter = 0.2;
            }
            if (driveBase.rotSpeedLimiter < 0.4) {
                driveBase.rotSpeedLimiter = 0.4;
            }
        }
    }
    }

    /**
     * remove setpoint generation, essentially making the elevator go limp
     * and disables normal move() commands.
     */
    public void unlockPosition(){
        targetPosition = Double.NaN;
    }
    
    public double getElevatorPosition(){
        double position = mainEncoder.getPosition();
        SmartDashboard.putNumber("Elevator Position", position);
        return position;
    }

    /**
     * Increase/Decrease the target position by a certain amount
     * Changed typically using the joysticks
     */
    
    public void move(double change){
        isManualControl = true;
        targetPosition -= change;
    }

    public void moveSlowToHeight(double height){
        isSlow = true;
        targetPosition = height/ELEVATOR_WINCH_FACTOR;
    }
    /**
     * Bypass all setpoint generation and just run direct motor power. This
     * also bypasses all soft limits, so use EXTREME CAUTION! Remember, the elevator
     * is capable of TEARING OFF LIMBS AND DESTROYING ITSELF if not used properly. 
     * Should never use this unless you know what you're doing.
     */

    public void moveUnsafe(double speed){
        targetPosition = Double.NaN;
        motorMain.set(speed);  
        SmartDashboard.putNumber("moveUnsafe Speed", speed);
    }
    
    //Sets the target position of the elevator in meters, which is converted to rotations
    public void setElevatorHeight(double height){
        driveBase.setElevatorHeightSpeed(height);
        targetPosition = height/ELEVATOR_WINCH_FACTOR; //meters to rotations
    }

    //Checks if the elevator is at the target height by converting given height to rotations: returns rotations
    public boolean isElevatorAtTarget(double height){
        double setpoint = height/ELEVATOR_WINCH_FACTOR;
        return Math.abs(setpoint - mainEncoder.getPosition()) < TOLERANCE_ROTATIONS;
    }

    //Gets the height of the elevator in meters by converting rotations to meters: returns meters
    public double getElevatorHeight(){
        double elevatorHeight = mainEncoder.getPosition() * ELEVATOR_WINCH_FACTOR;
        return elevatorHeight;
    }

    //Resets the encoders to the current position, which sets it to the "zero/starting" position
    public void resetEncoders(){
        mainEncoder.setPosition(START_ROTATIONS);
        followerEncoder.setPosition(START_ROTATIONS);
        targetPosition = START_ROTATIONS;
    }

    //Sets the setpoint to current position which locks the Elevator in place.
    public void lockPosition(){
        targetPosition = mainEncoder.getPosition();
        mainPID.reset(targetPosition);
    }

}
