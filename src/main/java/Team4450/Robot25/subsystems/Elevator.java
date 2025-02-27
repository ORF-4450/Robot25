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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Elevator Subsystem that shouldn't be used on it's own, but rather as a part of the ElevatedManipulator Subsystem
public class Elevator extends SubsystemBase {
    
    private SparkFlex motorMain = new SparkFlex(ELEVATOR_LEFT, MotorType.kBrushless);
    private SparkFlex motorFollower = new SparkFlex(ELEVATOR_RIGHT, MotorType.kBrushless);
    private SparkFlexConfig mainConfig = new SparkFlexConfig();
    private SparkFlexConfig followerConfig = new SparkFlexConfig();

    //We use a ProfiledPIDController for acceleration and deceleration control
    private ProfiledPIDController mainPID;

    private RelativeEncoder mainEncoder;
    private RelativeEncoder followerEncoder;

    private final double TOLERANCE_ROTATIONS = 1.5;
    private final double START_ROTATIONS = 0.08 / ELEVATOR_WINCH_FACTOR; //NEEDS TO BE CHANGED TO ACTUAL VALUE

    private double targetPosition = Double.NaN; //in units of Rotations
    private boolean isManualControl = false;
    public Elevator(){
        Util.consoleLog();

        //Invert the follower motor: true
        followerConfig.follow(motorMain, true); // follow the main motor
        followerConfig.idleMode(IdleMode.kBrake);
        motorFollower.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        mainConfig.idleMode(IdleMode.kBrake);
        motorMain.configure(mainConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        mainEncoder = motorMain.getEncoder();
        followerEncoder = motorFollower.getEncoder();

        resetEncoders();

        // PID constants, but also the motion profiling constraints
        mainPID = new ProfiledPIDController(0.12, 0, 0, new Constraints(
            (1 / -ELEVATOR_WINCH_FACTOR), 8 / -ELEVATOR_WINCH_FACTOR // velocity / acceleration
        ));
        
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
        if (targetPosition < -52) 
            targetPosition = -52;
        if (targetPosition > -2) 
            targetPosition = -2; 

        //Main PID/Profile Loop which is used to control the elevator, and uses targetPosition 
        //which has units of rotations.  
        mainPID.setGoal(targetPosition);
        double nonclamped = mainPID.calculate(mainEncoder.getPosition());
        
        // Calculate the distance to the target
        double distanceToTarget = Math.abs(targetPosition - mainEncoder.getPosition());
        double slowDownThreshold = 0.2 * targetPosition; // 20% of the target position

        // Adjust motor output based on distance to target
        double motorOutput;
        
        if (distanceToTarget <= slowDownThreshold && isManualControl == false) {
            double slowDownFactor = 0.1; // Adjust this factor as needed
            motorOutput = Util.clampValue(nonclamped * slowDownFactor, 0.15);
            SmartDashboard.putString("Elevator Position Phase", "Slowing Down Elevator");
        } else {
            motorOutput = Util.clampValue(nonclamped, 0.20);
        }

        SmartDashboard.putNumber("Elevator Speed", motorOutput);
        motorMain.set(motorOutput);
    }

    /**
     * remove setpoint generation, essentially making the elevator go limp
     * and disables normal move() commands.
     */
    public void unlockPosition(){
        targetPosition = Double.NaN;
    }

    /**
     * Increase/Decrease the target position by a certain amount
     * Changed typically using the joysticks
     */
    
    public void move(double change){
        isManualControl = true;
        targetPosition -= change;
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