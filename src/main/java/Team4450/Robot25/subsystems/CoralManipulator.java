package Team4450.Robot25.subsystems;

import Team4450.Lib.Util;
import Team4450.Lib.ValveDA;
import static Team4450.Robot25.Constants.CORAL_MANIPULATOR;
import static Team4450.Robot25.Constants.CORAL_PIVOT;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralManipulator extends SubsystemBase {
    private SparkFlex coralMotor = new SparkFlex(CORAL_MANIPULATOR, MotorType.kBrushless);
    private SparkFlexConfig coralConfig = new SparkFlexConfig();

    private SparkLimitSwitch coralSensor;

    private ValveDA coralPivot = new ValveDA(CORAL_PIVOT, PneumaticsModuleType.REVPH);

    private boolean isRunning = false;
    public boolean coralPivotStatus = false;

    public CoralManipulator(){
        coralConfig.idleMode(IdleMode.kBrake);

        coralMotor.configure(coralConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        // coralSensor = coralMotor.getForwardLimitSwitch();

        Util.consoleLog("Coral Manipulator Initialized");
        }
    
    public void intialize(){
        pivotDown();

        coralPivotStatus = false;

        updateDS();
    }
    
    public boolean hasCoral(){

        return coralSensor.isPressed();

    }

    public void start(double speedfactor){

        isRunning = Math.abs(speedfactor) > 0.02;
        
        updateDS();

        coralMotor.set(Util.clampValue(speedfactor, 1));
    }

    public void startIntaking(){

        coralMotor.set(-0.5);
        isRunning = true;
        updateDS();
    }

    public void startOuttaking(){
        coralMotor.set(0.5);
        isRunning = true;
        updateDS();
    }
    public void start(){
       start(1);

       isRunning = true;
       updateDS();
    }

    public void stop(){
        Util.consoleLog();

        coralMotor.stopMotor();
        // pivotDown();

        isRunning = false;
        coralPivotStatus = false;
        updateDS();
    }

    public void pivotUp(){
        Util.consoleLog();

        coralPivot.SetA();
    
        coralPivotStatus = true;
        updateDS();

    }

    public void setCoralPivot(boolean status){
        Util.consoleLog();

        if (status == true){
            pivotUp();
            coralPivotStatus = true;
        } else{
            pivotDown();
            coralPivotStatus = false;
        }

        updateDS();
    }
    public void pivotDown(){
        Util.consoleLog();

        coralPivot.SetB();

        coralPivotStatus = false;
        updateDS();
    }

    public double getCurrent(){
        double current = coralMotor.getOutputCurrent();
        SmartDashboard.putNumber("Coral Manipulator Current", current);
        return current;
    }

    private void updateDS() {
        SmartDashboard.putBoolean("Coral Manipulator Running", isRunning);

        // SmartDashboard.putBoolean("Coral Pivot On", coralPivotStatus);
        // SmartDashboard.putBoolean("Has Coral", hasCoral());
    }
}
