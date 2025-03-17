package Team4450.Robot25.subsystems;

import static Team4450.Robot25.Constants.ALGAE_GROUND;
import static Team4450.Robot25.Constants.ALGAE_GROUND_INTAKE;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import Team4450.Lib.Util;
import Team4450.Lib.ValveDA;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeGroundIntake extends SubsystemBase {
    private SparkFlex algaeGroundMotor = new SparkFlex(ALGAE_GROUND_INTAKE, MotorType.kBrushless);
    private SparkFlexConfig algaeGroundConfig = new SparkFlexConfig();

    private ValveDA algaeGroundPiston = new ValveDA(ALGAE_GROUND, PneumaticsModuleType.REVPH);

    private boolean isRunning = false;
    public boolean algaeGroundPistonStatus = false;

    public AlgaeGroundIntake(){
        algaeGroundConfig.idleMode(IdleMode.kBrake);

        algaeGroundMotor.configure(algaeGroundConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        Util.consoleLog("Algae Ground Intake Initialized");
        }
    
    public void intialize(){

        retractIn();

        algaeGroundPistonStatus = false;
        updateDS();
    }

    public void start(double speedfactor){
        isRunning = Math.abs(speedfactor) > 0.02;
        
        updateDS();

        algaeGroundMotor.set(Util.clampValue(speedfactor, 1));
    }

    public void startRollers(){
        isRunning = true;
        algaeGroundMotor.set(1);
        updateDS();
    }

    public void start(){
        start(1);
        isRunning = true;
        updateDS();
     }

    public void stop(){
        algaeGroundMotor.set(0);
        isRunning = false;
        updateDS();
    }
    public void extendOut(){
        Util.consoleLog();

        algaeGroundPiston.SetB();

        algaeGroundPistonStatus = true;

        updateDS();
    }

    public void retractIn(){
        Util.consoleLog();

        algaeGroundPiston.SetA();

        algaeGroundPistonStatus = false;

        updateDS();
    }

    public void setAlgaeGroundExtend(boolean status){
        Util.consoleLog();

        if (status == true){
            extendOut();
            algaeGroundPistonStatus = true;
        } else if (status == false){
            retractIn();
            algaeGroundPistonStatus = false;
        }
        updateDS();
    }   

    public void updateDS(){
        SmartDashboard.putBoolean("Algae Ground Intake Running", isRunning);
        SmartDashboard.putBoolean("Algae Ground Intake Piston Status", algaeGroundPistonStatus);
    }
}
