package Team4450.Robot25.subsystems;

import Team4450.Lib.Util;
import Team4450.Lib.ValveDA;
import static Team4450.Robot25.Constants.ALGAE_MANIPULATOR;
import static Team4450.Robot25.Constants.ALGAE_PIVOT;
import static Team4450.Robot25.Constants.ALGAE_EXTEND;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeManipulator extends SubsystemBase {
    private SparkFlex algaeMotor = new SparkFlex(ALGAE_MANIPULATOR, MotorType.kBrushless);
    private SparkFlexConfig algaeConfig = new SparkFlexConfig();

    private ValveDA algaePivot = new ValveDA(ALGAE_PIVOT);
    private ValveDA algaeExtend = new ValveDA(ALGAE_EXTEND);

    private boolean isRunning = false;
    public boolean algaePivotStatus = false;
    public boolean algaeExtendStatus = false;

    public AlgaeManipulator(){
        algaeConfig.idleMode(IdleMode.kBrake);

        algaeMotor.configure(algaeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        algaeConfig.follow(algaeMotor);

        Util.consoleLog("Algae Manipulator Initialized");
        }
    

    public void start(double speedfactor){

        isRunning = Math.abs(speedfactor) > 0.02;
        updateDS();

        SmartDashboard.putNumber("Algae_SpeedFactor", speedfactor);
        algaeMotor.set(Util.clampValue(speedfactor, 1));
    }

    public void startIntaking(){
        SmartDashboard.putBoolean("Intake Status", true);
        algaeMotor.set(-0.5);

    }

    public void startOuttaking(){
        SmartDashboard.putBoolean("Intake Status", false);
        algaeMotor.set(0.5);
    }
    public void start(){
       start(1);
    }

    public void stop(){
        Util.consoleLog();

        algaeMotor.stopMotor();
        pivotDown();

        isRunning = false;
        algaePivotStatus = false;
        updateDS();
    }

    public void pivotUp(){
        Util.consoleLog();

        algaePivot.SetA();
    
        algaePivotStatus = true;
        updateDS();

    }

    
    public void pivotDown(){
        Util.consoleLog();

        algaePivot.SetB();

        algaePivotStatus = false;
        updateDS();
    }

    public void extendOut(){
        Util.consoleLog();

        algaeExtend.SetA();

        algaeExtendStatus = true;
    }

    public void retractIn(){
        Util.consoleLog();

        algaeExtend.SetB();

        algaeExtendStatus = false;
    }

    public void setAlgaePivot(boolean status){
        Util.consoleLog();

        if (status == true){
            pivotUp();
        } else if (status == false){
            pivotDown();
        }
    }

    public void setAlgaeExtend(boolean status){
        Util.consoleLog();

        if (status == true){
            algaeExtend.SetA();
        } else if (status == false){
            algaeExtend.SetB();
        }
    }   
    private void updateDS() {
        SmartDashboard.putBoolean("Algae Manipulator", isRunning);
        SmartDashboard.putBoolean("Algae Pivot", algaePivotStatus);
    }
}
