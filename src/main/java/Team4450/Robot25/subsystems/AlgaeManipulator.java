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

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeManipulator extends SubsystemBase {
    private SparkFlex algaeMotor = new SparkFlex(ALGAE_MANIPULATOR, MotorType.kBrushless);
    private SparkFlexConfig algaeConfig = new SparkFlexConfig();

    private ValveDA algaePivot = new ValveDA(ALGAE_PIVOT, PneumaticsModuleType.REVPH);
    private ValveDA algaeExtend = new ValveDA(ALGAE_EXTEND, PneumaticsModuleType.REVPH);

    public boolean isAlgaeMotorRunning = false;
    public boolean algaePivotStatus = false;
    public boolean algaeExtendStatus = false;
    public double algaeCurrent;

    public AlgaeManipulator(){
        //algaeConfig.idleMode(IdleMode.kBrake);

        algaeMotor.configure(algaeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        algaePivot.setName("algaePivot");
        algaeExtend.setName("algaeExtend");

        Util.consoleLog("Algae Manipulator Initialized");
    }
        
    public void intialize(){
        Util.consoleLog();
        
        pivotDown();
        retractIn();

        // runDefault();

        // isAlgaeMotorRunning = true;
        algaePivotStatus = false;
        algaeExtendStatus = false;

        updateDS();
    }

    // public void periodic(){
    //  Util.consoleLog("Algae Motor Current:" + algaeMotor.getOutputCurrent());
    //  Util.consoleLog("Algae Motor Voltage:"+ algaeMotor.getAppliedOutput());
     
    // }
    public void start(double speedfactor){
        isAlgaeMotorRunning = Math.abs(speedfactor) > 0.02;
        
        updateDS();

        algaeMotor.set(Util.clampValue(speedfactor, 1));
    }

    public void startIntaking(){
        isAlgaeMotorRunning = true;
        algaeMotor.set(-0.5);
        updateDS();
    }

    public void runDefault(){
        isAlgaeMotorRunning = true;
        algaeMotor.set(-0.1);
        updateDS();
    }

    public void holdAlgae(){
        isAlgaeMotorRunning = true;
        algaeMotor.set(-0.20);
        updateDS();
    }
    public void startOuttaking(){
        isAlgaeMotorRunning = true;
        algaeMotor.set(1);
        updateDS();
    }

    public void processAlgae(){
        isAlgaeMotorRunning = true;
        algaeMotor.set(0.05);
        updateDS();
    }

    public void start(){
       start(0.1);
       isAlgaeMotorRunning = true;
       updateDS();
    }

    public void stop(){
        Util.consoleLog();
        // if(hasAlgae() == true) {
        //     holdAlgae();
        // isAlgaeMotorRunning = true;
        // }
        // else if(hasAlgae() == false){
        algaeMotor.stopMotor();
        // isAlgaeMotorRunning = false;
        // }

        pivotDown();

        isAlgaeMotorRunning = false;
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

        updateDS();
    }

    public void retractIn(){
        Util.consoleLog();

        algaeExtend.SetB();

        algaeExtendStatus = false;

        updateDS();
    }

    public void setAlgaePivot(boolean status){
        Util.consoleLog();

        if (status == true){
            pivotUp();
            algaePivotStatus = true;
        } else if (status == false){
            pivotDown();
            algaePivotStatus = false;
        }
    
        updateDS();
    }

    public void setAlgaeExtend(boolean status){
        Util.consoleLog();

        if (status == true){
            extendOut();
            algaeExtendStatus = true;
        } else if (status == false){
            retractIn();
            algaeExtendStatus = false;
        }

        updateDS();
    }   

    public boolean hasAlgae(){
        return this.getCurrent() > 80.0;
    }

    public double getCurrent(){
        double algCurrent = algaeMotor.getOutputCurrent();
        SmartDashboard.putNumber("Algae Manipulator Current", algCurrent);
        return algCurrent;
    }
    
    private void updateDS() {
        SmartDashboard.putBoolean("Algae Manipulator Running", isAlgaeMotorRunning);
        SmartDashboard.putBoolean("Algae Pivot On", algaePivotStatus);
        SmartDashboard.putBoolean("Algae Extended Out", algaeExtendStatus);
   }
}
