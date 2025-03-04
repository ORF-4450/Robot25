package Team4450.Robot25.subsystems;

import Team4450.Lib.Util;
import Team4450.Lib.ValveSA;
import static Team4450.Robot25.Constants.CLIMBER_PISTON;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private ValveSA climberPiston = new ValveSA(CLIMBER_PISTON, PneumaticsModuleType.REVPH);

    private boolean pistonExtended = false;

    public Climber() {
        Util.consoleLog("Climber Initialized");
    }

    public void extendPiston() {
        Util.consoleLog();
        climberPiston.Open();
        pistonExtended = true;
        updateDS();
    }

    public void retractPiston() {
        Util.consoleLog();
        climberPiston.Close();
        pistonExtended = false;
        updateDS();
    }
    
    public boolean pistonStatus(){
        return pistonExtended;
    }

    private void updateDS() {
        SmartDashboard.putBoolean("Climber Piston", pistonExtended);
    }
}