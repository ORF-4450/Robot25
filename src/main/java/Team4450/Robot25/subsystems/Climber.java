package Team4450.Robot25.subsystems;

import Team4450.Lib.Util;
import Team4450.Lib.ValveSA;
import static Team4450.Robot25.Constants.CLIMBER_PISTON;
import static Team4450.Robot25.Constants.CLIMBER_PISTON2;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private ValveSA climberPiston = new ValveSA(CLIMBER_PISTON, PneumaticsModuleType.REVPH);
    private ValveSA climberPiston2 = new ValveSA(CLIMBER_PISTON2, PneumaticsModuleType.REVPH);

    private boolean pistonExtended = false;
    private boolean piston2Extended = false;

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

    public void extendPiston2() {
        Util.consoleLog();
        climberPiston2.Open();
        piston2Extended = true;
        updateDS();
    }

    public void retractPiston2() {
        Util.consoleLog();
        climberPiston2.Close();
        piston2Extended = false;
        updateDS();
    }

    private void updateDS() {
        SmartDashboard.putBoolean("Climber Piston", pistonExtended);
        SmartDashboard.putBoolean("Climber Piston2", piston2Extended);
    }
}