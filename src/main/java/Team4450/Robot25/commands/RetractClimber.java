package Team4450.Robot25.commands;

import Team4450.Lib.Util;
import Team4450.Robot25.subsystems.Climber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RetractClimber extends Command {
    private final Climber climber;
    private boolean isFinished = false;

    public RetractClimber(Climber climber) {
        this.climber = climber;

        addRequirements(climber);

        SmartDashboard.putString("Climber Status", "RETRACTING");
    }

    @Override
    public void initialize() {
        SmartDashboard.putString("Climber Status", "RETRACTING");
    }

    @Override
    public void execute(){
        
        climber.retractPiston();

        if(climber.pistonStatus() == false)
            isFinished = true;
        }

    @Override
    public boolean isFinished() {
        return isFinished; // This command runs until interrupted
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        SmartDashboard.putString("Climber Status", "STOPPED");
    }
}