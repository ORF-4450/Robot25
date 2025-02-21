package Team4450.Robot25.commands;

import Team4450.Lib.Util;
import Team4450.Robot25.subsystems.Climber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RetractClimber extends CommandBase {
    private final Climber climber;

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
    public void execute() {
        climber.retractPiston();
        climber.retractPiston2();
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs until interrupted
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        climber.retractPiston();
        climber.retractPiston2();
        SmartDashboard.putString("Climber Status", "STOPPED");
    }
}