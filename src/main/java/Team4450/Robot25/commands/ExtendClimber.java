package Team4450.Robot25.commands;

import Team4450.Lib.Util;
import Team4450.Robot25.subsystems.Climber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class ExtendClimber extends Command {
    private final Climber climber;

    private static enum State { EXTENDING, STOP }
    private State state = State.STOP;

    public ExtendClimber(Climber climber) {
        this.climber = climber;

        addRequirements(climber);

        SmartDashboard.putString("Climber Status", state.name());
    }

    @Override
    public void initialize() {
        state = State.EXTENDING;
        SmartDashboard.putString("Climber Status", state.name());
    }

    @Override
    public void execute() {
        switch (state) {
            case EXTENDING:
                climber.extendPiston();
                
                if(climber.pistonStatus() == true)
                    state = State.STOP;
                SmartDashboard.putString("Climber Status", state.name());
                break;

            case STOP:
                // Do nothing
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return state == State.STOP;
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
        SmartDashboard.putString("Climber Status", state.name());
    }
}