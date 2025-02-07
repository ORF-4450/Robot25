package Team4450.Robot25.commands;

import Team4450.Lib.Util;
import Team4450.Robot25.subsystems.AlgaeManipulator;
import Team4450.Robot25.subsystems.ElevatedManipulator;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RemoveAlgae extends Command {
    private final AlgaeManipulator algaeManipulator;

    private static enum State{REMOVE, RETURN, STOP};
    private State state = State.RETURN;

    public RemoveAlgae(AlgaeManipulator algaeManipulator){
        this.algaeManipulator = algaeManipulator;

        addRequirements(algaeManipulator);

        SmartDashboard.putString("Algae Manipulator Status", state.name());
    }

    public void initialize(){
        state = State.RETURN;
        SmartDashboard.putString("Algae Manipulator Status", state.name());
    }

    public void execute(){
        switch(state){

            case REMOVE:
                algaeManipulator.startIntaking();
                break;

            case RETURN:
                algaeManipulator.retractIn();
                algaeManipulator.pivotUp();

            case STOP:
                algaeManipulator.stop();
                break;
        }
    }

    public boolean isFinished(){
        return state == State.STOP;
    }   

    public void end(boolean interrupted){
        Util.consoleLog("interrupted=%b", interrupted);
        algaeManipulator.stop();    
    }
}
