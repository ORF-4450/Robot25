package Team4450.Robot25.commands;

import Team4450.Lib.Util;
import Team4450.Robot25.subsystems.AlgaeManipulator;
import Team4450.Robot25.subsystems.ElevatedManipulator;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RemoveAlgae extends Command {
    // private final AlgaeManipulator algaeManipulator;
    private final ElevatedManipulator elevatedManipulator;

    private static enum State{REMOVE, RETURN, STOP};
    private State state = State.RETURN;

    double startTime;
    public RemoveAlgae(ElevatedManipulator elevatedManipulator){
        // this.algaeManipulator = algaeManipulator;
        this.elevatedManipulator = elevatedManipulator;
        addRequirements(elevatedManipulator);
    }

    public void initialize(){
        state = State.REMOVE;
        SmartDashboard.putString("Algae Manipulator Status", state.name());
        startTime = Util.timeStamp();
    }

    public void execute(){
        switch(state){
            case REMOVE:
                elevatedManipulator.algaeManipulator.startIntaking();
                if(Util.timeStamp() - startTime > 2.0)
                    state = State.RETURN;
                break;

            case RETURN:
                elevatedManipulator.algaeManipulator.retractIn();
                elevatedManipulator.algaeManipulator.pivotUp();
                if(elevatedManipulator.algaeManipulator.algaeExtendStatus == false)
                    state = State.STOP;
                break;

            case STOP:
                elevatedManipulator.algaeManipulator.stop();
                break;
        }
    }

    public boolean isFinished(){
        return state == State.STOP;
    }   

    public void end(boolean interrupted){
        Util.consoleLog("interrupted=%b", interrupted);
        elevatedManipulator.algaeManipulator.stop();    
    }
}