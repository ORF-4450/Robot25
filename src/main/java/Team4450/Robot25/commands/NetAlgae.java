package Team4450.Robot25.commands;

import Team4450.Lib.Util;
import Team4450.Robot25.subsystems.AlgaeManipulator;
// import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class NetAlgae extends Command {
    private final AlgaeManipulator algaeManipulator;

    private static enum State{REMOVE, RETURN, STOP};
    private State state = State.REMOVE;
    double startTime;

    public NetAlgae(AlgaeManipulator algaeManipulator){
        this.algaeManipulator = algaeManipulator;

        addRequirements(algaeManipulator);

        SmartDashboard.putString("Algae Manipulator Status", state.name());
    }

    public void initialize(){
        state = State.REMOVE;
        SmartDashboard.putString("Algae Manipulator Status", state.name());
        startTime = Util.timeStamp();
    }

    public void execute(){
        switch(state){

            case REMOVE:
                algaeManipulator.startOuttaking();
                if(Util.timeStamp() - startTime > 3.0)
                    state = State.STOP;
                break;

            case RETURN:
                algaeManipulator.retractIn();
                algaeManipulator.pivotUp();
                if(algaeManipulator.algaeExtendStatus && algaeManipulator.algaePivotStatus== false)
                    state = State.STOP;
                break;

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