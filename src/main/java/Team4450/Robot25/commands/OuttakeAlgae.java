package Team4450.Robot25.commands;
import Team4450.Lib.Util;
// import Team4450.Robot25.subsystems.AlgaeManipulator;
import Team4450.Robot25.subsystems.ElevatedManipulator;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
public class OuttakeAlgae extends Command {
    
    private final ElevatedManipulator elevatedManipulator;
    private static enum State{OUTTAKE, STOP};
    private State state = State.OUTTAKE;
    double startTime;

    public OuttakeAlgae(ElevatedManipulator elevatedManipulator){
       this.elevatedManipulator = elevatedManipulator;

       SmartDashboard.putString("Algae Manipulator Status", state.name());
    }
    public void initialize(){
        state = State.OUTTAKE;
        SmartDashboard.putString("Algae Manipulator Status", state.name());
        startTime = Util.timeStamp();
    }
    public void execute(){
        switch(state){
            case OUTTAKE:
                elevatedManipulator.algaeManipulator.startOuttaking();
                if(Util.timeStamp() - startTime > 2.0)
                    state = State.STOP;
                break;
            // case RETURN:
            //     if(elevatedManipulator.algaeManipulator.algaeExtendStatus && elevatedManipulator.algaeManipulator.algaePivotStatus== false)
            //         state = State.STOP;
            //     break;
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
        elevatedManipulator.scoreCoralInsteadOfAlgae = true; // Change to true to score coral instead of algae.
    }
}