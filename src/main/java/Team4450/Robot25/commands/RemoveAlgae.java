package Team4450.Robot25.commands;

import Team4450.Lib.Util;
import Team4450.Robot25.subsystems.AlgaeManipulator;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RemoveAlgae extends Command {
    private final AlgaeManipulator algaeManipulator;

    private static enum State{REMOVE, RETRACT, STOP};
    private State state = State.REMOVE;

    double startTime;

    public RemoveAlgae(AlgaeManipulator algaeManipulator){
        this.algaeManipulator = algaeManipulator;

        addRequirements(algaeManipulator);

        SmartDashboard.putString("Algae Manipulator Status", state.name());
    }
    
    //Sets the state of the switch case in execute() to REMOVE
    public void initialize(){
        state = State.REMOVE;
        SmartDashboard.putString("Algae Manipulator Status", state.name());

        startTime = Util.timeStamp(); //Start the timer
    }

    public void execute(){
        switch(state){

            case REMOVE:
                algaeManipulator.startIntaking(); //Start intaking the algae
                if(Util.timeStamp() - startTime > 2.0){ //If the time elapsed is greater than 2 seconds
                    
                    state = State.RETRACT; //Switch the state of the switch case to RETURN
                 } 
                break;

            case RETRACT:
                algaeManipulator.retractIn();
                
            case STOP:
                algaeManipulator.stop();
                break;
        }
    }

    public void end(boolean interrupted){
        Util.consoleLog("interrupted=%b", interrupted);
        algaeManipulator.stop();    
    }
}
