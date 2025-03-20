package Team4450.Robot25.commands;

import Team4450.Lib.Util;
import Team4450.Robot25.subsystems.ElevatedManipulator;
import Team4450.Robot25.subsystems.ElevatedManipulator.PresetPosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCoral extends Command {
    private final ElevatedManipulator elevatedManipulator;

    private static enum State{MOVING, INTAKE, STOP};
    private State state = State.MOVING;

    public IntakeCoral(ElevatedManipulator elevatedManipulator){
        this.elevatedManipulator = elevatedManipulator;

        SmartDashboard.putString("Intake Coral Status", state.name());
    }

    public void initialize(){
        state = State.MOVING;
        elevatedManipulator.executeSetPosition(PresetPosition.CORAL_STATION_INTAKE); //Moves the coral manipulator and elevator to the intake position
        SmartDashboard.putString("Intake Coral Status", state.name());
    }

    public void execute(){
        switch(state){
            case MOVING:
                if(elevatedManipulator.executeSetPosition(PresetPosition.CORAL_STATION_INTAKE)) { //Checks if the manipulator and elevator are in the intake position
                    state = State.INTAKE; //Then move to the intake state
                SmartDashboard.putString("Intake Coral Status", state.name());
                }
                break;
                
            case INTAKE:
                elevatedManipulator.coralManipulator.startIntaking(); //Start intaking the coral

                if(elevatedManipulator.coralManipulator.hasCoral()) //If the getCurrent() method in the CoralManipulator class returns greater than 80 amps, set to true.
                    state = State.STOP; //Switch the state of the switch case to STOP
                SmartDashboard.putString("Intake Coral Status", state.name());
                break;

            case STOP:
                elevatedManipulator.coralManipulator.stop(); //Stop the coral manipulator
                break;
        }
    }

    public boolean isFinished(){
        return state == State.STOP; //If the state of the switch case is STOP, then move into the end() method
    }

    public void end(boolean interrupted){  //If isFinished() returns true, then this method is called
        Util.consoleLog("interrupted=%b", interrupted);
        elevatedManipulator.coralManipulator.stop();
        SmartDashboard.putString("Intake Coral Status", state.name());
    }
}
