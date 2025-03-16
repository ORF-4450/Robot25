package Team4450.Robot25.commands;

import Team4450.Lib.Util;
// import Team4450.Robot25.subsystems.CoralManipulator;
import Team4450.Robot25.subsystems.ElevatedManipulator;
import Team4450.Robot25.subsystems.ElevatedManipulator.PresetPosition;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class OuttakeCoral extends Command {
    // private final CoralManipulator coralManipulator;
    private final ElevatedManipulator elevatedManipulator;

    private static enum State{OUTTAKE, STOP};
    private State state = State.OUTTAKE;
    private double startTime;

    public OuttakeCoral(ElevatedManipulator elevatedManipulator){
        // this.coralManipulator = coralManipulator;
        this.elevatedManipulator = elevatedManipulator;

        SmartDashboard.putString("Outtake Coral Status", state.name());
    }

    

    public void initialize(){
        state = State.OUTTAKE;
        startTime = Util.timeStamp();
        SmartDashboard.putString("Outtake Coral Status: ", state.name());
    }

    public void execute(){
        switch(state){          
            case OUTTAKE:
            if(elevatedManipulator.isElevatorAtTarget(0.05))
                elevatedManipulator.coralManipulator.startL1Outtaking();
            else
                elevatedManipulator.coralManipulator.startOuttaking();
                
                if(Util.getElaspedTime(startTime) > 0.35)
                    state = State.STOP;
                SmartDashboard.putString("Outtake Coral Status", state.name());
                Util.consoleLog("Outtake Coral Status: " + state.name());
                break;
                
            case STOP:
                elevatedManipulator.coralManipulator.stop();
                SmartDashboard.putString("Outtake Coral Status: ", state.name());
                break;
        }
    }

    public boolean isFinished(){
        return state == State.STOP;
    }

    public void end(boolean interrupted){
        Util.consoleLog("interrupted=%b", interrupted);
        elevatedManipulator.coralManipulator.stop();
        elevatedManipulator.executeSetPosition(PresetPosition.NONE);
    }
}
