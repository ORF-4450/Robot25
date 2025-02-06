package Team4450.Robot25.commands;

import Team4450.Lib.Util;
import Team4450.Robot25.subsystems.CoralManipulator;
import Team4450.Robot25.subsystems.ElevatedManipulator;
import Team4450.Robot25.subsystems.ElevatedManipulator.PresetPosition;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeCoral extends Command {
    private final CoralManipulator coralManipulator;
    private final ElevatedManipulator elevatedManipulator;

    private static enum State{MOVING, INTAKE, STOP};
    private State state = State.INTAKE;

    public IntakeCoral(CoralManipulator coralManipulator, ElevatedManipulator elevatedManipulator){
        this.coralManipulator = coralManipulator;
        this.elevatedManipulator = elevatedManipulator;

        addRequirements(coralManipulator, elevatedManipulator);

        SmartDashboard.putString("Intake Coral Status", state.name());
    }

    public void initialize(){
        state = State.MOVING;
        elevatedManipulator.executeSetPosition(PresetPosition.CORAL_STATION_INTAKE);
        SmartDashboard.putString("Intake Coral Status", state.name());
    }

    public void execute(){
        switch(state){
            case MOVING:
                if(elevatedManipulator.executeSetPosition(PresetPosition.CORAL_STATION_INTAKE))
                    state = State.INTAKE;
                    SmartDashboard.putString("Intake Coral Status", state.name());
                break;
                
            case INTAKE:
                coralManipulator.startIntaking();

                if(elevatedManipulator.hasCoral())
                    state = State.STOP;
                    SmartDashboard.putString("Intake Coral Status", state.name());
                break;

            case STOP:
                coralManipulator.stop();
                break;
        }
    }

    public boolean isFinished(){
        return state == State.STOP;
    }

    public void end(boolean interrupted){
        Util.consoleLog("interrupted=%b", interrupted);
        coralManipulator.stop();
        SmartDashboard.putString("Intake Coral Status", state.name());
    }
}
