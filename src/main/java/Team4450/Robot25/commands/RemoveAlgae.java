package Team4450.Robot25.commands;

import Team4450.Lib.Util;
import Team4450.Robot25.subsystems.AlgaeManipulator;
import Team4450.Robot25.subsystems.ElevatedManipulator;
import Team4450.Robot25.subsystems.ElevatedManipulator.PresetPosition;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RemoveAlgae extends Command {
    private final AlgaeManipulator algaeManipulator;
    private final ElevatedManipulator elevatedManipulator;

    private static enum State{MOVING, LV2EXTEND, LV3EXTEND, REMOVE, RETURN, STOP};
    private State state = State.RETURN;

    public RemoveAlgae(AlgaeManipulator algaeManipulator, ElevatedManipulator elevatedManipulator){
        this.algaeManipulator = algaeManipulator;
        this.elevatedManipulator = elevatedManipulator;

        addRequirements(algaeManipulator, elevatedManipulator);

        SmartDashboard.putString("Algae Manipulator Status", state.name());
    }

    public void initialize(){
        state = State.MOVING;
        elevatedManipulator.executeSetPosition(PresetPosition.NONE);
        SmartDashboard.putString("Algae Manipulator Status", state.name());
    }

    public void execute(){
        switch(state){
            case MOVING:
                if(elevatedManipulator.executeSetPosition(PresetPosition.CORAL_STATION_INTAKE))
                    state = State.REMOVE;
                    SmartDashboard.putString("Intake Coral Status", state.name());
                break;
                
            case LV2EXTEND:
                elevatedManipulator.executeSetPosition(PresetPosition.ALGAE_REMOVE_L2);
                algaeManipulator.extendOut();
                algaeManipulator.pivotDown();
                break;

            case LV3EXTEND:
                elevatedManipulator.executeSetPosition(PresetPosition.ALGAE_REMOVE_L3);
                algaeManipulator.extendOut();
                algaeManipulator.pivotDown();
                break;

            case REMOVE:
                algaeManipulator.startOuttaking();
                break;

            case RETURN:
                algaeManipulator.retractIn();
                algaeManipulator.pivotUp();

            case STOP:
                algaeManipulator.stop();
                break;
        }
    }
}
