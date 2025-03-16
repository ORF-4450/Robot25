package Team4450.Robot25.commands;

import Team4450.Lib.Util;
import Team4450.Robot25.subsystems.AlgaeManipulator;
import Team4450.Robot25.subsystems.ElevatedManipulator;
import Team4450.Robot25.subsystems.ElevatedManipulator.PresetPosition;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeAlgaeGround extends Command {
    private final ElevatedManipulator elevatedManipulator;

    private static enum State{MOVING, INTAKE_ALGAE, STOP};
    private State state = State.MOVING;

    double startTime;
    public IntakeAlgaeGround(ElevatedManipulator elevatedManipulator){
        this.elevatedManipulator = elevatedManipulator;
        addRequirements(elevatedManipulator);
    }

    public void initialize(){
        state = State.INTAKE_ALGAE;
        elevatedManipulator.executeSetPosition(PresetPosition.ALGAE_GROUND_INTAKE); //Moves the algae manipulator and elevator to the intake position and extends the ground intake
        SmartDashboard.putString("Algae Ground Intake Status", state.name());
        startTime = Util.timeStamp();
    }

    public void execute(){
        switch(state){

            case MOVING:
                if(elevatedManipulator.executeSetPosition(PresetPosition.ALGAE_GROUND_INTAKE)) { //Checks if the manipulator and elevator are in the ground intake position
                    state = State.INTAKE_ALGAE; //Then move to the intake state
                SmartDashboard.putString("Algae Ground Intake Status", state.name());
                }
                break;

            case INTAKE_ALGAE:
                elevatedManipulator.algaeManipulator.startIntaking();
                elevatedManipulator.algaeGroundIntake.startRollers();
                if(elevatedManipulator.algaeManipulator.hasAlgae() == true){
                    elevatedManipulator.elevator.setElevatorHeight(0.20);
                    state = State.STOP;
                }
                break;

            case STOP:
                elevatedManipulator.algaeManipulator.holdAlgae();
                elevatedManipulator.algaeGroundIntake.stop();
                elevatedManipulator.algaeGroundIntake.setAlgaeGroundExtend(false);
                elevatedManipulator.executeSetPosition(PresetPosition.NONE);
                break;
        }
    }

    public boolean isFinished(){
        return state == State.STOP;
    }   

    public void end(boolean interrupted){
        Util.consoleLog("interrupted=%b", interrupted);
        elevatedManipulator.algaeManipulator.holdAlgae();    
        elevatedManipulator.algaeGroundIntake.stop();
        elevatedManipulator.executeSetPosition(PresetPosition.NONE);
    }
}