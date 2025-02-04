package Team4450.Robot25.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatedManipulator extends SubsystemBase {

    public final CoralManipulator coralManipulator;
    public final AlgaeManipulator algaeManipulator;
    public final Elevator elevator;
    
    public static enum PresetPosition{
        /* Coral Station Intaking Position */ CORAL_STATION_INTAKE,
        /* Coral Scoring Position L1 */ CORAL_SCORING_L1,
        /* Coral Scoring Position L2 */ CORAL_SCORING_L2,
        /* Coral Scoring Position L3 */ CORAL_SCORING_L3,
        /* Coral Scoring Position L4 */ CORAL_SCORING_L4,
        /* Algae Removing Position L2 */ ALGAE_REMOVE_L2,
        /*Algae Removing Position L3 */ ALGAE_REMOVE_L3,
        /*Algae Removing Position L4 */ ALGAE_REMOVE_L4,
        /* Alage Net Scoring Position */ ALGAE_NET_SCORING,
        /* No Position */ NONE,
    };

    public boolean intakeDoesTheAlgaeInsteadOfCoral = true;
    private double endGoalElevatorPosition;


    private boolean endGoalCoralPivotStatus;

    private boolean endGoalAlgaeExtendStatus;

    private boolean endGoalAlgaePivotStatus;

    private PresetPosition position = PresetPosition.NONE;

    public ElevatedManipulator(){
        coralManipulator = new CoralManipulator();
        algaeManipulator = new AlgaeManipulator();
        elevator = new Elevator();
        SmartDashboard.putString("Elevator Position Phase", "_");
    }

    public boolean executeSetPosition(PresetPosition position){
        this.position = position;
        
        switch(position){
            case CORAL_STATION_INTAKE:
                endGoalElevatorPosition = 0.0;
                endGoalCoralPivotStatus = true;
                endGoalAlgaeExtendStatus = false;
                endGoalAlgaePivotStatus = false;
                break;
            case CORAL_SCORING_L1:
                endGoalElevatorPosition = 0.0;
                endGoalCoralPivotStatus = false;
                endGoalAlgaeExtendStatus = false;
                endGoalAlgaePivotStatus = false;
                break;

            case CORAL_SCORING_L2:
                endGoalElevatorPosition = 0.0;
                endGoalCoralPivotStatus = false;
                endGoalAlgaeExtendStatus = false;
                endGoalAlgaePivotStatus = false;
                break;
            
            case CORAL_SCORING_L3:
                endGoalElevatorPosition = 0.0;
                endGoalCoralPivotStatus = false;
                endGoalAlgaeExtendStatus = false;
                endGoalAlgaePivotStatus = false;
                break;

            case CORAL_SCORING_L4:
                endGoalElevatorPosition = 0.0;
                endGoalCoralPivotStatus = false;
                endGoalAlgaeExtendStatus = false;
                endGoalAlgaePivotStatus = false;
                break;

            case ALGAE_REMOVE_L2:
                endGoalElevatorPosition = 0.0;
                endGoalCoralPivotStatus = false;
                endGoalAlgaeExtendStatus = true;
                endGoalAlgaePivotStatus = false;
                break;
            
            case ALGAE_REMOVE_L3:
                endGoalElevatorPosition = 0.0;
                endGoalCoralPivotStatus = false;
                endGoalAlgaeExtendStatus = true;
                endGoalAlgaePivotStatus = false;
                break;
            
            case ALGAE_REMOVE_L4:
                endGoalElevatorPosition = 0.0;
                endGoalCoralPivotStatus = false;
                endGoalAlgaeExtendStatus = true;
                endGoalAlgaePivotStatus = false;
                break;
            
            case ALGAE_NET_SCORING:
                endGoalElevatorPosition = 0.0;
                endGoalCoralPivotStatus = false;
                endGoalAlgaeExtendStatus = false;
                endGoalAlgaePivotStatus = true;
                break;
            
            case NONE:
                break;
        }

        return execute();
    }

    public boolean executeSetPosition(double elevatorPosition, boolean coralPivotStatus, boolean algaeExtendStatus, boolean algaePivotStatus){
        this.position = PresetPosition.NONE;
        this.endGoalCoralPivotStatus = coralPivotStatus;
        this.endGoalAlgaeExtendStatus = algaeExtendStatus;
        this.endGoalAlgaePivotStatus = algaePivotStatus;
        this.endGoalElevatorPosition = elevatorPosition;

        return execute();
    }

    public boolean execute() {
        boolean atTarget = true;

        if (coralManipulator.coralPivotStatus != endGoalCoralPivotStatus) {
            coralManipulator.setCoralPivot(endGoalCoralPivotStatus);
            SmartDashboard.putString("Elevator Position Phase", "Setting Coral Pivot");
            atTarget = false;
        }

        if (algaeManipulator.algaeExtendStatus != endGoalAlgaeExtendStatus) {
            algaeManipulator.setAlgaeExtend(endGoalAlgaeExtendStatus);
            SmartDashboard.putString("Elevator Position Phase", "Setting Algae Extend");
            atTarget = false;
        }

        if (algaeManipulator.algaePivotStatus != endGoalAlgaePivotStatus) {
            algaeManipulator.setAlgaePivot(endGoalAlgaePivotStatus);
            SmartDashboard.putString("Elevator Position Phase", "Setting Algae Pivot");
            atTarget = false;
        }

        if (atTarget) {
            if (isElevatorAtTarget(endGoalElevatorPosition)) {
                elevator.setElevatorHeight(endGoalElevatorPosition);
                elevator.move(0);
                SmartDashboard.putString("Elevator Position Phase", "Elevator at Target");
            } else {
                elevator.setElevatorHeight(endGoalElevatorPosition);
                SmartDashboard.putString("Elevator Position Phase", "Setting Elevator Height");
            }
        }

        return atTarget;
    }

    public void periodic() {
        SmartDashboard.putString("Preset Targret", position.name());
    }

    public void unlockPosition(){
        elevator.unlockPosition();
    }

    private boolean isElevatorAtTarget(double height){
        return elevator.isElevatorAtTarget(height);
    }

    public void resetEncoders(){
        elevator.resetEncoders();
    }
}
