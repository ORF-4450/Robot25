package Team4450.Robot25.subsystems;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ElevatedManipulator extends SubsystemBase {

    public final CoralManipulator coralManipulator;
    public final AlgaeManipulator algaeManipulator;
    public final AlgaeGroundIntake algaeGroundIntake;
    public final Elevator elevator;
    
    public static enum PresetPosition{
        /* Reset Position */ RESET,
        /* Coral Station Intaking Position */ CORAL_STATION_INTAKE,
        /* Coral Scoring Position L1 */ CORAL_SCORING_L1,
        /* Coral Scoring Position L2 */ CORAL_SCORING_L2,
        /* Coral Scoring Position L3 */ CORAL_SCORING_L3,
        /* Coral Scoring Position L4 */ CORAL_SCORING_L4,
        /* Algae Removing Position L2 */ ALGAE_REMOVE_L2,
        /*Algae Removing Position L3 */ ALGAE_REMOVE_L3,
        /* Alage Net Scoring Position */ ALGAE_NET_SCORING,
        /* Algae Processor Scoring Position */ ALGAE_PROCESSOR_SCORING,
        /*Algae Ground Intake Position */ ALGAE_GROUND_INTAKE,
        /* No Position */ NONE,
    };

    public boolean scoreCoralInsteadOfAlgae = true;
    
    public boolean intakeCoralInsteadOfAlgae = true;

    private double endGoalElevatorHeight;

    private boolean endGoalCoralPivotStatus;

    private boolean endGoalAlgaeExtendStatus;

    private boolean endGoalAlgaePivotStatus;

    private boolean endGoalAlgaeGroundPistonStatus;

    private PresetPosition position = PresetPosition.NONE;

    public ElevatedManipulator(CoralManipulator coralManipulator, 
                               AlgaeManipulator algaeManipulator, 
                               AlgaeGroundIntake algaeGroundIntake,
                               Elevator elevator){
        Util.consoleLog();

        this.coralManipulator = coralManipulator;
        this.algaeManipulator = algaeManipulator;
        this.algaeGroundIntake = algaeGroundIntake;
        this.elevator = elevator;
        
        SmartDashboard.putString("Elevator Position Phase", "Initalized");
    }

    public boolean executeSetPosition(PresetPosition position){
        this.position = position;
        
        switch(position){
            case RESET:
                endGoalElevatorHeight = 0.05;
                endGoalCoralPivotStatus = false;
                if(algaeManipulator.hasAlgae())
                    endGoalAlgaeExtendStatus = true;
                else
                    endGoalAlgaeExtendStatus = false;
                endGoalAlgaePivotStatus = false;
                endGoalAlgaeGroundPistonStatus = false;
                break;

            case CORAL_STATION_INTAKE:
                endGoalElevatorHeight = 0.0;
                endGoalCoralPivotStatus = true;
                if(algaeManipulator.hasAlgae())
                    endGoalAlgaeExtendStatus = true;
                else
                    endGoalAlgaeExtendStatus = false;
                endGoalAlgaePivotStatus = false;
                endGoalAlgaeGroundPistonStatus = false;
                break;

            case CORAL_SCORING_L1:
                endGoalElevatorHeight = 0.44;
                endGoalCoralPivotStatus = false;
                if(algaeManipulator.hasAlgae())
                    endGoalAlgaeExtendStatus = true;
                else
                    endGoalAlgaeExtendStatus = false;          
                endGoalAlgaeGroundPistonStatus = false;
                break;

            case CORAL_SCORING_L2:
                endGoalElevatorHeight = 0.60;
                endGoalCoralPivotStatus = false;
                if(algaeManipulator.hasAlgae())
                    endGoalAlgaeExtendStatus = true;
                else
                    endGoalAlgaeExtendStatus = false;
                endGoalAlgaePivotStatus = false;
                endGoalAlgaeGroundPistonStatus = false;
                break;
            
            case CORAL_SCORING_L3:
                endGoalElevatorHeight = 0.99;
                endGoalCoralPivotStatus = false;
                if(algaeManipulator.hasAlgae())
                    endGoalAlgaeExtendStatus = true;
                else
                    endGoalAlgaeExtendStatus = false;
                endGoalAlgaePivotStatus = false;
                endGoalAlgaeGroundPistonStatus = false;
                break;

            case CORAL_SCORING_L4:
                endGoalElevatorHeight = 1.60;
                endGoalCoralPivotStatus = false;
                if(algaeManipulator.hasAlgae())
                    endGoalAlgaeExtendStatus = true;
                else
                    endGoalAlgaeExtendStatus = false;
                endGoalAlgaePivotStatus = false;
                endGoalAlgaeGroundPistonStatus = false;
                break;

            case ALGAE_REMOVE_L2:
                endGoalElevatorHeight = 0.61;
                endGoalCoralPivotStatus = false;
                endGoalAlgaeExtendStatus = true;
                endGoalAlgaePivotStatus = false;
                endGoalAlgaeGroundPistonStatus = false;
                break;
            
            case ALGAE_REMOVE_L3:
                endGoalElevatorHeight = 1.07;
                endGoalCoralPivotStatus = false;
                endGoalAlgaeExtendStatus = true;
                endGoalAlgaePivotStatus = false;
                endGoalAlgaeGroundPistonStatus = false;
                break;
            
            case ALGAE_NET_SCORING:
                endGoalElevatorHeight = 1.52;
                endGoalCoralPivotStatus = false;
                endGoalAlgaeExtendStatus = true;
                endGoalAlgaePivotStatus = true;
                endGoalAlgaeGroundPistonStatus = false;
                break;
            
            case ALGAE_PROCESSOR_SCORING:
                endGoalElevatorHeight = 0.05;
                endGoalCoralPivotStatus = false;
                endGoalAlgaeExtendStatus = true;
                endGoalAlgaePivotStatus = false;
                endGoalAlgaeGroundPistonStatus = false;
                break;

            case ALGAE_GROUND_INTAKE:
                endGoalElevatorHeight = 0.05;
                endGoalCoralPivotStatus = false;
                endGoalAlgaeExtendStatus = true;
                endGoalAlgaePivotStatus = false;
                endGoalAlgaeGroundPistonStatus = true;
                break;
            
            case NONE:
                break;
        }

        return execute();
    }

    public boolean executeSetPosition(double elevatorPosition, boolean coralPivotStatus, boolean algaeExtendStatus, boolean algaePivotStatus, boolean algaeGroundPistonStatus){
        this.position = PresetPosition.NONE;
        this.endGoalCoralPivotStatus = coralPivotStatus;
        this.endGoalAlgaeExtendStatus = algaeExtendStatus;
        this.endGoalAlgaePivotStatus = algaePivotStatus;
        this.endGoalElevatorHeight = elevatorPosition;
        this.endGoalAlgaeGroundPistonStatus = algaeGroundPistonStatus;

        return execute();
    }

    public boolean execute() {
        boolean atTarget = true;

        // Handle Coral Pivot
        if (coralManipulator.coralPivotStatus != endGoalCoralPivotStatus) {
            coralManipulator.setCoralPivot(endGoalCoralPivotStatus);
            SmartDashboard.putString("Elevator Position Phase", "Setting Coral Pivot");
            atTarget = false;
        }

        //Determine if we are extending or retracting
        //endGoalAlgaeExtendStatus is equal to true when we are extending & algaeExtendStatus is equal to false when we aren't extended
        //endGoalAlgaeExtendStatus is equal to false when we are retracting & algaeExtendStatus is equal to true when we are extended
        boolean isExtending = endGoalAlgaeExtendStatus && !algaeManipulator.algaeExtendStatus;
        boolean isRetracting = !endGoalAlgaeExtendStatus && algaeManipulator.algaeExtendStatus;

        // Handle Algae Extend, Pivot, and Ground Piston based on direction
        if (isExtending) {
            // Extend ground intake piston first
            if (algaeGroundIntake.algaeGroundPistonStatus != endGoalAlgaeGroundPistonStatus) {
                algaeGroundIntake.setAlgaeGroundExtend(endGoalAlgaeGroundPistonStatus);
                SmartDashboard.putString("Elevator Position Phase", "Setting Algae Ground Piston Extend");
                atTarget = false;
            } else if (algaeManipulator.algaeExtendStatus != endGoalAlgaeExtendStatus) {
                algaeManipulator.setAlgaeExtend(endGoalAlgaeExtendStatus);
                SmartDashboard.putString("Elevator Position Phase", "Setting Algae Extend");
                atTarget = false;
            } else if (algaeManipulator.algaePivotStatus != endGoalAlgaePivotStatus) {
                algaeManipulator.setAlgaePivot(endGoalAlgaePivotStatus);
                SmartDashboard.putString("Elevator Position Phase", "Setting Algae Pivot Up");
                atTarget = false;
            }
        } else if (isRetracting) {
            // Retract pivot first
            if (algaeManipulator.algaePivotStatus != endGoalAlgaePivotStatus) {
                algaeManipulator.setAlgaePivot(endGoalAlgaePivotStatus);
                SmartDashboard.putString("Elevator Position Phase", "Setting Algae Pivot Down");
                atTarget = false;
            } else if (algaeManipulator.algaeExtendStatus != endGoalAlgaeExtendStatus) {
                algaeManipulator.setAlgaeExtend(endGoalAlgaeExtendStatus);
                SmartDashboard.putString("Elevator Position Phase", "Setting Algae Retract");
                atTarget = false;
            } else if (algaeGroundIntake.algaeGroundPistonStatus != endGoalAlgaeGroundPistonStatus) {
                algaeGroundIntake.setAlgaeGroundExtend(endGoalAlgaeGroundPistonStatus);
                SmartDashboard.putString("Elevator Position Phase", "Setting Algae Ground Piston Retract");
                atTarget = false;
            }
        } 

        // Handle Elevator
        if (atTarget) {
            if (isElevatorAtTarget(endGoalElevatorHeight)) {
                elevator.setElevatorHeight(endGoalElevatorHeight);
                elevator.move(0);
                SmartDashboard.putString("Elevator Position Phase", "Elevator at Target");
            } else {
                elevator.setElevatorHeight(endGoalElevatorHeight);
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

    public void moveRelative(double elevatorHeightChange) {
        elevator.move(elevatorHeightChange * 0.9);
    }

    public boolean hasCoral(){
        return coralManipulator.hasCoral();
    }
    private boolean isElevatorAtTarget(double height){
        return elevator.isElevatorAtTarget(height);
    }

    public void resetEncoders(){
        elevator.resetEncoders();
    }
}
