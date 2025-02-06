package Team4450.Robot25.commands;

import Team4450.Lib.Util;
import Team4450.Robot25.subsystems.ElevatedManipulator;
import Team4450.Robot25.subsystems.ElevatedManipulator.PresetPosition;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Move the shooter and elevator to one of many "PresetPosition"s defined in the
 * ElevatedManipulator subsystem class. Very helpful to chain commands with in RobotContainer!
 */
public class Preset extends Command {
    private final ElevatedManipulator elevatedManipulator;
    private PresetPosition preset;

    private boolean done = false; // whether it's done or not

    /**
     * Move the shooter and elevator to one of many "PresetPosition"s defined in the
     * ElevatedManipulator subsystem class.
     * @param ElevatedManipulator the ElevatedManipulator subsystem
     * @param preset the PresetPosition to go to
     */
    public Preset(ElevatedManipulator elevatedManipulator, PresetPosition preset) {
        this.elevatedManipulator = elevatedManipulator;
        this.preset = preset;
        addRequirements(elevatedManipulator);
    }

    @Override
    public void initialize() {
        Util.consoleLog();
    }

    @Override
    public void execute() {
        // must be called every loop!
        // most of the code for this is in ElevatedManipulator, so better to look
        // there for info/code
        done = elevatedManipulator.executeSetPosition(preset);
    }

    @Override
    public boolean isFinished() {
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        Util.consoleLog("interrupted=%b", interrupted);
    }
}
