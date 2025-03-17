
package Team4450.Robot25;

import static Team4450.Robot25.Constants.*;

import org.opencv.photo.Photo;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import Team4450.Robot25.commands.DriveCommand;
import Team4450.Robot25.commands.DriveToTag;
import Team4450.Robot25.commands.ExtendClimber;
// import Team4450.Robot25.commands.GetPoseEsimate;
import Team4450.Robot25.commands.IntakeCoral;
import Team4450.Robot25.commands.OuttakeCoral;
import Team4450.Robot25.commands.OuttakeProcessor;
import Team4450.Robot25.commands.PointToYaw;
// import Team4450.Robot25.commands.SetTargetPose;
import Team4450.Robot25.commands.UpdateCandle;
import Team4450.Robot25.commands.UpdateVisionPose;
// import Team4450.Robot25.commands.GoToPose;
import Team4450.Robot25.commands.IntakeAlgaeGround;
import Team4450.Robot25.commands.Preset;
import Team4450.Robot25.commands.RemoveAlgae;
import Team4450.Robot25.commands.RetractClimber;
import Team4450.Robot25.commands.SetTarget;
// import Team4450.Robot25.commands.RotateToPose;
// import Team4450.Robot25.commands.RotateToTag;
// import Team4450.Robot25.commands.GoToTag;
// import Team4450.Robot25.commands.SetTagBasedPosition;
import Team4450.Robot25.commands.OuttakeAlgae;
import Team4450.Robot25.commands.DriveToTag;
import Team4450.Robot25.commands.DriveToRight;
import Team4450.Robot25.commands.DriveToLeft;


import Team4450.Robot25.subsystems.AlgaeManipulator;
import Team4450.Robot25.subsystems.AlgaeGroundIntake;
import Team4450.Robot25.subsystems.Candle;
import Team4450.Robot25.subsystems.CoralManipulator;
import Team4450.Robot25.subsystems.DriveBase;
import Team4450.Robot25.subsystems.PhotonVision;
import Team4450.Robot25.subsystems.ShuffleBoard;
import Team4450.Robot25.subsystems.ElevatedManipulator.PresetPosition;
import Team4450.Robot25.subsystems.PhotonVision.PipelineType;
import Team4450.Robot25.subsystems.ElevatedManipulator;
import Team4450.Robot25.subsystems.Elevator;
import Team4450.Robot25.subsystems.Climber;
// import Team4450.Robot25.subsystems.CoralGroundIntake;
import Team4450.Lib.MonitorPDP;
import Team4450.Lib.NavX;
import Team4450.Lib.Util;
import Team4450.Lib.CameraFeed;
import Team4450.Lib.XboxController;
import Team4450.Lib.MonitorCompressorPH;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
	// Subsystems.

	public static ShuffleBoard			shuffleBoard;
	public static DriveBase 			driveBase;
	public static PhotonVision			pvTagCamera;
	private Candle        				candle = null;
	public static Elevator				elevator;
	public static ElevatedManipulator	elevatedManipulator;
	public static AlgaeManipulator 		algaeManipulator;
	public static AlgaeGroundIntake		algaeGroundIntake;
	public static CoralManipulator		coralManipulator;
	// public static CoralGroundIntake 	coralGroundIntake;
	public static Climber 				climber;

	// Subsystem Default Commands.

    // Persistent Commands.

	// Some notes about Commands.
	// When a Command is created with the New operator, its constructor is called. When the
	// command is added to the Scheduler to be run, its initialize method is called. Then on
	// each scheduler run, as long as the command is still scheduled, its execute method is
	// called followed by isFinished. If isFinished it false, the command remains in the
	// scheduler list and on next run, execute is called followed by isFinished. If isFinished
	// returns true, the end method is called and the command is removed from the scheduler list.
	// Now if you create another instance with new, you get the constructor again. But if you 
	// are re-scheduling an existing command instance (like the ones above), you do not get the
	// constructor called, but you do get initialize called again and then on to execute & etc.
	// So this means you have to be careful about command initialization activities as a persistent
	// command in effect has two lifetimes (or scopes): Class global and each new time the command
	// is scheduled. Note the FIRST doc on the scheduler process is not accurate as of 2020.
	
	// GamePads. 2 Game Pads use RobotLib XboxController wrapper class for some extra features.
	// Note that button responsiveness may be slowed as the schedulers command list gets longer 
	// or commands get longer as buttons are processed once per scheduler run.
	
	private XboxController			driverController =  new XboxController(DRIVER_PAD);
	public static XboxController	utilityController = new XboxController(UTILITY_PAD);

	// private PowerDistribution	pdp = new PowerDistribution(REV_PDB, PowerDistribution.ModuleType.kCTRE);
	private PowerDistribution		pdp = new PowerDistribution(REV_PDB, PowerDistribution.ModuleType.kRev);

	// Compressor class controls the CTRE/REV Pneumatics control Module.
	private Compressor				pcm = new Compressor(PneumaticsModuleType.REVPH);

	// Navigation board.
	public static NavX			navx;

	private MonitorPDP     		monitorPDPThread;
	private MonitorCompressorPH	monitorCompressorThread;
    private CameraFeed			cameraFeed;
    
	// Trajectories we load manually.
	//public static PathPlannerTrajectory	ppTestTrajectory;

	private static SendableChooser<Command>	autoChooser;
	
	private static String 					autonomousCommandName = "none";

	/**
	 * The container for the robot. Contains subsystems, Opertor Interface devices, and commands.
	 */
	public RobotContainer() throws Exception
	{
		Util.consoleLog();
		
	    SendableRegistry.addLW(pdp, "PDH"); // Only sent to NT in Test mode.

		// Get information about the match environment from the Field Control System.
      
		getMatchInformation();

		// Read properties file from RoboRio "disk". If we fail to open the file,
		// log the exception but continue and default to competition robot.
      
		try {
			robotProperties = Util.readProperties();
		} catch (Exception e) { Util.logException(e);}

		// Is this the competition or clone robot?
   		
		if (robotProperties == null || robotProperties.getProperty("RobotId").equals("comp"))
			isComp = true;
		else
			isClone = true;
 		
		// Set compressor enabled switch on dashboard from properties file.
		// Later code will read that setting from the dashboard and turn 
		// compressor on or off in response to dashboard setting.
 		
		boolean compressorEnabled = true;	// Default if no property.

		if (robotProperties != null) 
			compressorEnabled = Boolean.parseBoolean(robotProperties.getProperty("CompressorEnabledByDefault"));
		
		SmartDashboard.putBoolean("CompressorEnabled", compressorEnabled);

		// Reset PDB & PCM sticky faults.
    
		resetFaults();

		// Create NavX object here since must done before CameraFeed is created (don't remember why).
        // Navx calibrates at power on and must complete before robot moves. Takes ~1 second for 2nd
        // generation Navx ~15 seconds for classic Navx. We assume there will be enough time between
        // power on and our first movement because normally things don't happen that fast

		// Warning: The navx instance is shared with the swerve drive code. Resetting or otherwise
		// manipulating the navx (as opposed to just reading data) may crash the swerve drive code.

		navx = NavX.getInstance();

		// Add navx as a Sendable. Updates the dashboard heading indicator automatically.
 		
		SmartDashboard.putData("Gyro2", navx);

		// Invert driving joy sticks Y axis so + values mean forward.
		// Invert driving joy sticks X axis so + values mean right.
	  
		driverController.invertY(true);
		driverController.invertX(true);		

		// Create subsystems prior to button mapping.

		shuffleBoard = new ShuffleBoard();
		driveBase = new DriveBase();
		pvTagCamera = new PhotonVision(CORAL_CAMERA_TAG, PipelineType.POSE_ESTIMATION, CORAL_CAMERA_TAG_TRANSFORM);
		// pvAlgaeTagCamera = new PhotonVision(ALGAE_CAMERA_TAG, PipelineType.POSE_ESTIMATION, ALGAE_CAMERA_TAG_TRANSFORM);
		algaeManipulator = new AlgaeManipulator();
		coralManipulator = new CoralManipulator();
		elevator = new Elevator(driveBase);
		climber = new Climber();
		algaeGroundIntake = new AlgaeGroundIntake();
		// coralGroundIntake = new CoralGroundIntake();
		elevatedManipulator = new ElevatedManipulator(coralManipulator, 
														// coralGroundIntake, 
														algaeManipulator, 
														algaeGroundIntake, 
														elevator);
		
		// if (RobotBase.isReal()) 
		// {
		// 	candle = new Candle(CTRE_CANDLE, 8+26);
		// 	candle.setDefaultCommand(new UpdateCandle(candle));
		// }

		// Create any persistent commands.

		// Set any subsystem Default commands.

		// This sets up the photonVision subsystem to constantly update the robotDrive odometry
	    // with AprilTags (if it sees them). (As well as vision simulator)

		pvTagCamera.setDefaultCommand(new UpdateVisionPose(pvTagCamera, driveBase));
		// pvAlgaeTagCamera.setDefaultCommand(new UpdateVisionPose(pvAlgaeTagCamera, driveBase));

		// Set the default drive command. This command will be scheduled automatically to run
		// every teleop period and so use the gamepad joy sticks to drive the robot. 

		// We pass the GetY() functions on the Joysticks as a DoubleSuppier. The point of this 
		// is removing the direct connection between the Drive and XboxController classes. We
		// are in effect passing functions into the Drive command so it can read the values
		// later when the Drive command is executing under the Scheduler. Drive command code does
		// not have to know anything about the JoySticks (or any other source) but can still read
		// them. We can pass the DoubleSupplier two ways. First is with () -> lambda expression
		// which wraps the getLeftY() function in a DoubleSupplier instance. Second is using the
		// controller class convenience method getRightYDS() which returns getRightY() as a 
		// DoubleSupplier. We show both ways here as an example.

		// The joystick controls for driving:
		// Left stick Y axis -> forward and backwards movement (throttle)
		// Left stick X axis -> left and right movement (strafe)
		// Right stick X axis -> rotation
		// Note: X and Y axis on stick is opposite X and Y axis on the WheelSpeeds object
		// and the odometry pose2d classes.
		// Wheelspeeds +X axis is down the field away from alliance wall. +Y axis is left
		// when standing at alliance wall looking down the field.
		// This is handled here by swapping the inputs. Note that first axis parameter below
		// is the X wheelspeeds input and the second is Y wheelspeeds input.

		// Note that field oriented driving does the movements in relation to the field. So
		// throttle is always down the field and back and strafe is always left right from
		// the down the field axis, no matter which way the robot is pointing. Robot oriented
		// driving movemments are in relation to the direction the robot is currently pointing.

		// Note that the controller instance is passed to the drive command for use in displaying
		// debugging information on Shuffleboard. It is not required for the driving function.

		driveBase.setDefaultCommand(new DriveCommand(driveBase,
		 							() -> driverController.getLeftY(),
									driverController.getLeftXDS(), 
									driverController.getRightXDS(),
									driverController));
		
		// elevatedManipulator.setDefaultCommand(new RunCommand(
		//  	()->{elevatedManipulator.moveRelative(-MathUtil.applyDeadband(utilityController.getLeftY() * 0.1, DRIVE_DEADBAND));
		//  	}, elevatedManipulator));
		
		elevator.setDefaultCommand(new RunCommand(
		 	()->{elevator.move(-MathUtil.applyDeadband(utilityController.getLeftY() * 0.5, DRIVE_DEADBAND));
		 	}, elevator));
		//Start the compressor, PDP and camera feed monitoring Tasks.

   		//monitorCompressorThread = MonitorCompressorPH.getInstance(pcm);
   		//monitorCompressorThread.setDelay(1.0);
   		//monitorCompressorThread.SetLowPressureAlarm(50);
   		//monitorCompressorThread.start();
   		//
   		//monitorPDPThread = MonitorPDP.getInstance(pdp);
   		//monitorPDPThread.start();
   		//
		//pdp.setSwitchableChannel(true);
		
		// Start camera server thread using our class for usb cameras.
    
		if (RobotBase.isReal())
		{
			cameraFeed = CameraFeed.getInstance(); 
			cameraFeed.start();
		} 

		// Start a thread that will wait 30 seconds then disable the missing
		// joystick warning. This is long enough for when the warning is valid
		// but will stop flooding the console log when we are legitimately
		// running without both joysticks plugged in.

		new Thread(() -> {
			try {
				Timer.delay(30);    
	  
				DriverStation.silenceJoystickConnectionWarning(true);
			} catch (Exception e) { }
		  }).start();

		// Log info about NavX.
	  
		navx.dumpValuesToNetworkTables();
 		
		if (navx.isConnected())
			Util.consoleLog("NavX connected version=%s", navx.getAHRS().getFirmwareVersion());
		else
		{
			Exception e = new Exception("NavX is NOT connected!");
			Util.logException(e);
		}
        
        // Configure autonomous routines and send to dashboard.
		
		setAutoChoices();

		// Configure the button bindings.
		
        configureButtonBindings();
        
        // Load any trajectory files in a separate thread on first scheduler run.
        // We do this because trajectory loads can take up to 10 seconds to load so we want this
        // being done while we are getting started up. Hopefully will complete before we are ready to
        // use the trajectory.
		
		// NotifierCommand loadTrajectory = new NotifierCommand(this::loadTestTrajectory, 0);
        // loadTrajectory.setRunWhenDisabled(true);
        // CommandScheduler.getInstance().schedule(loadTrajectory);
		
		// //testTrajectory = loadTrajectoryFile("Slalom-1.wpilib.json");
		
		// loadTrajectory = new NotifierCommand(this::loadPPTestTrajectory, 0);
        // loadTrajectory.setRunWhenDisabled(true);
        // CommandScheduler.getInstance().schedule(loadTrajectory);

		//PathPlannerTrajectory ppTestTrajectory = loadPPTrajectoryFile("richard");

		Util.consoleLog(functionMarker);
	}

	/**
	 * Use this method to define your button->command mappings.
     * 
     * These buttons are for real robot driver station with 3 sticks and launchpad.
	 * The launchpad makes the colored buttons look like a joystick.
	 */
	private void configureButtonBindings() 
	{
		Util.consoleLog();
	  
		// ------- Driver pad buttons -------------
		
		// For simple functions, instead of creating commands, we can call convenience functions on
		// the target subsystem from an InstantCommand. It can be tricky deciding what functions
		// should be an aspect of the subsystem and what functions should be in Commands...

		// POV buttons do same as alternate driving mode but without any lateral
		// movement and increments of 45deg.
		// new Trigger(()-> driverController.getPOV() != -1)
		// 	.onTrue(new PointToYaw(()->PointToYaw.yawFromPOV(driverController.getPOV()), driveBase, false))

		// vibrate between 30 and 25 sec left in match.
		new Trigger(() -> Timer.getMatchTime() < 30 && Timer.getMatchTime() > 25).whileTrue(new StartEndCommand(
			() -> {
				driverController.setRumble(RumbleType.kBothRumble, 0.5);
				utilityController.setRumble(RumbleType.kBothRumble, 0.5);},
			() -> {
				driverController.setRumble(RumbleType.kBothRumble, 0);
				utilityController.setRumble(RumbleType.kBothRumble, 0);
		}));

		// holding top right bumper enables the alternate rotation mode in
		// which the driver points stick to desired heading.

		//new Trigger(() -> driverController.getRightBumperButton())
		//	.whileTrue(new PointToYaw(
		//		()->PointToYaw.yawFromAxes(
		//			-MathUtil.applyDeadband(driverController.getRightX(), Constants.DRIVE_DEADBAND),
		//			-MathUtil.applyDeadband(driverController.getRightY(), Constants.DRIVE_DEADBAND)
		//		), driveBase, false
		//));

		// toggle slow-mode
		new Trigger(() -> driverController.getLeftBumperButton())
			.onTrue(new InstantCommand(driveBase::enableSlowMode))
			.onFalse(new InstantCommand(driveBase::disableSlowMode));

		// reset field orientation (direction).
		new Trigger(() -> driverController.getStartButton())
			.onTrue(new InstantCommand(driveBase::zeroGyro));

		// toggle field-oriented driving mode.
		new Trigger(() -> driverController.getBackButton())
			.onTrue(new InstantCommand(driveBase::toggleFieldRelative));

		//Holding Right D-Pad button sets X pattern to stop movement.
		new Trigger(() -> driverController.getPOV() == 90)
				.onTrue(new RunCommand(() -> driveBase.setX(), driveBase));

		// toggle brake mode

		
		// //Drive to the Left Branch, offsetting from AprilTag (using Pitch/Yaw information)
		// new Trigger(() -> driverController.getLeftTrigger())
		// 	.onTrue(new DriveToLeft(driveBase, pvCoralTagCamera, true, true));
		
		// //Drive to the Right Branch, offsetting from AprilTag (using Pitch/Yaw information)
		// new Trigger(() -> driverController.getRightTrigger())
		// 	.onTrue(new DriveToRight(driveBase, pvCoralTagCamera, true, true));

 		//Drive to the AprilTag
		new Trigger(() -> driverController.getXButton())
			.whileTrue(new DriveToTag(driveBase, pvTagCamera, true, true));

		new Trigger(() -> driverController.getRightTrigger())
			.whileTrue(new DriveToRight(driveBase, pvTagCamera));
		new Trigger(() -> driverController.getLeftTrigger())
			.whileTrue(new DriveToLeft(driveBase, pvTagCamera, true, true));
		
    	// Drive to the AprilTag using Pose information
		 //new Trigger(()-> driverController.getLeftTrigger())
		 //	.onTrue(new SetTagBasedPosition(driveBase, pvTagCamera, 0, false));
		 	//.andThen(new RotateToPose(driveBase, true, true))
		 	//.andThen(new GoToPose(driveBase, true, true)));

         
		//COMMENTED OUT FOR BACKUP
// 		 new Trigger(()-> driverController.getLeftTrigger())
// 		          .onTrue(new SetTagBasedPosition(driveBase, pvCoralTagCamera, 0));
// 		 //                  .andThen(new RotateToPose(driveBase, true, true)));

//         new Trigger(()-> driverController.getRightBumperButton())
// 		 	.whileTrue(new RotateToPose(driveBase, true, true)
// 		 	// .andThen(new GoToPose(driveBase, true, true)));
// 		 	//.whileTrue(new GoToPose(driveBase, true, true));
// 			// .whileTrue(new RotateToTag(pvCoralTagCamera, driveBase));
// 			.andThen(new GoToTag(driveBase, true, true, pvCoralTagCamera)));

// 		new Trigger(() -> driverController.getRightBumperButton())
// 			.onFalse(new InstantCommand(() -> driveBase.setRotatedToTargetPose(false)));
			
		// //Drive to the Right Branch, offsetting from AprilTag (using Pose information)
		// new Trigger(()-> driverController.getRightTrigger())
		// 	// .whileTrue(new SetTagBasedPosition(driveBase, pvTagCamera, 1)
		// 	// .andThen(new RotateToPose(driveBase, true, true))
		// 	// .andThen(new GoToPose(driveBase, true, true)));
		// 	.onTrue(new InstantCommand(() -> driveBase.setFieldRelative(false)))
        //     .onFalse(new InstantCommand(() -> driveBase.setFieldRelative(true)));

		// // Drive to the Right Branch, offsetting from AprilTag (using Pose information)
		// // new Trigger(()-> driverController.getLeftTrigger())
		// // 	.whileTrue(new SetTagBasedPosition(driveBase, pvTagCamera, -1)
		// // 	.andThen(new RotateToPose(driveBase, true, true))
		// // 	.andThen(new GoToPose(driveBase, true, true)));
		
		new Trigger(() -> driverController.getBButton())
			.onTrue(new ParallelCommandGroup(new InstantCommand(() -> elevatedManipulator.executeSetPosition(PresetPosition.CLIMB), elevatedManipulator),
                new ExtendClimber(climber),
				new InstantCommand(() -> algaeManipulator.extendOut())));

		new Trigger(() -> driverController.getAButton())
    		.onTrue(new RetractClimber(climber));

        new Trigger(() -> driverController.getXButton())
            .onTrue(new IntakeAlgaeGround(elevatedManipulator));

        // new Trigger(() -> driverController.getYButton())
        //     .onTrue(new InstantCommand(() -> algaeGroundIntake.stop()));		
			
		// -------- Utility pad buttons ----------

		//Use Preset Command for the following:

		// Moves the coral manipulator/elevator to the L1 Branch scoring position
		new Trigger(() -> utilityController.getXButton())
		.onTrue(new Preset(elevatedManipulator, PresetPosition.CORAL_SCORING_L1_NEW));

		// Moves the coral manipulator/elevator to the L2 Branch scoring position.
		new Trigger(() -> utilityController.getAButton())
		.onTrue(new Preset(elevatedManipulator, PresetPosition.CORAL_SCORING_L2));

		// Moves the coral manipulator/elevator to the L3 Branch scoring position.
		new Trigger(() -> utilityController.getBButton())
		.onTrue(new Preset(elevatedManipulator, PresetPosition.CORAL_SCORING_L3));

		// Moves the coral manipulator/elevator to the L4 Branch scoring position.
		new Trigger(() -> utilityController.getYButton())
		.onTrue(new Preset(elevatedManipulator, PresetPosition.CORAL_SCORING_L4));

		
		// Moves the coral manipulator/elevator to the intake position for the coral station and runs the intake until it has coral.
		new Trigger(() -> utilityController.getLeftTrigger())
			.whileTrue(new IntakeCoral(elevatedManipulator))
			.onFalse(new InstantCommand(() -> elevatedManipulator.coralManipulator.stop()));
			
		//If the algae manipulator is in one of the removing positions, it will use the same intake button to remove algae.
		// new Trigger(()-> utilityController.getLeftTrigger() && !elevatedManipulator.intakeCoralInsteadOfAlgae)
		// 	.onTrue(new RemoveAlgae(elevatedManipulator));
		

		//Moves the algae Manipulator/elevator to the removing position for Algae on L3
		new Trigger(()-> utilityController.getPOV() == 0)
		.onTrue(new ParallelCommandGroup(new Preset(elevatedManipulator, PresetPosition.ALGAE_REMOVE_L3), 
			new InstantCommand(() -> elevatedManipulator.intakeCoralInsteadOfAlgae = false)));
			// .onTrue(new Preset(elevatedManipulator, PresetPosition.ALGAE_REMOVE_L3));


		//Moves the algae Manipulator/Elevator to the removing position for Algae on L2
		new Trigger(()-> utilityController.getPOV() == 180)
		.onTrue(new ParallelCommandGroup(new Preset(elevatedManipulator, PresetPosition.ALGAE_REMOVE_L2), 
			new InstantCommand(() -> elevatedManipulator.intakeCoralInsteadOfAlgae = false)));

		//Moves the elevator and algae manipulator to the scoring position for the algae net.
		new Trigger(()-> utilityController.getPOV() == 90)
		.onTrue(new ParallelCommandGroup(new Preset(elevatedManipulator, PresetPosition.ALGAE_NET_SCORING), 
			new InstantCommand(() -> elevatedManipulator.scoreCoralInsteadOfAlgae = false)));
		
		//Moves the elevator and algae manipulator to the scoring position for the algae processor.
		new Trigger(()-> utilityController.getPOV() == 0)
		.onTrue(new ParallelCommandGroup(new Preset(elevatedManipulator, PresetPosition.ALGAE_PROCESSOR_SCORING), 
			new InstantCommand(() -> elevatedManipulator.scoreCoralInsteadOfAlgae = false)));
		
		//Runs coral outtake if the elevator and manipulator are in the correct position.
		new Trigger(() -> utilityController.getRightTrigger() && elevatedManipulator.scoreCoralInsteadOfAlgae)
			.onTrue(new OuttakeCoral(elevatedManipulator));
		
		// Runs algae outtake if the elevator and manipulator are in the correct position.
		new Trigger(() -> utilityController.getRightTrigger() && !elevatedManipulator.scoreCoralInsteadOfAlgae)
			.onTrue(new OuttakeAlgae(elevatedManipulator));

		// new Trigger(() -> utilityController.getLeftBumperButton() )
		// 	.onTrue(new Preset(elevatedManipulator, PresetPosition.ALGAE_GROUND_INTAKE));

		//Moves the elvator and manipulator to the reset position and extends out ground intake, and algae manipulator, and starts intaking.
		new Trigger(() -> utilityController.getLeftBumperButton())
			.whileTrue(new RemoveAlgae(elevatedManipulator));

		new Trigger(() -> utilityController.getLeftTrigger() && elevatedManipulator.intakeCoralInsteadOfAlgae)
			.onTrue(new OuttakeCoral(elevatedManipulator));

		new Trigger(() -> utilityController.getRightTrigger() && !elevatedManipulator.scoreCoralInsteadOfAlgae)
			.onTrue(new OuttakeAlgae(elevatedManipulator));
		// new Trigger(() -> utilityController.getLeftBumperButton() && elevatedManipulator.hasAlgae() == false)
		// 	.whileTrue(new RemoveAlgae(elevatedManipulator))
		// 	.onFalse(new InstantCommand(algaeManipulator::stop));

		// new Trigger(() -> utilityController.getLeftBumperButton() && elevatedManipulator.hasAlgae() == true)
		// 	.toggleOnTrue(new InstantCommand(() -> elevatedManipulator.algaeManipulator.holdAlgae()));

		new Trigger(() -> utilityController.getRightBumperButton())
			.onTrue(new OuttakeAlgae(elevatedManipulator));
		
		 //Resets the manipulators and elevator to the default position.
		new Trigger(() -> utilityController.getBackButton())
			.onTrue(new Preset(elevatedManipulator, PresetPosition.RESET));
		
		new Trigger(() -> utilityController.getStartButton())
			.onTrue(new InstantCommand(elevator::resetEncoders));
		
		
			
		
	}
	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 * Determines which auto command from the selection made by the operator on the
	 * DS drop down list of commands.
	 * @return The Command to run in autonomous.
	 */
	public Command getAutonomousCommand() {
		// PathPlannerAuto  	ppAutoCommand;
		Command				autoCommand;

		autoCommand = autoChooser.getSelected();

		if (autoCommand == null) 
		{
			autonomousCommandName = "none";

			return autoCommand;
		}

		autonomousCommandName = autoCommand.getName();

		Util.consoleLog("auto name=%s", autonomousCommandName);

		if (autoCommand instanceof PathPlannerAuto)
		{
			// ppAutoCommand = (PathPlannerAuto) autoCommand;
	
			// Util.consoleLog("pp starting pose=%s", PathPlannerAuto.getStaringPoseFromAutoFile(autoCommand.getName().toString()));
		}

		return autoCommand;
  	}

	public static String getAutonomousCommandName()
	{
		return autonomousCommandName;
	}
  
    // Configure SendableChooser (drop down list on dashboard) with auto program choices and
	// send them to SmartDashboard/ShuffleBoard.
	
	private void setAutoChoices()
	{
	 	Util.consoleLog();
		
		// Register commands called from PathPlanner Autos.

		NamedCommands.registerCommand("Intake Coral", new IntakeCoral(elevatedManipulator));
		NamedCommands.registerCommand("Outtake Coral", new OuttakeCoral(elevatedManipulator));
		NamedCommands.registerCommand("Remove Algae", new RemoveAlgae(elevatedManipulator));
		NamedCommands.registerCommand("Outtake Algae", new OuttakeAlgae(elevatedManipulator));
		NamedCommands.registerCommand("Outtake Processor", new OuttakeProcessor(elevatedManipulator));
		NamedCommands.registerCommand("Raise to L1", new ParallelCommandGroup(new InstantCommand(() -> elevatedManipulator.executeSetPosition(PresetPosition.CORAL_SCORING_L1_NEW), elevatedManipulator),
				new InstantCommand(() -> coralManipulator.pivotUp())));
		NamedCommands.registerCommand("Raise to L2", new ParallelCommandGroup(new InstantCommand(() -> elevatedManipulator.executeSetPosition(PresetPosition.CORAL_SCORING_L2), elevatedManipulator),
				new InstantCommand(() -> coralManipulator.pivotDown())));
		NamedCommands.registerCommand("Raise to L3", new ParallelCommandGroup(new InstantCommand(() -> elevatedManipulator.executeSetPosition(PresetPosition.CORAL_SCORING_L3), elevatedManipulator),
				new InstantCommand(() -> coralManipulator.pivotDown())));
		NamedCommands.registerCommand("Raise to L4", new ParallelCommandGroup(new InstantCommand(() -> elevatedManipulator.executeSetPosition(PresetPosition.CORAL_SCORING_L4), elevatedManipulator),
				new InstantCommand(() -> coralManipulator.pivotDown())));
		NamedCommands.registerCommand("Remove Algae L2", new Preset(elevatedManipulator, PresetPosition.ALGAE_REMOVE_L2));
		NamedCommands.registerCommand("Remove Algae L3", new Preset(elevatedManipulator, PresetPosition.ALGAE_REMOVE_L3)); 
		NamedCommands.registerCommand("Algae Net Scoring", new Preset(elevatedManipulator, PresetPosition.ALGAE_NET_SCORING)); 
		NamedCommands.registerCommand("Algae Processor Scoring", new Preset(elevatedManipulator, PresetPosition.ALGAE_PROCESSOR_SCORING));
		NamedCommands.registerCommand("Intake Algae Ground", new IntakeAlgaeGround(elevatedManipulator));
		NamedCommands.registerCommand("Reset Elevator", new Preset(elevatedManipulator, PresetPosition.RESET));
		//NamedCommands.registerCommand("Align Left", new SetTagBasedPosition(driveBase, pvTagCamera, -1)
		//												.andThen(new RotateToPose(driveBase, true, true)
		//												.andThen(new GoToPose(driveBase, true, true))));
		//NamedCommands.registerCommand("Align Right", new SetTagBasedPosition(driveBase, pvTagCamera, 1)
		//												.andThen(new RotateToPose(driveBase, true, true))
		//												.andThen(new GoToPose(driveBase, true, true)));
		//NamedCommands.registerCommand("Align Center", new SetTagBasedPosition(driveBase, pvTagCamera, 0)
		//												.andThen(new RotateToPose(driveBase, true, true))
		//												.andThen(new GoToPose(driveBase, true, true)));
		NamedCommands.registerCommand("Algae Pivot Up", new InstantCommand(() -> algaeManipulator.pivotUp()));
		NamedCommands.registerCommand("Climb", new Preset(elevatedManipulator, PresetPosition.CLIMB));
		// Create a chooser with the PathPlanner Autos located in the PP
		// folders.

	    autoChooser = AutoBuilder.buildAutoChooser();
		
    	SmartDashboard.putData("Auto Program", autoChooser);
	}

	/**
	 *  Get and log information about the current match from the FMS or DS.
	 */
	public void getMatchInformation()
	{
		alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
  	  	location = DriverStation.getLocation().orElse(0);
  	  	eventName = DriverStation.getEventName();
	  	matchNumber = DriverStation.getMatchNumber();
	  	gameMessage = DriverStation.getGameSpecificMessage();
    
	  	Util.consoleLog("Alliance=%s, Location=%d, FMS=%b event=%s match=%d msg=%s", 
    		  		   alliance.name(), location, DriverStation.isFMSAttached(), eventName, matchNumber, 
    		  		   gameMessage);
	}
		
	/**
	 * Reset sticky faults in PDP and PCM and turn compressor on/off as
	 * set by switch on DS.
	 */
	public void resetFaults()
	{
		// This code turns on/off the automatic compressor management if requested by DS. Putting this
		// here is a convenience since this function is called at each mode change.
		if (SmartDashboard.getBoolean("CompressorEnabled", true)) 
			pcm.enableDigital();
		else
			pcm.disable();
		
		pdp.clearStickyFaults();
		//pcm.clearAllStickyFaults(); // Add back if we use a CTRE pcm.
		
		if (monitorPDPThread != null) monitorPDPThread.reset();
    }

	public void fixPathPlannerGyro() {
		driveBase.fixPathPlannerGyro();
	}

	/**
     * Loads a PathPlanner path file into a path planner trajectory.
     * @param fileName Name of file. Will automatically look in deploy directory and add the .path ext.
     * @return The path's trajectory.
     */
    // public static PathPlannerTrajectory loadPPTrajectoryFile(String fileName)
    // {
    //     PathPlannerTrajectory  	trajectory;
    //     Path        			trajectoryFilePath;

	// 	// We fab up the full path for tracing but the loadPath() function does it's own
	// 	// thing constructing a path from just the filename.
	// 	trajectoryFilePath = Filesystem.getDeployDirectory().toPath().resolve("pathplanner/" + fileName + ".path");

	// 	Util.consoleLog("loading PP trajectory: %s", trajectoryFilePath);
		
	// 	trajectory = PathPlanner.loadPath(fileName,
	// 									  new PathConstraints(MAX_WHEEL_SPEED, MAX_WHEEL_ACCEL));

	// 	if (trajectory == null) 
	// 	{
	// 		Util.consoleLog("Unable to open pp trajectory: " + fileName);
	// 		throw new RuntimeException("Unable to open PP trajectory: " + fileName);
	// 	}

    //     Util.consoleLog("PP trajectory loaded: %s", fileName);

    //     return trajectory;
    // }

	// private void loadPPTestTrajectory()
	// {
	// 	ppTestTrajectory = loadPPTrajectoryFile("Test-Path");
	// }
}
