package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmCommands.ArmDefaultCommand;
import frc.robot.commands.ArmCommands.ToggleArmPositionCommand;
import frc.robot.commands.ArmCommands.ToggleEmergencyArmRecovery;
import frc.robot.commands.IntakeCommands.AcquireFromHPCommand;
import frc.robot.commands.IntakeCommands.DeployIntakeCommand;
import frc.robot.commands.IntakeCommands.EjectPosition;
import frc.robot.commands.IntakeCommands.IntakeArmDefaultCommand;
import frc.robot.commands.IntakeCommands.StowIntakeCommand;
import frc.robot.commands.Scoring.ManipulatorDefaultCommand;
import frc.robot.commands.Scoring.ManualScore;
import frc.robot.commands.Scoring.SequentialScoringCommand;
import frc.robot.commands.Scoring.TrajectoryToGoal;
import frc.robot.commands.StateCommands.SetArmLocation;
import frc.robot.commands.StateCommands.SetGamePiece;
import frc.robot.commands.StateCommands.SetRobotLocation;
import frc.robot.commands.SwerveCommands.SwerveXWheels;
import frc.robot.commands.SwerveCommands.TeleopSwerve;
import frc.robot.interfaces.AutoChooser;
import frc.robot.subsystems.*;
import frc.robot.util.PathPlannerUtils.AutoFromPathPlanner;
import frc.robot.util.StateVariables.HorizontalLocations;
import frc.robot.util.StateVariables.VerticalLocations;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);

    //need this for rumble setup, DO NOT USE
    private final XboxController xboxDriverController = new XboxController(0);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton rightBumper = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton leftBumper = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton aButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton bButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton xButton = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton yButton = new JoystickButton(driver, XboxController.Button.kY.value);

    /* Operator Buttons */
    private final JoystickButton operatorSquareButton = new JoystickButton(operator, 3);
    private final JoystickButton operatorTriangleButton = new JoystickButton(operator, 4); //4
    private final JoystickButton operatorCircleButton = new JoystickButton(operator, 2);
    private final JoystickButton operatorCrossButton = new JoystickButton(operator,1);
    private final JoystickButton operatorLeftBumper = new JoystickButton(operator, 5); //5
    private final JoystickButton operatorRightBumper = new JoystickButton(operator, 6); //6
    private final POVButton operatorUpDPad = new POVButton(operator, 0);
    private final POVButton operatorRightDPad = new POVButton(operator, 90);
    private final POVButton operatorDownDPad = new POVButton(operator, 180);
    private final POVButton operatorLeftDPad = new POVButton(operator, 270);
    private final JoystickButton centerRightButton = new JoystickButton(operator, 8);
    private final JoystickButton centerLeftButton = new JoystickButton(operator, 7);


    /* Subsystems */
    public final SwerveSubsystem s_Swerve = new SwerveSubsystem();
    public final ArmSubsystem armSubsystem = new ArmSubsystem();
    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    public final ManipulatorSubsystem gripper = new ManipulatorSubsystem();
    public final ShuffleboardSubsystem shuffleboard = new ShuffleboardSubsystem();
    public final ControllerRumble controllerRumble = new ControllerRumble(xboxDriverController);

    final AutoFromPathPlanner test5MStrafeRight = new AutoFromPathPlanner(s_Swerve, "Test5mStrafeRight", 4.5, 3.5, false, false, true);
    final AutoFromPathPlanner test5mForward = new AutoFromPathPlanner(s_Swerve, "Test5mForward", 4.5, 3.5, false, false, true);
    final AutoFromPathPlanner TestSCurve = new AutoFromPathPlanner(s_Swerve, "TestSCurve", 4.5, 3.5, false, false, true);


    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        setDefaultCommands();

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        yButton.toggleOnTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        aButton.toggleOnTrue(new TrajectoryToGoal(s_Swerve));
        rightBumper.toggleOnTrue(new SequentialScoringCommand(s_Swerve));

        operatorUpDPad.onTrue(new SetArmLocation(VerticalLocations.HIGH));
        operatorDownDPad.onTrue(new SetArmLocation(VerticalLocations.LOW));
        operatorLeftDPad.onTrue(new SetArmLocation(VerticalLocations.MID));

        centerRightButton.toggleOnTrue(new SetGamePiece());

        operatorTriangleButton.onTrue(new SetRobotLocation(HorizontalLocations.CENTER));
        operatorCircleButton.onTrue(new SetRobotLocation(HorizontalLocations.RIGHT));
        operatorSquareButton.onTrue(new SetRobotLocation(HorizontalLocations.LEFT));

        new Trigger(() -> operator.getRawAxis(3) > 0.2).toggleOnTrue(new ManualScore());

        operatorLeftBumper.onTrue(new DeployIntakeCommand(intakeSubsystem));
        operatorRightBumper.onTrue(new StowIntakeCommand(intakeSubsystem, false));

        operatorRightDPad.onTrue(new ToggleArmPositionCommand());

        // operatorCrossButton.onTrue(new AcquireFromHPCommand());

        // leftBumper.onTrue(new EjectPosition());
        operatorCrossButton.onTrue(new EjectPosition());


        bButton.toggleOnTrue(new SwerveXWheels(s_Swerve));

    }

    private void setDefaultCommands() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> driver.getRawAxis(2) > 0.2,
                        () -> xButton.getAsBoolean()));

        intakeSubsystem.setDefaultCommand(new IntakeArmDefaultCommand(intakeSubsystem, () -> driver.getRawAxis(3), () -> centerLeftButton.getAsBoolean()));
       
        armSubsystem.setDefaultCommand(new ArmDefaultCommand(armSubsystem));
        //find the operator axis for right trigger
        gripper.setDefaultCommand(new ManipulatorDefaultCommand(gripper, () -> operator.getRawAxis(2)));

        // SmartDashboard.putData("EMERGENCY ARM RECOVERY", new ToggleEmergencyArmRecovery(
        //     armSubsystem,
        //     () -> Math.abs(operator.getRawAxis(1)) < 0.05 ? 0 : operator.getRawAxis(1),
        //     () -> Math.abs(operator.getRawAxis(5)) < 0.05 ? 0 : operator.getRawAxis(5)
        // ));

    }

    public Command initializeAuto(AutoChooser selector){
        return selector.startMode(s_Swerve, intakeSubsystem);
    }
}