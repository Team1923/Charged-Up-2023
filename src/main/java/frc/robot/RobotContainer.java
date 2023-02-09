package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ArmCommands.OldArmDefaultCommand;
import frc.robot.commands.ArmCommands.ArmDefaultCommand;
import frc.robot.commands.ArmCommands.ArmToPosition;
import frc.robot.commands.IntakeCommands.DeployIntakeCommand;
import frc.robot.commands.IntakeCommands.IntakeAndLiftCommand;
import frc.robot.commands.IntakeCommands.IntakeArmDefaultCommand;
import frc.robot.commands.IntakeCommands.IntakeSolenoidCommand;
import frc.robot.commands.IntakeCommands.StowIntakeCommand;
import frc.robot.commands.Scoring.ManipulatorDefaultCommand;
import frc.robot.commands.Scoring.ManualScore;
import frc.robot.commands.StateCommands.SetArmLocation;
import frc.robot.commands.StateCommands.SetGamePiece;
import frc.robot.commands.StateCommands.SetRobotLocation;
import frc.robot.commands.SwerveCommands.TeleopSwerve;
import frc.robot.subsystems.*;
import frc.robot.util.StateVariables.ArmPositions;
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
    private final Joystick test = new Joystick(2);

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton aButton = new JoystickButton(driver, XboxController.Button.kA.value);
    private final JoystickButton bButton = new JoystickButton(driver, XboxController.Button.kB.value);
    private final JoystickButton xButton = new JoystickButton(driver, XboxController.Button.kX.value);
    private final JoystickButton yButton = new JoystickButton(driver, XboxController.Button.kY.value);

    /* Operator Buttons */
    private final JoystickButton operatorSquareButton = new JoystickButton(operator, 3);
    private final JoystickButton operatorTriangleButton = new JoystickButton(operator, 49); //find
    private final JoystickButton operatorCircleButton = new JoystickButton(operator, 2);
    private final JoystickButton operatorCrossButton = new JoystickButton(operator,1);
    private final JoystickButton operatorLeftBumper = new JoystickButton(operator, 55); //find
    private final JoystickButton operatorRightBumper = new JoystickButton(operator, 56); //find
    private final POVButton operatorUpDPad = new POVButton(operator, 0);
    private final POVButton operatorRightDPad = new POVButton(operator, 90);
    private final POVButton operatorDownDPad = new POVButton(operator, 180);
    private final POVButton operatorLeftDPad = new POVButton(operator, 270);
    private final JoystickButton centerButton = new JoystickButton(operator, 57);


    /* Subsystems */
    private final SwerveSubsystem s_Swerve = new SwerveSubsystem();
    public final ArmSubsystem armSubsystem = new ArmSubsystem();
    private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
    private final ManipulatorSubsystem gripper = new ManipulatorSubsystem();

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
        zeroGyro.toggleOnTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));

        operatorUpDPad.onTrue(new SetArmLocation(VerticalLocations.HIGH));
        operatorRightDPad.onTrue(new SetArmLocation(VerticalLocations.RESET));
        operatorDownDPad.onTrue(new SetArmLocation(VerticalLocations.LOW));
        operatorLeftDPad.onTrue(new SetArmLocation(VerticalLocations.MID));

        centerButton.toggleOnTrue(new SetGamePiece());

        operatorTriangleButton.onTrue(new SetRobotLocation(HorizontalLocations.LEFT));
        operatorCircleButton.onTrue(new SetRobotLocation(HorizontalLocations.CENTER));
        operatorCrossButton.onTrue(new SetRobotLocation(HorizontalLocations.RIGHT));
        operatorSquareButton.onTrue(new SetRobotLocation(HorizontalLocations.RESET));

        //find axis for left trigger
        new Trigger(() -> operator.getRawAxis(58) > 0.2).onTrue(new ManualScore());

        operatorLeftBumper.onTrue(new DeployIntakeCommand());
        operatorRightBumper.onTrue(new StowIntakeCommand());

        
    }

    private void setDefaultCommands() {
        s_Swerve.setDefaultCommand(
                new TeleopSwerve(
                        s_Swerve,
                        () -> -driver.getRawAxis(translationAxis),
                        () -> -driver.getRawAxis(strafeAxis),
                        () -> -driver.getRawAxis(rotationAxis),
                        () -> robotCentric.getAsBoolean(),
                        () -> driver.getRawAxis(2)));

        intakeSubsystem.setDefaultCommand(new IntakeArmDefaultCommand(intakeSubsystem, () -> driver.getRawAxis(3)));
        armSubsystem.setDefaultCommand(new ArmDefaultCommand(armSubsystem));
        //find the operator axis for right trigger
        gripper.setDefaultCommand(new ManipulatorDefaultCommand(gripper, () -> operator.getRawAxis(59)));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return null;
    }
}
