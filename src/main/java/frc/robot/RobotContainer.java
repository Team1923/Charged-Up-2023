// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ArmDefaultCommand;
import frc.robot.commands.ArmToPosition;
import frc.robot.commands.ArmToPositionCartesian;
import frc.robot.commands.ArmToPositionCartesianOld;
import frc.robot.commands.ArmToPositionCartesianSine;
import frc.robot.commands.ChangePipelineCommand;
import frc.robot.commands.ElbowToPosition;
import frc.robot.commands.SequentialArmToPosition;
import frc.robot.commands.ShoulderToPosition;
import frc.robot.interfaces.LimelightInterface;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private ArmSubsystem armSubsystem = new ArmSubsystem();
  private SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  // Controller
  private final XboxController controller = new XboxController(0);
  private final JoystickButton aButton = new JoystickButton(controller, 1);
  private final JoystickButton bButton = new JoystickButton(controller, 2);
  private final JoystickButton xButton = new JoystickButton(controller, 3);
  private final JoystickButton yButton = new JoystickButton(controller, 4);
  

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        
        break;

      // Replayed robot, disable IO implementations
      default:
        
        break;
    }

    // Set up auto routines
   

    // Configure the button bindings
    configureButtonBindings();

    setDefaultCommands();

    aButton.whileTrue(new ArmToPositionCartesianSine(armSubsystem, 1));
    bButton.whileTrue(new ArmToPositionCartesian(armSubsystem, -1, 1));
    yButton.whileTrue(new ArmToPositionCartesian(armSubsystem, 1, 1));
    xButton.whileTrue(new SequentialArmToPosition(armSubsystem));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  private void setDefaultCommands(){
    armSubsystem.setDefaultCommand(new ArmDefaultCommand(armSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
