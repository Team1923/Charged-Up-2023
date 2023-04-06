// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import frc.robot.Constants;
import frc.robot.interfaces.LimelightInterface;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class AlignToTag extends CommandBase {
    private SwerveSubsystem s_Swerve;
    private DoubleSupplier translationSup;
    
    PIDController rotationController = new PIDController(0.002, 0, 0);
    PIDController strafeController = new PIDController(0.01, 0, 0);

    LimelightInterface limelight = LimelightInterface.getInstance();


    public AlignToTag(SwerveSubsystem s_Swerve, DoubleSupplier translationSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);
        this.translationSup = translationSup;
      
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        s_Swerve.drive(new Translation2d(translationVal, -strafeController.calculate(limelight.getXOffset(), 0)).times(Constants.Swerve.maxSpeed),
                rotationController.calculate(s_Swerve.getYaw().getDegrees(),0) * Constants.Swerve.maxAngularVelocity, false, true);
    }

    @Override
    public void end(boolean interrupted) {

    }
}