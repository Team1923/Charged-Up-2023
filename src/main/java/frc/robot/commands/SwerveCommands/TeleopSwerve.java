// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SwerveCommands;

import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {
    private SwerveSubsystem s_Swerve;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public TeleopSwerve(SwerveSubsystem s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup,
            DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband */
        double translationVal = 0.2* MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = 0.2* MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = 0.20 * MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);

        /* Drive */
        s_Swerve.drive(new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.maxAngularVelocity, !robotCentricSup.getAsBoolean(), true);
    }
}
