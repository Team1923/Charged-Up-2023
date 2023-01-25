// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Scoring;

// import java.util.List;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.trajectory.Trajectory;
// import edu.wpi.first.math.trajectory.TrajectoryConfig;
// import edu.wpi.first.math.trajectory.TrajectoryGenerator;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.Constants.Swerve;
// import frc.robot.commands.SwerveCommands.AlignScoringCommand;
// import frc.robot.subsystems.SwerveSubsystem;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class TrajectoryToGoal extends SequentialCommandGroup {
//   public TrajectoryToGoal(SwerveSubsystem swerve) {
//     TrajectoryConfig config = new TrajectoryConfig(
//             AutoConstants.kMaxSpeedMetersPerSecond,
//             AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//     .setKinematics(Swerve.swerveKinematics);

//     config.setStartVelocity(0);
//     config.setEndVelocity(0);
//     config.setReversed(false); //TO-DO: check if correct orientation

//     /*Generate the trajectory based on waypoints */
    

//     var thetaController =
//         new ProfiledPIDController(
//             AutoConstants.kPThetaController, 0, 0.001, AutoConstants.kThetaControllerConstraints);
//     thetaController.enableContinuousInput(-Math.PI, Math.PI);

        
//     // AlignScoringCommand swerveControllerCommand =
//     //     new AlignScoringCommand(
//     //         trajectory,
//     //         swerve::getPose,
//     //         Swerve.swerveKinematics,
//     //         new PIDController(AutoConstants.kPXController, 0, 0),
//     //         new PIDController(AutoConstants.kPYController, 0, 0),
//     //         thetaController,
//     //         swerve::setModuleStates,
//     //         swerve);

    
//     addCommands(
//       new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose())),
//       swerveControllerCommand
//     );
//   }
  

  
// }
