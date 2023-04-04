// package frc.robot.commands;

// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.CommandBase;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import frc.robot.commands.SwerveCommands.SwerveXWheels;
// import frc.robot.subsystems.SwerveSubsystem;


// public class AutoBalance extends CommandBase {

//   private final double speedInchesPerSec = 15;
//   private final double positionThresholdDegrees = 3.0;
//   private final double velocityThresholdDegreesPerSec = 8.0;

//   private final SwerveSubsystem swerveSubsystem;
//   private double angleDegrees;

//   public AutoBalance(SwerveSubsystem swerve) {
//     this.swerveSubsystem = swerve;
//     addRequirements(swerve);
//   }

//   @Override
//   public void initialize() {
//     angleDegrees = Double.POSITIVE_INFINITY;
//   }

//   @Override
//   public void execute() {
//     // Calculate charge station angle and velocity
//     angleDegrees =
//         swerveSubsystem.getYaw().getCos() * swerveSubsystem.getPitch().getDegrees()
//             + swerveSubsystem.getYaw().getSin() * swerveSubsystem.getRoll().getDegrees();
//     double angleVelocityDegreesPerSec =
//         swerveSubsystem.getYaw().getCos() * Units.radiansToDegrees(swerveSubsystem.getPitchVelocity())
//             + swerveSubsystem.getYaw().getSin() * Units.radiansToDegrees(swerveSubsystem.getRollVelocity());
//     boolean shouldStop =
//         (angleDegrees < 0.0 && angleVelocityDegreesPerSec > velocityThresholdDegreesPerSec)
//             || (angleDegrees > 0.0
//                 && angleVelocityDegreesPerSec < -velocityThresholdDegreesPerSec);

//     // Send velocity to swerveSubsystem
//     if (shouldStop) {
//       swerveSubsystem.stop();
//     } else {
//       swerveSubsystem.drive(new Translation2d(Units.inchesToMeters(speedInchesPerSec) * (angleDegrees > 0.0 ? -1.0 : 1.0),
//       0),
//       0,
//        true,
//        true);
//     }

//     // Log data
//     // Logger.getInstance().recordOutput("AutoBalance/AngleDegrees", angleDegrees);
//     // Logger.getInstance()
//     //     .recordOutput("AutoBalance/AngleVelocityDegreesPerSec", angleVelocityDegreesPerSec);
//     // Logger.getInstance().recordOutput("AutoBalance/Stopped", shouldStop);
//   }

//   @Override
//   public void end(boolean interrupted) {
//    CommandScheduler.getInstance().schedule(new SwerveXWheels(swerveSubsystem));
//   }

//   @Override
//   public boolean isFinished() {
//     return Math.abs(angleDegrees) < positionThresholdDegrees;
//   }
// }
