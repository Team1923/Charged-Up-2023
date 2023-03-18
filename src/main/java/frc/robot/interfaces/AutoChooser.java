package frc.robot.interfaces;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Autos.SingleScoreAuto;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoChooser {
	public enum AutoMode {
		SINGLE_SCORE
	}

	private SendableChooser<AutoMode> chooser;
	ShuffleboardTab driverDashboard = Shuffleboard.getTab("Driver Dashboard");
	ShuffleboardLayout auto = driverDashboard.getLayout("Auto Mode", "List Layout")
			.withPosition(4, 0)
			.withSize(2, 1);
	
	public AutoChooser(){
		chooser = new SendableChooser<>();
		chooser.setDefaultOption("SINGLE SCORE", AutoMode.SINGLE_SCORE);
		auto.add(chooser);
	}

	public Command startMode(SwerveSubsystem swerve, IntakeSubsystem intake){
		AutoMode mode = (AutoMode)(chooser.getSelected());
		switch(mode){
			case SINGLE_SCORE:
				return new SingleScoreAuto();
			default:
				return new SingleScoreAuto();
		}
	}

}
