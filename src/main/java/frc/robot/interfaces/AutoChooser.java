package frc.robot.interfaces;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Autos.McDonaldsPath;
import frc.robot.commands.Autos.TwoConeLeft;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoChooser {
	public enum AutoMode {
		ONE_CUBE,
		TWO_CONE_LEFT
	}

	private SendableChooser<AutoMode> chooser;
	ShuffleboardTab driverDashboard = Shuffleboard.getTab("Driver Dashboard");
	ShuffleboardLayout auto = driverDashboard.getLayout("Auto Mode", "List Layout")
			.withPosition(0, 3)
			.withSize(2, 1);
	
	public AutoChooser(){
		chooser = new SendableChooser<>();
		chooser.setDefaultOption("ONE CUBE", AutoMode.ONE_CUBE);
		chooser.addOption("TWO CONE LEFT", AutoMode.TWO_CONE_LEFT);
		auto.add(chooser);
	}

	public Command startMode(SwerveSubsystem swerve){
		AutoMode mode = (AutoMode)(chooser.getSelected());
		switch(mode){
			case ONE_CUBE:
				return new McDonaldsPath(swerve);
			case TWO_CONE_LEFT:
				return new TwoConeLeft(swerve);
			default:
				return new McDonaldsPath(swerve);
		}
	}

}
