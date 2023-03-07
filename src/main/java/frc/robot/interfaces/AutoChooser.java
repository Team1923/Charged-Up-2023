package frc.robot.interfaces;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Autos.McDonaldsPath;
import frc.robot.commands.Autos.ScoreCenterAndBalance;
import frc.robot.commands.Autos.TwoConeBalanceNonCableProtector;
import frc.robot.commands.Autos.TwoHalfConeBalanceNonCableProtector;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoChooser {
	public enum AutoMode {
		ONE_CUBE,
		TWO_CONE_NOCP,
		TWO_HALF_CONE_NOCP,
		CENTER_BALANCE,
		LEFT_BALANCE,
		RIGHT_BALANCE,
	}

	private SendableChooser<AutoMode> chooser;
	ShuffleboardTab driverDashboard = Shuffleboard.getTab("Driver Dashboard");
	ShuffleboardLayout auto = driverDashboard.getLayout("Auto Mode", "List Layout")
			.withPosition(0, 3)
			.withSize(2, 1);
	
	public AutoChooser(){
		chooser = new SendableChooser<>();
		chooser.setDefaultOption("ONE CUBE", AutoMode.ONE_CUBE);
		chooser.addOption("TWO CONE BALANCE NO CP", AutoMode.TWO_CONE_NOCP);
		chooser.addOption("TWO HALF CONE BALANCE", AutoMode.TWO_HALF_CONE_NOCP);
		auto.add(chooser);
	}

	public Command startMode(SwerveSubsystem swerve, IntakeSubsystem intake){
		AutoMode mode = (AutoMode)(chooser.getSelected());
		switch(mode){
			case ONE_CUBE:
				return new McDonaldsPath(swerve);
			case TWO_CONE_NOCP:
				return new TwoConeBalanceNonCableProtector(swerve, intake);
			case TWO_HALF_CONE_NOCP:
				return new TwoHalfConeBalanceNonCableProtector(swerve, intake);
			case CENTER_BALANCE:
				return new ScoreCenterAndBalance(swerve);
			default:
				return new McDonaldsPath(swerve);
		}
	}

}
