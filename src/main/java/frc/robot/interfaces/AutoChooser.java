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
		SINGLE_SCORE,
		TWO_GP_NOCP,
		TWO_GP_NO_BALANCE,
		TWO_HALF_GP_NOCP,
		CENTER_BALANCE,
		LEFT_BALANCE,
		RIGHT_BALANCE,
		SCORE_BACK_OUT_NOCP,
		SCORE_BACK_OUT_OVERCP
	}

	private SendableChooser<AutoMode> chooser;
	ShuffleboardTab driverDashboard = Shuffleboard.getTab("Driver Dashboard");
	ShuffleboardLayout auto = driverDashboard.getLayout("Auto Mode", "List Layout")
			.withPosition(0, 3)
			.withSize(2, 1);
	
	public AutoChooser(){
		chooser = new SendableChooser<>();
		chooser.setDefaultOption("SINGLE SCORE", AutoMode.SINGLE_SCORE);
		chooser.addOption("TWO GAME PIECE NO BALANCE", AutoMode.TWO_GP_NO_BALANCE);
		chooser.addOption("TWO GAME PIECE BALANCE", AutoMode.TWO_GP_NOCP);
		chooser.addOption("TWO HALF GAME PIECE NO BALANCE", AutoMode.TWO_HALF_GP_NOCP);
		chooser.addOption("CENTER BALANCE", AutoMode.CENTER_BALANCE);
		chooser.addOption("SCORE BACK OUT NO CP", AutoMode.SCORE_BACK_OUT_NOCP);
		chooser.addOption("SCORE BACK OUT OVER CP", AutoMode.SCORE_BACK_OUT_OVERCP);
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
