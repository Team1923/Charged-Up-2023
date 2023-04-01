package frc.robot.interfaces;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.Autos.SingleScoreAuto;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.AutonInstantiate;

public class AutoChooser {
	public enum AutoMode {
		SINGLE_SCORE,
		FOUR_CUBE,
		FOUR_CUBE_SIDELINE,
		FIVE_CUBE,
		TWO_METER_TEST,
		TUNE_BALANCE, 
		THREE_CUBE,
		FIVE_CUBE_NO_BALANCE
	}

	private SendableChooser<AutoMode> chooser;
	ShuffleboardTab driverDashboard = Shuffleboard.getTab("Driver Dashboard");
	ShuffleboardLayout auto = driverDashboard.getLayout("Auto Mode", "List Layout")
			.withPosition(4, 0)
			.withSize(2, 1);
	
	public AutoChooser(){
		chooser = new SendableChooser<>();
		chooser.setDefaultOption("SINGLE SCORE", AutoMode.SINGLE_SCORE);
		chooser.addOption("FourCubeWithBalance", AutoMode.FOUR_CUBE);
		chooser.addOption("FiveCubeWithBalance", AutoMode.FIVE_CUBE);
		chooser.addOption("FourCubeWithBalanceSideLine", AutoMode.FOUR_CUBE_SIDELINE);
		chooser.addOption("TwoMeterTest", AutoMode.TWO_METER_TEST);
		chooser.addOption("TuneBalance", AutoMode.TUNE_BALANCE);
		chooser.addOption("THREE CUBE BALANCE", AutoMode.THREE_CUBE);
		chooser.addOption("FiveCubeNoBalance", AutoMode.FIVE_CUBE_NO_BALANCE);
		auto.add(chooser);
	}

	public Command startMode(SwerveSubsystem swerve, IntakeSubsystem intake){
		AutoMode mode = (AutoMode)(chooser.getSelected());
		switch(mode){
			case SINGLE_SCORE:
				return new SingleScoreAuto();
			case FOUR_CUBE:
				return AutonInstantiate.getInstance(swerve).get4CubeAuton();
			case FOUR_CUBE_SIDELINE:
				return AutonInstantiate.getInstance(swerve).get4CubeSideLineAuton();
			case FIVE_CUBE:
				return AutonInstantiate.getInstance(swerve).getFiveCubeAuton();
			case TWO_METER_TEST:
				return AutonInstantiate.getInstance(swerve).getTwoMeter();
			case TUNE_BALANCE:
				return AutonInstantiate.getInstance(swerve).getBalanceTuning();
			case THREE_CUBE:
				return AutonInstantiate.getInstance(swerve).getThreeCube();
			case FIVE_CUBE_NO_BALANCE:
				return AutonInstantiate.getInstance(swerve).getFiveCubeNoBalance();
			default:
				return new SingleScoreAuto();
		}
	}

}
