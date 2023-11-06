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
		FIVE_CUBE_NO_BALANCE, 
		FANCY_3_CUBE,
		NEW_3_CUBE_CHARGE,
		CURVED_THREE_CUBE,
		SCORE_CENTER_BALANCE, 
		TWO_CUBE_CURVED,
		FOUR_CUBE_WITH_BALANCE_MALL_RAT,
		THREE_CUBE_WITH_BALANCE_MALL_RAT,
		FIVE_CUBE_NO_BALANCE_MALL_RAT,
		NOTHING,
		COOL_CHARGE_STATION,
		THREE_CUBE_NB,
		SCORE_CENTER_BALANCE_MR;
	}

	private SendableChooser<AutoMode> chooser;
	ShuffleboardTab driverDashboard = Shuffleboard.getTab("Driver Dashboard");
	ShuffleboardLayout auto = driverDashboard.getLayout("Auto Mode", "List Layout")
			.withPosition(4, 0)
			.withSize(2, 1);
	
	public AutoChooser(){
		chooser = new SendableChooser<>();
		chooser.setDefaultOption("FOUR CUBE WITH BALANCE MALL RAT", AutoMode.FOUR_CUBE_WITH_BALANCE_MALL_RAT);
		chooser.addOption("THREE CUBE WITH BALANCE MALL RAT", AutoMode.THREE_CUBE_WITH_BALANCE_MALL_RAT);
		chooser.addOption("FIVE CUBE NO BALANCE MALL RAT", AutoMode.FIVE_CUBE_NO_BALANCE_MALL_RAT);
		chooser.addOption("NOTHING", AutoMode.NOTHING);
		chooser.addOption("THREE CUBE NO BALANCE MR", AutoMode.THREE_CUBE_NB);
		chooser.addOption("SCORE CENTER BALANCE MR", AutoMode.SCORE_CENTER_BALANCE_MR);

		chooser.setDefaultOption("SINGLE SCORE", AutoMode.SINGLE_SCORE);
		chooser.addOption("4CubeWithBalance", AutoMode.FOUR_CUBE);
		chooser.addOption("5CubeWithBalance", AutoMode.FIVE_CUBE);
		chooser.addOption("4CubeWithBalanceSideLine", AutoMode.FOUR_CUBE_SIDELINE);
		chooser.addOption("TwoMeterTest", AutoMode.TWO_METER_TEST);
		chooser.addOption("TuneBalance", AutoMode.TUNE_BALANCE);
		chooser.addOption("3 CUBE BALANCE", AutoMode.THREE_CUBE);
		chooser.addOption("5CubeNoBalance", AutoMode.FIVE_CUBE_NO_BALANCE);
		chooser.addOption("FANCY 3 CUBE", AutoMode.FANCY_3_CUBE);
		chooser.addOption("NEW 3 CUBE CHARGE STATION", AutoMode.NEW_3_CUBE_CHARGE);
		chooser.addOption("SCORE CENTER BALANCE", AutoMode.SCORE_CENTER_BALANCE);
		chooser.addOption("CURVED THREE CUBE", AutoMode.CURVED_THREE_CUBE);
		chooser.addOption("TWO CUBE CURVED", AutoMode.TWO_CUBE_CURVED);
		chooser.addOption("COOL CHARGE STATION NO ACQUIRE", AutoMode.COOL_CHARGE_STATION);
		auto.add(chooser);
	}

	public Command startMode(SwerveSubsystem swerve, IntakeSubsystem intake){
		AutoMode mode = (AutoMode)(chooser.getSelected());
		switch(mode){
			case FOUR_CUBE_WITH_BALANCE_MALL_RAT:
				return AutonInstantiate.getInstance(swerve).getFourCubeWithBalanceMallRat();

			case THREE_CUBE_WITH_BALANCE_MALL_RAT:
				return AutonInstantiate.getInstance(swerve).getThreeCubeWithBalanceMallRat();

			case FIVE_CUBE_NO_BALANCE_MALL_RAT:
				return AutonInstantiate.getInstance(swerve).getFiveCubeNoBalanceMallRat();
			case NOTHING:
				return null;
			case THREE_CUBE_NB:
				return AutonInstantiate.getInstance(swerve).getThreeCubeNBMall();
			case SCORE_CENTER_BALANCE_MR:
				return AutonInstantiate.getInstance(swerve).getScoreCenterBalanceMR();

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
			case FANCY_3_CUBE:
				return AutonInstantiate.getInstance(swerve).getFancy3Cube();
			case NEW_3_CUBE_CHARGE:
				return AutonInstantiate.getInstance(swerve).getNew3CubeChargeStation();
			case SCORE_CENTER_BALANCE:
			// 	return AutonInstantiate.getInstance(swerve).getScoreCenterBalance();
			// case CURVED_THREE_CUBE:
			// 	return AutonInstantiate.getInstance(swerve).getCurvedThreeCube();
			// case TWO_CUBE_CURVED:
			// 	return AutonInstantiate.getInstance(swerve).getTwoCubeCurved();
			// case COOL_CHARGE_STATION:
			// 	return AutonInstantiate.getInstance(swerve).getCoolChargeStationAuto();
			default:
				return null;
		}
	}

}
