package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.BalanceTuning;
import frc.robot.commands.Autos.Fancy3Cube;
import frc.robot.commands.Autos.FiveCubeNoBalance;
import frc.robot.commands.Autos.FiveCubeWithBalance;
import frc.robot.commands.Autos.FourCubeWithBalance;
import frc.robot.commands.Autos.FourCubeWithBalanceSideLine;
import frc.robot.commands.Autos.ThreeCubeWithBalance;
import frc.robot.commands.Autos.TwoMeterTest;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonInstantiate {

	public static AutonInstantiate autoInstantiate;
	private FourCubeWithBalance fourcubeAuton;
	private FiveCubeWithBalance fiveCube;
	private FourCubeWithBalanceSideLine fourcubeSideLineAuton;
	private SwerveSubsystem swerve;
	private TwoMeterTest twoMeterTest;
	private BalanceTuning tuneBalance;
	private ThreeCubeWithBalance threeCube;
	private FiveCubeNoBalance fiveCubeNoBalance;
	private Fancy3Cube fancy3Cube;

	public static synchronized AutonInstantiate getInstance(SwerveSubsystem s) {
		if (autoInstantiate == null) {
			autoInstantiate = new AutonInstantiate(s);
		}
		return autoInstantiate;
	}

	public AutonInstantiate(SwerveSubsystem swerve) {
		this.swerve = swerve;
		fiveCube = new FiveCubeWithBalance(this.swerve);
		fourcubeAuton = new FourCubeWithBalance(this.swerve);
		fourcubeSideLineAuton = new FourCubeWithBalanceSideLine(this.swerve);
		twoMeterTest = new TwoMeterTest(this.swerve);
		tuneBalance = new BalanceTuning(this.swerve);
		threeCube = new ThreeCubeWithBalance(swerve);
		fiveCubeNoBalance = new FiveCubeNoBalance(swerve);
		fancy3Cube = new Fancy3Cube(swerve);
	}

	public SequentialCommandGroup get4CubeSideLineAuton() {
		return fourcubeSideLineAuton;
	}

	public SequentialCommandGroup getFiveCubeAuton() {
		return fiveCube;
	}

	public SequentialCommandGroup get4CubeAuton() {
		return fourcubeAuton;
	}

	public SequentialCommandGroup getTwoMeter() {
		return twoMeterTest;
	}

	public SequentialCommandGroup getBalanceTuning() {
		return tuneBalance;
	}

	public SequentialCommandGroup getThreeCube() {
		return threeCube;
	}

	public SequentialCommandGroup getFiveCubeNoBalance(){
		return fiveCubeNoBalance;
	}

	public SequentialCommandGroup getFancy3Cube() {
		return fancy3Cube;
	}

}
