package frc.robot.util;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Autos.FiveCubeWithBalance;
import frc.robot.commands.Autos.FourCubeWithBalance;
import frc.robot.commands.Autos.FourCubeWithBalanceSideLine;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class AutonInstantiate {

	public static AutonInstantiate autoInstantiate;
	private FourCubeWithBalance fourcubeAuton;
	private FiveCubeWithBalance fiveCube;
	private FourCubeWithBalanceSideLine fourcubeSideLineAuton;
	private SwerveSubsystem swerve;

	
	public static synchronized AutonInstantiate getInstance(SwerveSubsystem s) {
		if (autoInstantiate == null) {
		  autoInstantiate = new AutonInstantiate(s);
		}
		return autoInstantiate;
	  }

	  public AutonInstantiate (SwerveSubsystem swerve){
		this.swerve = swerve;
		fiveCube = new FiveCubeWithBalance(this.swerve);
		fourcubeAuton = new FourCubeWithBalance(this.swerve);
		fourcubeSideLineAuton = new FourCubeWithBalanceSideLine(this.swerve);
	  }

	  public SequentialCommandGroup get4CubeSideLineAuton(){
		return fourcubeSideLineAuton;
	  } 

	  public SequentialCommandGroup getFiveCubeAuton() {
		return fiveCube;

	  }

	  //As we are able to get more successful auton routines, we can add them to this class and create 
	  //getters for them
	  public SequentialCommandGroup get4CubeAuton(){
		return fourcubeAuton;
	  }

	  
	
}
