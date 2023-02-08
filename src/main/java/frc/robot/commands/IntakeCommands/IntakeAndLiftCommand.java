package frc.robot.commands.IntakeCommands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAndLiftCommand extends CommandBase {
	private IntakeSubsystem intake;
	private BooleanSupplier liftArm, gripIntake, ejectIntake;
	private DoubleSupplier runIntake;
	private boolean latch;

	/** Creates a new IntakeAndLiftCommand. */
	public IntakeAndLiftCommand(IntakeSubsystem i, BooleanSupplier liftArm,
			DoubleSupplier runIntake, BooleanSupplier gripIntake, BooleanSupplier ejectIntake) {
		this.intake = i;
		this.liftArm = liftArm;
		this.gripIntake = gripIntake;
		this.runIntake = runIntake;
		this.ejectIntake = ejectIntake;

		addRequirements(intake);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		latch = false;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {

		// if(liftArm.getAsBoolean()) {
		// intake.setIntakeProximalPosition(Math.PI/2);
		// } else {
		// intake.setIntakeProximalPosition(0);
		// }
		intake.setIntakeDistalPosition(0);
		intake.setIntakeProximalPosition(0);
		SmartDashboard.putNumber("current", intake.getCurrentDraw());
		SmartDashboard.putBoolean("latch", latch);

		if(intake.getCurrentDraw() > 75){
			latch = true;
		}
		if (Math.abs(runIntake.getAsDouble()) > 0.2 && !latch) {
			intake.setRawWheelSpeed(.5);
		} else if (ejectIntake.getAsBoolean()) {
			latch = false;
			intake.setRawWheelSpeed(-0.3);
		} else {
			intake.setRawWheelSpeed(.1);
		}

	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
