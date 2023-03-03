package frc.robot.subsystems.Arm;

import java.util.Set;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;

/** All of the parameters required to generate a trajectory. */
public class TrajectoryParameters {

	private Vector<N2> initialJointPositions;
	private Vector<N2> finalJointPositions;
	private Set<String> constraintOverrides;

	/** Creates a new Parameters object with no constraint overrides. */
	public TrajectoryParameters(Vector<N2> initialJointPositions, Vector<N2> finalJointPositions) {
		this(initialJointPositions, finalJointPositions, Set.of());
	}

	/** Creates a new Parameters object with constraint overrides. */
	public TrajectoryParameters(Vector<N2> initialJointPositions, Vector<N2> finalJointPositions, Set<String> constraintOverrides) {
		this.initialJointPositions = initialJointPositions;
		this.finalJointPositions = finalJointPositions;
		this.constraintOverrides = constraintOverrides;
	}

	public Vector<N2> initialJointPositions() {
		return this.initialJointPositions;
	}

	public Vector<N2> finalJointPositions() {
		return this.finalJointPositions;
	}

	public Set<String> constraintOverrides() {
		return constraintOverrides;
	}


}