// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Represents a target position for the arm. */
public class ArmPose {

    private Translation2d endEffectorPosition;

    public ArmPose(Translation2d endEffectorPosition) {
        this.endEffectorPosition = endEffectorPosition;
    }

    public Translation2d endEffectorPosition() {
        return this.endEffectorPosition;
    }

    public static enum Preset {
        STOW(new ArmPose(new Translation2d(0, 0)));

        private ArmPose pose;

        private Preset(ArmPose pose) {
            this.pose = pose;
        }

        public ArmPose getPose() {
            return pose;
        }

        public static void updateStowPreset(ArmConfig config) {
            STOW.pose = new ArmPose(
                    new Translation2d(
                            config.origin().getX(),
                            config.origin().getY() + config.shoulder().length() - config.elbow().length()));
        }
    }

}
