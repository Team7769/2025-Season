package frc.robot.utilities;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ReefFace {
    private Pose2d right;
    private Pose2d left;
    private Pose2d algae;
    private Rotation2d rotation;

    public ReefFace(Translation2d rightLocation, Translation2d algaeLocation, Translation2d leftLocation, Rotation2d setRotation) {
        right = new Pose2d(rightLocation, setRotation);
        left = new Pose2d(leftLocation, setRotation);
        algae = new Pose2d(algaeLocation, setRotation);
        rotation = setRotation;
    }

    public Pose2d[] getBothPoles() {
        return new Pose2d[]{left, right};
    }

    public Pose2d[] getAllPoles() {
        return new Pose2d[]{left, algae, right};
    }

    public Pose2d getRight() {
        return right;
    }

    public Pose2d getAlgae() {
        return algae;
    }

    public Pose2d getLeft() {
        return left;
    }
}
