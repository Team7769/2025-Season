package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.utilities.VisionMeasurement;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    private Pose2d _limelightThreePose = new Pose2d();
    private Pose2d _limelightFourPose = new Pose2d();

    private PoseEstimate _limelightFourPoseEstimate = new PoseEstimate();
    private AprilTagFieldLayout _fieldLayout;

    private double coralPoseOffsetX = 0.5;
    private double coralPoseOffsetY = 0.5;
    
    private double algaePoseOffsetX = 0.5;
    private double algaePoseOffsetY = 0.5;

    private static final double filterDistanceError = 2;
    private static final double filterAngleError = 5; 
    private LinearFilter limelightDistanceFilter = LinearFilter.singlePoleIIR(1/(2* Math.PI * filterDistanceError), 0.02);
	private LinearFilter limelightAngleFilter = LinearFilter.singlePoleIIR(1/(2*Math.PI * filterAngleError), 0.02);

    public Vision()
    {
        _fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    }

    public void updateLimelightPosition(Rotation2d rotation)
    {
            LimelightHelpers.SetRobotOrientation(
                "limelight-three",
                rotation.getDegrees(),
                0,
                0,
                0,
                0,
                0
            );

            
    }

    @Override
    public void periodic() {
        _limelightThreePose = LimelightHelpers.getBotPose2d("limelight-three");
        _limelightFourPose = LimelightHelpers.getBotPose2d("limelight-four");
    }

    public Pose2d getLimelightThreePose() {
        return _limelightThreePose;
    }

    public Pose2d getLimelightFourPose() {
        return _limelightFourPose;
    }

    public double getDistance()
    {
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        var ty = table.getEntry("ty").getDouble(0) * 1.18; // constant
        var tx = table.getEntry("tx").getDouble(0);
        var tv = table.getEntry("tv").getDouble(0);
        
        if (tv != 0.0) {
            double distance = .905 / Math.tan(Math.toRadians(ty)); // constant
            double filterDistance = limelightDistanceFilter.calculate(distance);
            SmartDashboard.putNumber("VisionSystemGetDistance", distance);
            return filterDistance; 
        }
        else {
            return 0.0;
        }
    }

    public ArrayList<VisionMeasurement> getVisionMeasurements(
        Rotation2d rotation
    )
    {
        ArrayList<VisionMeasurement> visionMeasurements = new ArrayList<>();

            LimelightHelpers.SetRobotOrientation(
                "limelight-three",
                rotation.getDegrees(),
                0, 
                0, 
                0, 
                0,
                0
            );

            PoseEstimate limelightPoseEstimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
                    "limelight-three"
                );

            if (limelightPoseEstimate != null && limelightPoseEstimate.tagCount > 0) {
                visionMeasurements.add(
                    new VisionMeasurement(
                        limelightPoseEstimate.pose,
                        limelightPoseEstimate.timestampSeconds
                    )
                );
            }
            LimelightHelpers.SetRobotOrientation(
                "limelight-four",
                rotation.getDegrees(),
                0, 
                0, 
                0, 
                0,
                0
            );

            PoseEstimate limelightFourPoseEstimate =
                LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(
                    "limelight-four"
                );

            if (limelightFourPoseEstimate != null && limelightFourPoseEstimate.tagCount > 0) {
                _limelightFourPoseEstimate = limelightFourPoseEstimate;
                visionMeasurements.add(
                    new VisionMeasurement(
                        limelightFourPoseEstimate.pose,
                        limelightFourPoseEstimate.timestampSeconds
                    )
                );
            }
        return visionMeasurements;
    }

    public Pose2d getFrontLimelightPose(){
        return _limelightFourPoseEstimate.pose;
    }

    public Pose2d getRobotPoseInTargetSpace() {
        var botPoseTargetSpace = LimelightHelpers.getBotPose_TargetSpace("limelight-four");

        return new Pose2d(botPoseTargetSpace[0], botPoseTargetSpace[1], new Rotation2d(botPoseTargetSpace[4]));
    }
}


    

