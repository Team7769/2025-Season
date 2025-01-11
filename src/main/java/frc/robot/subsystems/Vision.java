package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.utilities.VisionMeasurement;


import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase{
    private Pose2d _limelightPose = new Pose2d();

    private static final double filterDistanceError = 2;
    private static final double filterAngleError = 5; 
    private LinearFilter limelightDistanceFilter = LinearFilter.singlePoleIIR(1/(2* Math.PI * filterDistanceError), 0.02);
	private LinearFilter limelightAngleFilter = LinearFilter.singlePoleIIR(1/(2*Math.PI * filterAngleError), 0.02);

    Vision()
    {

    }

    public void updateLimelightPosition(Rotation2d rotation)
    {
            LimelightHelpers.SetRobotOrientation(
                "limelight",
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
        _limelightPose = LimelightHelpers.getBotPose2d("limelight");
    }

    public Pose2d getLimelightPose() {
        return _limelightPose;
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


}


    

