// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.utilities.*;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class FieldConstants {
    public static final double kFieldLength = 17.548;
    public static final double kFieldWidth = 8.052;
    public static final double kHalfFieldWidth = 4.026;
    public static final double kQuarterFieldWidth = 4.026/2;
    public static final double kHalfFieldLength = 8.774;
    public static final Translation2d kFieldCenter = new Translation2d(kHalfFieldLength, kHalfFieldWidth);
    // cage
    public static final Pose2d kCage = new Pose2d(0,0, Rotation2d.fromDegrees(90));

    // public static final Translation2d kBlueSourceTop = new Translation2d(1, 7);
    // public static final Translation2d kBlueSourceLow = new Translation2d(1, .9);

    // public static final Translation2d kRedSourceTop =
    // GeometryUtil.mirrorTranslationForRedAlliance(kBlueSourceTop);
    // public static final Translation2d kRedSourceLow =
    // GeometryUtil.mirrorTranslationForRedAlliance(kBlueSourceLow);

    // Coral Source Coordinates:
    public static final int[] kRedTagIDs = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
    public static final int[] kBlueTagIDs = {12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22};

    public static final Rotation2d kBlueSourceRotationTop = new Rotation2d(125.2 * (Math.PI / 180));
    public static final Rotation2d kBlueSourceRotationBottom = new Rotation2d(234.8 * (Math.PI / 180));
    public static final Rotation2d kRedSourceRotationTop = new Rotation2d(55 * (Math.PI / 180));
    public static final Rotation2d kRedSourceRotationBottom = new Rotation2d(306 * (Math.PI / 180));

    // blue side top
    public static final Pose2d kBlueSourceTop_A = new Pose2d(new Translation2d(0.714, 6.620), kBlueSourceRotationTop);
    //public static final Pose2d kBlueSourceTop_B = new Pose2d(new Translation2d(0.85, 7.45), kSourceRotationTop);
    public static final Pose2d kBlueSourceTop_C = new Pose2d(new Translation2d(1.702, 7.447), kBlueSourceRotationTop);

    public static final Pose2d[] kBlueSourceTop = { kBlueSourceTop_A, kBlueSourceTop_C };

    // blue side bottom
    public static final Pose2d kBlueSourceBottom_A = new Pose2d(new Translation2d(0.644, 1.34), kBlueSourceRotationTop);
    //public static final Pose2d kBlueSourceBottom_B = new Pose2d(new Translation2d(0.85, 0.6), kSourceRotationBottom);
    public static final Pose2d kBlueSourceBottom_C = new Pose2d(new Translation2d(1.6, 0.65), kBlueSourceRotationTop);

    public static final Pose2d[] kBlueSourceBottom = { kBlueSourceBottom_A, kBlueSourceBottom_C };

    // red side top
    public static final Translation2d kRedSourceTop_A_translation2d = GeometryUtil
        .mirrorTranslationForRedAlliance(kBlueSourceTop_A.getTranslation());
    public static final Pose2d kRedSourceTop_A = new Pose2d(kRedSourceTop_A_translation2d, kRedSourceRotationTop);

    // public static final Translation2d kRedSourceTop_B_translation2d = GeometryUtil
    //     .mirrorTranslationForRedAlliance(kBlueSourceTop_B.getTranslation());
    // public static final Pose2d kRedSourceTop_B = new Pose2d(kRedSourceTop_B_translation2d, kSourceRotationTop);

    public static final Translation2d kRedSourceTop_C_translation2d = GeometryUtil
        .mirrorTranslationForRedAlliance(kBlueSourceTop_C.getTranslation());
    public static final Pose2d kRedSourceTop_C = new Pose2d(kRedSourceTop_C_translation2d, kRedSourceRotationTop);

    public static final Pose2d[] kRedSourceTop = { kRedSourceTop_A, kRedSourceTop_C };

    // red side bottom
    public static final Translation2d kRedSourceBottom_A_translation2d = GeometryUtil
        .mirrorTranslationForRedAlliance(kBlueSourceBottom_A.getTranslation());
    public static final Pose2d kRedSourceBottom_A = new Pose2d(kRedSourceBottom_A_translation2d, kRedSourceRotationBottom);

    // public static final Translation2d kRedSourceBottom_B_translation2d = GeometryUtil
    //     .mirrorTranslationForRedAlliance(kBlueSourceBottom_B.getTranslation());
    // public static final Pose2d kRedSourceBottom_B = new Pose2d(kRedSourceBottom_B_translation2d, kSourceRotationTop);

    public static final Translation2d kRedSourceBottom_C_translation2d = GeometryUtil
        .mirrorTranslationForRedAlliance(kBlueSourceBottom_C.getTranslation());
    public static final Pose2d kRedSourceBottom_C = new Pose2d(kRedSourceBottom_C_translation2d, kRedSourceRotationBottom);

    public static final Pose2d[] kRedSourceBottom = { kRedSourceBottom_A, kRedSourceBottom_C };

    public static final Pose2d kBlueProcessor = new Pose2d(new Translation2d(6, 0.52),
        new Rotation2d(90 * (Math.PI / 180)));
    public static final Pose2d kRedProcessor = new Pose2d(GeometryUtil.mirrorReef(kBlueProcessor.getTranslation()),
        new Rotation2d(270 * (Math.PI / 180)));

    public static final ReefFace[] kBlueReefFaces = { ReefConstants.kBlueReefFace1, ReefConstants.kBlueReefFace2,
        ReefConstants.kBlueReefFace3, ReefConstants.kBlueReefFace4, ReefConstants.kBlueReefFace5,
        ReefConstants.kBlueReefFace6 };

    public static final ReefFace[] kRedReefFaces = { ReefConstants.kRedReefFace1, ReefConstants.kRedReefFace2,
        ReefConstants.kRedReefFace3, ReefConstants.kRedReefFace4, ReefConstants.kRedReefFace5,
        ReefConstants.kRedReefFace6 };

    // the Y value will not be used for the Barge Pose2d coordinates
    public static final Pose2d kBlueBarge = new Pose2d(new Translation2d(7.6, 0), new Rotation2d(180 * (Math.PI / 180)));
    public static final Pose2d kRedBarge = new Pose2d(new Translation2d(10, 0), new Rotation2d(0 * (Math.PI / 180)));
    public static final Translation2d kBlueBargeHalf = new Translation2d(kHalfFieldLength, kQuarterFieldWidth + kHalfFieldWidth);
    public static final Translation2d kRedBargeHalf = GeometryUtil.mirrorReef(kBlueBargeHalf);

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kReefControllerPort = 2;
  }

  public static class DrivetrainConstants {
    public static final double kDeadband = 0.15;
    public static final double[] XY_Axis_inputBreakpoints = { -1, -0.9, -0.85, -0.7, -0.6, -0.5, -0.2, -0.12, 0.12, 0.2,
        0.5, 0.6, 0.7, 0.85, .9, 1 };
    public static final double[] XY_Axis_outputTable = { -1.0, -.7, -0.6, -0.4, -0.3, -0.2, -0.05, 0, 0, 0.05, 0.2, 0.3,
        0.4, 0.6, .7, 1.0 };
    public static final double[] RotAxis_inputBreakpoints = { -1, -.9, -0.85, -0.7, -0.6, -0.5, -0.2, -0.12, 0.12, 0.2,
        0.5, 0.6, 0.7, 0.85, .9, 1 };
    public static final double[] RotAxis_outputTable = { -1.0, -.7, -0.6, -0.4, -0.3, -0.2, -0.05, 0, 0, 0.05, 0.2, 0.3,
        0.4, 0.6, .7, 1.0 };
    public static final double kSpeedAt12VoltsMps = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MaxAngularRate = RotationsPerSecond.of(Math.PI / 2).in(RadiansPerSecond);
    //Front Left
    public static final int kFrontLeftDriveMotorId = 2;
    public static final int kFrontLeftSteerMotorId = 3;
    public static final int kFrontLeftEncoderId = 4;

    // public static final double kFrontLeftEncoderOffset = -0.91259765625;
    // public static final double kFrontLeftEncoderOffset = 179.087402;

    public static final double kFrontLeftXPosInches = 9;
    public static final double kFrontLeftYPosInches = 9;

    // Front Right
    public static final int kFrontRightDriveMotorId = 5;
    public static final int kFrontRightSteerMotorId = 6;
    public static final int kFrontRightEncoderId = 7;
    // public static final double kFrontRightEncoderOffset = -0.5576171875;

    public static final double kFrontRightXPosInches = 9;
    public static final double kFrontRightYPosInches = -9;

    // Back Left
    public static final int kBackLeftDriveMotorId = 8;
    public static final int kBackLeftSteerMotorId = 9;
    public static final int kBackLeftEncoderId = 10;
    // public static final double kBackLeftEncoderOffset = -0.664794921875;

    public static final double kBackLeftXPosInches = -9;
    public static final double kBackLeftYPosInches = 9;

    // Back Right
    public static final int kBackRightDriveMotorId = 11;
    public static final int kBackRightSteerMotorId = 12;
    public static final int kBackRightEncoderId = 13;
//     public static final double kBackRightEncoderOffset = -0.74267578125;
  }

  public static class ReefConstants {
    public static final int kReefLeft = 0;
    public static final int kReefAlgae = 1;
    public static final int kReefRight = 2;


    // #region blueReef
    private static final Translation2d kBlueRight1 = new Translation2d(4.95, 5.29);
    private static final Translation2d kBlueAlgae1 = new Translation2d(5.15, 5.15);
    private static final Translation2d kBlueLeft1 = new Translation2d(5.25, 5.1);
    private static final Rotation2d kBlueRotation1 = new Rotation2d(60 * (Math.PI / 180));

    private static final ReefFace kBlueReefFace1 = new ReefFace(kBlueRight1, kBlueAlgae1, kBlueLeft1, kBlueRotation1);

    private static final Translation2d kBlueRight2 = new Translation2d(5.8, 4.2);
    private static final Translation2d kBlueAlgae2 = new Translation2d(5.8, 4.04);
    private static final Translation2d kBlueLeft2 = new Translation2d(5.8, 3.9);
    private static final Rotation2d kBlueRotation2 = new Rotation2d(0);

    private static final ReefFace kBlueReefFace2 = new ReefFace(kBlueRight2, kBlueAlgae2, kBlueLeft2, kBlueRotation2);

    private static final Translation2d kBlueRight3 = new Translation2d(5.3, 3);
    private static final Translation2d kBlueAlgae3 = new Translation2d(5.15, 2.9);
    private static final Translation2d kBlueLeft3 = new Translation2d(5.025, 2.825);
    private static final Rotation2d kBlueRotation3 = new Rotation2d(300 * (Math.PI / 180));

    private static final ReefFace kBlueReefFace3 = new ReefFace(kBlueRight3, kBlueAlgae3, kBlueLeft3, kBlueRotation3);

    private static final Translation2d kBlueRight4 = new Translation2d(4., 2.8);
    private static final Translation2d kBlueAlgae4 = new Translation2d(3.85, 2.9);
    private static final Translation2d kBlueLeft4 = new Translation2d(3.7, 3);
    private static final Rotation2d kBlueRotation4 = new Rotation2d(240 * (Math.PI / 180));

    private static final ReefFace kBlueReefFace4 = new ReefFace(kBlueRight4, kBlueAlgae4, kBlueLeft4, kBlueRotation4);

    private static final Translation2d kBlueRight5 = new Translation2d(3.2, 3.8);
    private static final Translation2d kBlueAlgae5 = new Translation2d(3.2, 4.04);
    private static final Translation2d kBlueLeft5 = new Translation2d(3.2, 4.175);
    private static final Rotation2d kBlueRotation5 = new Rotation2d(180 * (Math.PI / 180));

    private static final ReefFace kBlueReefFace5 = new ReefFace(kBlueRight5, kBlueAlgae5, kBlueLeft5, kBlueRotation5);

    private static final Translation2d kBlueRight6 = new Translation2d(3.69, 5.05);
    private static final Translation2d kBlueAlgae6 = new Translation2d(3.85, 5.15);
    private static final Translation2d kBlueLeft6 = new Translation2d(3.95, 5.2);
    private static final Rotation2d kBlueRotation6 = new Rotation2d(120 * (Math.PI / 180));

    private static final ReefFace kBlueReefFace6 = new ReefFace(kBlueRight6, kBlueAlgae6, kBlueLeft6, kBlueRotation6);
    // private static final Translation2d kBlueRight1 = new Translation2d(4.95, 5.29);
    // private static final Translation2d kBlueAlgae1 = new Translation2d(5.15, 5.15);
    // private static final Translation2d kBlueLeft1 = new Translation2d(5.25, 5.1);
    // private static final Rotation2d kBlueRotation1 = new Rotation2d(60 * (Math.PI / 180));

    // private static final ReefFace kBlueReefFace1 = new ReefFace(kBlueRight1, kBlueAlgae1, kBlueLeft1, kBlueRotation1);

    // private static final Translation2d kBlueRight2 = new Translation2d(5.8, 4.2);
    // private static final Translation2d kBlueAlgae2 = new Translation2d(5.8, 4.04);
    // private static final Translation2d kBlueLeft2 = new Translation2d(5.8, 3.9);
    // private static final Rotation2d kBlueRotation2 = new Rotation2d(0);

    // private static final ReefFace kBlueReefFace2 = new ReefFace(kBlueRight2, kBlueAlgae2, kBlueLeft2, kBlueRotation2);

    // private static final Translation2d kBlueRight3 = new Translation2d(5.3, 3);
    // private static final Translation2d kBlueAlgae3 = new Translation2d(5.15, 2.9);
    // private static final Translation2d kBlueLeft3 = new Translation2d(5.025, 2.825);
    // private static final Rotation2d kBlueRotation3 = new Rotation2d(300 * (Math.PI / 180));

    // private static final ReefFace kBlueReefFace3 = new ReefFace(kBlueRight3, kBlueAlgae3, kBlueLeft3, kBlueRotation3);

    // private static final Translation2d kBlueRight4 = new Translation2d(4., 2.8);
    // private static final Translation2d kBlueAlgae4 = new Translation2d(3.85, 2.9);
    // private static final Translation2d kBlueLeft4 = new Translation2d(3.7, 3);
    // private static final Rotation2d kBlueRotation4 = new Rotation2d(240 * (Math.PI / 180));

    // private static final ReefFace kBlueReefFace4 = new ReefFace(kBlueRight4, kBlueAlgae4, kBlueLeft4, kBlueRotation4);

    // private static final Translation2d kBlueRight5 = new Translation2d(3.2, 3.8);
    // private static final Translation2d kBlueAlgae5 = new Translation2d(3.2, 4.04);
    // private static final Translation2d kBlueLeft5 = new Translation2d(3.2, 4.175);
    // private static final Rotation2d kBlueRotation5 = new Rotation2d(180 * (Math.PI / 180));

    // private static final ReefFace kBlueReefFace5 = new ReefFace(kBlueRight5, kBlueAlgae5, kBlueLeft5, kBlueRotation5);

    // private static final Translation2d kBlueRight6 = new Translation2d(3.69, 5.05);
    // private static final Translation2d kBlueAlgae6 = new Translation2d(3.85, 5.15);
    // private static final Translation2d kBlueLeft6 = new Translation2d(3.95, 5.2);
    // private static final Rotation2d kBlueRotation6 = new Rotation2d(120 * (Math.PI / 180));

    // private static final ReefFace kBlueReefFace6 = new ReefFace(kBlueRight6, kBlueAlgae6, kBlueLeft6, kBlueRotation6);

    // #endregion
    // #region redReef

    private static final Translation2d kRedRight1 = GeometryUtil.mirrorReef(kBlueRight1);
    private static final Translation2d kRedAlgae1 = GeometryUtil.mirrorReef(kBlueAlgae1);
    private static final Translation2d kRedLeft1 = GeometryUtil.mirrorReef(kBlueLeft1);
    private static final Rotation2d kRedRotation1 = new Rotation2d(240 * (Math.PI / 180));

    private static final ReefFace kRedReefFace1 = new ReefFace(kRedRight1, kRedAlgae1, kRedLeft1, kRedRotation1);

    private static final Translation2d kRedRight2 = GeometryUtil.mirrorReef(kBlueRight2);
    private static final Translation2d kRedAlgae2 = GeometryUtil.mirrorReef(kBlueAlgae2);
    private static final Translation2d kRedLeft2 = GeometryUtil.mirrorReef(kBlueLeft2);
    private static final Rotation2d kRedRotation2 = new Rotation2d(180 * (Math.PI / 180));

    private static final ReefFace kRedReefFace2 = new ReefFace(kRedRight2, kRedAlgae2, kRedLeft2, kRedRotation2);

    private static final Translation2d kRedRight3 = GeometryUtil.mirrorReef(kBlueRight3);
    private static final Translation2d kRedAlgae3 = GeometryUtil.mirrorReef(kBlueAlgae3);
    private static final Translation2d kRedLeft3 = GeometryUtil.mirrorReef(kBlueLeft3);
    private static final Rotation2d kRedRotation3 = new Rotation2d(120 * (Math.PI / 180));

    private static final ReefFace kRedReefFace3 = new ReefFace(kRedRight3, kRedAlgae3, kRedLeft3, kRedRotation3);

    private static final Translation2d kRedRight4 = GeometryUtil.mirrorReef(kBlueRight4);
    private static final Translation2d kRedAlgae4 = GeometryUtil.mirrorReef(kBlueAlgae4);
    private static final Translation2d kRedLeft4 = GeometryUtil.mirrorReef(kBlueLeft4);
    private static final Rotation2d kRedRotation4 = new Rotation2d(60 * (Math.PI / 180));

    private static final ReefFace kRedReefFace4 = new ReefFace(kRedRight4, kRedAlgae4, kRedLeft4, kRedRotation4);

    private static final Translation2d kRedRight5 = GeometryUtil.mirrorReef(kBlueRight5);
    private static final Translation2d kRedAlgae5 = GeometryUtil.mirrorReef(kBlueAlgae5);
    private static final Translation2d kRedLeft5 = GeometryUtil.mirrorReef(kBlueLeft5);
    private static final Rotation2d kRedRotation5 = new Rotation2d(0);

    private static final ReefFace kRedReefFace5 = new ReefFace(kRedRight5, kRedAlgae5, kRedLeft5, kRedRotation5);

    private static final Translation2d kRedRight6 = GeometryUtil.mirrorReef(kBlueRight6);
    private static final Translation2d kRedAlgae6 = GeometryUtil.mirrorReef(kBlueAlgae6);
    private static final Translation2d kRedLeft6 = GeometryUtil.mirrorReef(kBlueLeft6);
    private static final Rotation2d kRedRotation6 = new Rotation2d(300 * (Math.PI / 180));

    private static final ReefFace kRedReefFace6 = new ReefFace(kRedRight6, kRedAlgae6, kRedLeft6, kRedRotation6);
  }

    public static class ClawConstants {
      // Claw constants
      public static final int kClawPivotinatorID = 19;
      public static final int kClawTopRollinatorID = 20;
      public static final int kClawBottomRollinatorID = 21;

      public static final int kClawAlgaeDetectinatorID = 22;
      // TODO: change the detectinator channel
      public static final int kClawAlgaeDetectinatorChannel = 2;
      // TODO: change debounce time
      public static final double kClawAlgaeDebounceTime = .5;

    }

    public static class AscendinatorConstants {
      public static final int kPrimaryinatorAscendinatorID = 16;
      public static final int kSecondaryinatorAscendinatorID = 17;
      public static final int kAscendinatorDetectinatorID = 18;
      public static final int kDebouncinatorTime = 0;
      public static final int kAscendinatorDetectinatorChannel = 3;
      public static final double kIdle = 0;
      // public static final double kPrepClimb = .47;
      public static final double kPrepClimb = .57;
      public static final double kEndClimb = .79;
    }
    public static class CalsificationinatorConstants {
      public static final int kCalsificationatorCANcoderID = 30;
        public static final int kSuckinatorScoreinatorID = 23;
        public static final int kPivotinatorID = 24;
        public static final int kPostioninatorID = 25;
        public static final int kFirstDetectinatorID = 26;
        public static final int kSecondDetectinatorID = 27; /*First is the one closer to the rooler*/
        public static final int kPivotinatorCancoderID = 30;
        public static final int kDebouncinatorCoralTime = 0;
        public static final int kCalsificationDetectinatorChanel = 0;
        public static final int kCalsificationDetectinatorTwoChanel = 1;
        public static final double kIdlePosition = 0;
        public static final double kPickUpPosition = .876;
        public static final double kL4Position = -.1;
        public static final double kL3Position = -.11;
        public static final double kL2Position = -.11;
        public static final double kL1Position = -.11;
        public static final double kPrepClimb = .876;
    }

    public static class ElevatinatorConstants {
      public static final int kLifinatorMotor = 15;
      public static final double kAlgaeNet = 71.46;
      public static final double kL4Coral = 71.46;
      public static final double kL3Coral = 37;
      public static final double kL3Algae = 55;
      public static final double kL2Coral = 16.17;
      public static final double kL2Algae = 34;
      public static final double kL1Coral = 0;
      public static final double kHumanPlayer = 3;
      public static final double kAlgaePickup = 15;
      public static final double kAlgaeProcessor = 8;
      public static final double kAlgaeHold = 18;
      public static final double kHome = 0;
      public static final double kLiftinatorMaxHeight = 74;
    }

    public static class LEDinatorConstants
    {
      public static final int kLEDinatorID = 28;
      public static final int kLEDPartyLightinatorID = 29;
    }
}
