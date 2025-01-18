// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.utilities.*;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Translation2d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class FieldConstants {
    public static final double kFieldLength = 17.548;
    public static final double kFieldWidth = 8.052;
    public static final double kHalfFieldWidth = 4.026;
    public static final double kHalfFieldLength = 8.774;

    public static final Translation2d kBlueSourceTop = new Translation2d(1, 7);
    public static final Translation2d kBlueSourceLow = new Translation2d(1, .9);

    public static final Translation2d kRedSourceTop = GeometryUtil.mirrorTranslationForRedAlliance(kBlueSourceTop);
    public static final Translation2d kRedSourceLow = GeometryUtil.mirrorTranslationForRedAlliance(kBlueSourceLow);

    public static final Translation2d kBlueAlgea = new Translation2d(6.4, .6);

    public static final Translation2d kRedAlgea = new Translation2d(11.5, 7.5);

    public static final Translation2d[] kBlueCoralArray = {new Translation2d(4.778, 4.859), new Translation2d(5.079, 4.694), new Translation2d(5.409, 4.213),
       new Translation2d(5.379, 3.852), new Translation2d(5.109, 3.296), new Translation2d(4.823, 3.131), new Translation2d(4.207, 3.131), new Translation2d(3.907, 3.281),
       new Translation2d(3.576, 3.837), new Translation2d(3.576, 4.198), new Translation2d(3.907, 4.769), new Translation2d(4.252, 4.889)};

    public static final Translation2d[] kRedCoralArray = {GeometryUtil.mirrorTranslationForRedAlliance(new Translation2d(4.778, 4.859)), GeometryUtil.mirrorTranslationForRedAlliance(new Translation2d(5.079, 4.694)),
      GeometryUtil.mirrorTranslationForRedAlliance(new Translation2d(5.409, 4.213)), GeometryUtil.mirrorTranslationForRedAlliance(new Translation2d(5.379, 3.852)), GeometryUtil.mirrorTranslationForRedAlliance(new Translation2d(5.109, 3.296)),
      GeometryUtil.mirrorTranslationForRedAlliance(new Translation2d(4.823, 3.131)), GeometryUtil.mirrorTranslationForRedAlliance(new Translation2d(4.207, 3.131)), GeometryUtil.mirrorTranslationForRedAlliance(new Translation2d(3.907, 3.281)),
      GeometryUtil.mirrorTranslationForRedAlliance(new Translation2d(3.576, 3.837)), GeometryUtil.mirrorTranslationForRedAlliance(new Translation2d(3.576, 4.198)), GeometryUtil.mirrorTranslationForRedAlliance(new Translation2d(3.907, 4.769)), GeometryUtil.mirrorTranslationForRedAlliance(new Translation2d(4.252, 4.889))};
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
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
    public static final double MaxAngularRate = RotationsPerSecond.of(Math.PI/2).in(RadiansPerSecond);
    public static final int kFrontLeftDriveMotorId = 2;
    public static final int kFrontLeftSteerMotorId = 3;
    public static final int kFrontLeftEncoderId = 4;
//     public static final double kFrontLeftEncoderOffset = -0.91259765625;
//     public static final double kFrontLeftEncoderOffset = 179.087402;

    public static final double kFrontLeftXPosInches = 9;
    public static final double kFrontLeftYPosInches = 9;

    // Front Right
    public static final int kFrontRightDriveMotorId = 5;
    public static final int kFrontRightSteerMotorId = 6;
    public static final int kFrontRightEncoderId = 7;
//     public static final double kFrontRightEncoderOffset = -0.5576171875;

    public static final double kFrontRightXPosInches = 9;
    public static final double kFrontRightYPosInches = -9;

    // Back Left
    public static final int kBackLeftDriveMotorId = 8;
    public static final int kBackLeftSteerMotorId = 9;
    public static final int kBackLeftEncoderId = 10;
//     public static final double kBackLeftEncoderOffset = -0.664794921875;

    public static final double kBackLeftXPosInches = -9;
    public static final double kBackLeftYPosInches = 9;

    // Back Right
    public static final int kBackRightDriveMotorId = 11;
    public static final int kBackRightSteerMotorId = 12;
    public static final int kBackRightEncoderId = 13;
//     public static final double kBackRightEncoderOffset = -0.74267578125;
  }
}
