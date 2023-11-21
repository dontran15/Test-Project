// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 
The Constants class provides a convenient place for teams to hold robot-wide
numerical or boolean
constants. This class should not be used for any other purpose. All constants
should be declared
globally (i.e. public static). Do not put anything functional in this class.
*
<p>
It is advised to statically import this class (or one of its inner classes)
wherever the
constants are needed, to reduce verbosity.
*/

public final class Constants {

  public static class Robot {
    public static final boolean isSim = true;
  }

  public static class Swerve {
    public static final double maxDriveSpeedMPS = 5;
    public static final double maxDriveAccelerationMPSS = 5;
    public static final double maxRotationSpeedRadPS = 2 * Math.PI;
    public static final double maxRotationAccelerationRadPSS = 2 * Math.PI;

    public static final double controllerDeadband = 0.1;
    public static final double horizontalBaseM = Units.inchesToMeters(17.5);
    public static final double verticaleBaseM = Units.inchesToMeters(17.5);
    public static final double wheelDiameterM = Units.inchesToMeters(3.5); 

    public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
        new Translation2d(horizontalBaseM / 2, verticaleBaseM / 2), // front left
        new Translation2d(horizontalBaseM / 2, -verticaleBaseM / 2), // front right
        new Translation2d(-horizontalBaseM / 2, verticaleBaseM / 2), // back left
        new Translation2d(-horizontalBaseM / 2, -verticaleBaseM / 2)); // back right

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}