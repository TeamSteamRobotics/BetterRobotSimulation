// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class Ports {
        public static final int frontLeftID = 0;
        public static final int backLeftID = 1;
        public static final int frontRightID = 2;
        public static final int backRightID = 3;
    }
    public static final class PhysicalRobotConstants {
       public static final double kS = .579; //Volts
       public static final double kV = 2.37; //VoltsSecondsPerMeter
       public static final double kA = 0.21; //VoltsSecondsSquaredPerMeter
       public static final double kMaxVoltage = 12; //Volts
       public static final double kFalconCPR = 2048; //2048 Cycles Per Rotation
       public static final double kSensorGearRatio = 1; //Gear ratio is the ratio between the *encoder* and the wheels. Mounted 1:1 on AndyMark Drivetrain
       public static final double kGearRatio = 10.71; //Gear ratio when the encoder is on the motor instead of the gearbox //4.71
       public static final double kWheelDiameterMeters =  0.1524;
       public static final double kTrackWidthMeters = 0.552976;
       public static final double kWheelRadiusInches = 3;
    }

    public static final class DriveConstants {
        public static final double kP = 0.1;
        public static final double kI = 0.2;
        public static final double kD = 0.3;
        public static final double kMaxVelocity = 3; //MetersPerSecond
        public static final double kMaxAcceleration = 3; //MetersPerSecondSquared
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final DifferentialDriveKinematics kDriveKinematics =
        new DifferentialDriveKinematics(PhysicalRobotConstants.kTrackWidthMeters); 
            //set of physics equations to make sure velocity and acceleration caps are obeyed
    }

}
