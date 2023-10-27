// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static enum Mode {
        REAL, SIM, REPLAY
    }

    public static enum Field {
        BASEMENT (2.0),
        PRACTICE (4.0);

        private final double maxFieldSpeedMetersPerSec;

        private Field(double maxSpeed) {
            this.maxFieldSpeedMetersPerSec = maxSpeed;
        }
    }

    public static final Mode mode = Mode.REAL;
    public static final Field field = Field.BASEMENT;

    public static final boolean tuningMode = true;    

    public static final double dtSeconds = 0.02;
    public static final double loopFrequencyHz = 1.0/dtSeconds;
    
    public static final class CANDevices {

        public static final String driveCanBusName = "rio";

        // Front Left: Green
        public static final int frontLeftDriveMotorID  = 21;
        public static final int frontLeftTurnMotorID   = 22;
        public static final int frontLeftTurnEncoderID = 23;

        // Front Right: Blue
        public static final int frontRightDriveMotorID  = 31;
        public static final int frontRightTurnMotorID   = 32;
        public static final int frontRightTurnEncoderID = 33;

        // Back Left: Red
        public static final int backLeftDriveMotorID  = 11;
        public static final int backLeftTurnMotorID   = 12;
        public static final int backLeftTurnEncoderID = 13;

        // Back Right: Yellow
        public static final int backRightDriveMotorID  = 41;
        public static final int backRightTurnMotorID   = 42;
        public static final int backRightTurnEncoderID = 43;

        public static final int pigeonCanID = 0;
        public static final int candleCanID = 0;

        public static final double minCanUpdateRate = 4.0;        
    }

    public static final class DIOPorts {

        public static final int brakeSwitchPort = 1;
        public static final int ledSwitchPort = 2;
    }
    
    public static final class DriveConstants {

        public static int numDriveModules = DriveModulePosition.values().length;
        
        public static enum DriveModulePosition {
            FRONT_LEFT  (CANDevices.frontLeftDriveMotorID, CANDevices.frontLeftTurnMotorID, CANDevices.frontLeftTurnEncoderID, InvertedValue.CounterClockwise_Positive,
            0.139160),
            FRONT_RIGHT (CANDevices.frontRightDriveMotorID, CANDevices.frontRightTurnMotorID, CANDevices.frontRightTurnEncoderID, InvertedValue.Clockwise_Positive,
            0.315430),
            BACK_LEFT   (CANDevices.backLeftDriveMotorID, CANDevices.backLeftTurnMotorID, CANDevices.backLeftTurnEncoderID, InvertedValue.CounterClockwise_Positive,
            0.178467),
            BACK_RIGHT  (CANDevices.backRightDriveMotorID, CANDevices.backRightTurnMotorID, CANDevices.backRightTurnEncoderID, InvertedValue.Clockwise_Positive,
            0.177490);

            public final int driveMotorID;
            public final int turnMotorID;
            public final int turnEncoderID;
            // motor direction to drive 'forward' (cancoders at angles given in cancoderOffsetRotations)
            public final InvertedValue driveInverted;
            // absolute position of cancoder when drive wheel is facing 'forward'
            public final double cancoderOffsetRotations;

            DriveModulePosition(int driveMotorID, int turnMotorID, int turnEncoderID, InvertedValue driveInverted, double cancoderOffsetRotations) {
                this.driveMotorID = driveMotorID;
                this.turnMotorID = turnMotorID;
                this.turnEncoderID = turnEncoderID;
                this.driveInverted = driveInverted;
                this.cancoderOffsetRotations = cancoderOffsetRotations;
            }
        }

        // weight with battery and bumpers
        public static final double weightKg = Units.lbsToKilograms(58.0);

        public static final double trackWidthXMeters = Units.inchesToMeters(12.00); // distance between the front and back wheels
        public static final double trackWidthYMeters = Units.inchesToMeters(12.00); // distance between the left and right wheels
        public static final double wheelRadiusMeters = Units.inchesToMeters(2.056);

        public static final double driveWheelGearReduction = 1.0 / ((15.0/60.0)*(28.0/16.0)*(14.0/50.0));
        public static final double turnWheelGearReduction = 1.0 / ((15.0/32.0)*(10.0/60.0));

        
        public static final double[] driveRealKps = {0.7, 0.4, 0.7, 0.7};
        public static final double[] driveRealKds = {3.5, 2.5, 3.7, 3.5};

        public static final double driveSnapKp = 1.5;
        public static final double driveSnapKi = 0;
        public static final double driveSnapKd = 0;

        public static final double maxDriveSpeedMetersPerSec = field.maxFieldSpeedMetersPerSec;
        // tangential speed (m/s) = radial speed (rad/s) * radius (m)  
        public static final double maxTurnRateRadiansPerSec = maxDriveSpeedMetersPerSec / Math.hypot(trackWidthXMeters/2, trackWidthYMeters/2);

        public static final double joystickSlewRateLimit = 1.0 / 0.25;     // full speed in 0.25 sec
        public static final double driveJoystickDeadbandPercent = 0.12;
        public static final double driveMaxJerk = 200.0;

        public static final double precisionLinearMultiplier = 0.2;
        public static final double precisionTurnMulitiplier = 0.2;
      
        public static final double poseMoveTranslationkP = 1;
        public static final double poseMoveTranslationMaxVel = 3;
        public static final double poseMoveTranslationMaxAccel = 3;

        public static final double poseMoveRotationkP = 0.05;
        public static final double poseMoveRotationMaxVel = Math.PI;
        public static final double poseMoveRotationMaxAccel = Math.PI;

    }

   

    public static final class VisionConstants {

        public static final String[] cameraNames = {
            "FrontLeft",    // OV2311
            "FrontRight",   // AR0144
            "BackLeft",     // OV9281
            "BackRight",    // OV9281
            "limelight"
        };

        // TODO: update Limelight webUI with camera position, AprilTags field locations

        private static final double photonCamX = Units.inchesToMeters(10.375/2);
        private static final double photonCamY = Units.inchesToMeters(10.00/2);
        private static final double photonCamZ = Units.inchesToMeters(8.5);
        private static final double photonCamPitch = -Units.degreesToRadians(15.0);

        private static final double limelightCamX = Units.inchesToMeters(18.25/2 - 3.25);
        private static final double limelightCamY = Units.inchesToMeters(0);
        private static final double limelightCamZ = Units.inchesToMeters(7.875);
        private static final double limelightCamPitch = Units.degreesToRadians(5.0);

        public static final Transform3d[] robotToCameras = {
            new Transform3d(new Translation3d(-photonCamX, +photonCamY, photonCamZ), new Rotation3d(Units.degreesToRadians(+0.90), photonCamPitch, Units.degreesToRadians(-0.25))),
            new Transform3d(new Translation3d(-photonCamX, -photonCamY, photonCamZ), new Rotation3d(0, photonCamPitch, Units.degreesToRadians(-0.85))),
            new Transform3d(new Translation3d(+photonCamX, +photonCamY, photonCamZ), new Rotation3d(0, photonCamPitch, Units.degreesToRadians(180.0 - 1.48))),
            new Transform3d(new Translation3d(+photonCamX, -photonCamY, photonCamZ), new Rotation3d(0, photonCamPitch, Units.degreesToRadians(180.0 - 0.40))),
            new Transform3d(new Translation3d(limelightCamX, limelightCamY, limelightCamZ), new Rotation3d(0, limelightCamPitch, Units.degreesToRadians(-0.37)))
        };

        // linear regression (in distanceMeters^2) of empirically measured data 
        public static final double k1XYStdDev = 0.03;
        public static final double k0XYStdDev = 0;
        public static final double k1HeadingStdDev = 0;
        //public static final double k0HeadingStdDev = 0.003;   // measured std dev 
        public static final double k0HeadingStdDev = 1000;  // we will trust the gyro, so just make this a large number
 
        public static final int demoTagId = 5;
    }

    public static final class AutoConstants {
        
        public static final double maxVel = 3;
        public static final double maxAccel = 3;

        public static final double maxVelFast = 4;
        public static final double maxAccelFast = 4.5;

        public static final double maxVelSlow = 0.75;
        public static final double maxAccelSlow = 1.5;

        public static final double autoTranslationXKp = 11;
        public static final double autoTranslationXKi = 0;
        public static final double autoTranslationXKd = 0;

        public static final double autoTranslationYKp = 8;
        public static final double autoTranslationYKi = 0;
        public static final double autoTranslationYKd = 0;

        public static final double autoTranslationSlowXKp = 8;
        public static final double autoTranslationSlowXKi = 0;
        public static final double autoTranslationSlowXKd = 0;

        public static final double autoTranslationSlowYKp = 6;
        public static final double autoTranslationSlowYKi = 0;
        public static final double autoTranslationSlowYKd = 0;

        public static final double autoRotationKp = 8;
        public static final double autoRotationKi = 0;
        public static final double autoRotationKd = 0;

        public static final double autoBalanceKp = 0.4;
        public static final double autoBalanceKi = 0.05;
        public static final double autoBalanceKd = 0.0;

        public static final double initialBalanceSpeed = 1;

    }


   
    // Not the robot main function. This is called by Gradle when deploying to
    // make sure nobody deploys sim code. 
    // Stolen from 6328 ;-)
    // See build.gradle

    /**
     * Checks that code is set to right mode when deploying
     */
    public static void main(String... args) {
        if (mode != Mode.REAL) {
            System.err.println("Cannot deploy. Invalid mode: " + mode);
            System.exit(1);
        }
    }
}
