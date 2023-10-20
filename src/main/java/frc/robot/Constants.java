// Copyright (c) 2023 FRC team 3374

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.util.Units;

public final class Constants {
    public static class ModuleConstants {
        public static double maxModuleAngularSpeed = 2 * Math.PI;
        public static double maxModuleAngularAcceleration = 2 * Math.PI;

        public static final double maxSpeed = Units.feetToMeters(13.5);

        //CHECK VALUES IN RL
        public static double wheelDiameter = Units.inchesToMeters(3);
        public static double wheelCircumference = wheelDiameter * Math.PI;
        public static double driveGearRatio = 8.14; // 8.16 for mk3
        public static double angleGearRatio = ((150.0 / 7.0) / 1.0); // published angle gear reduction on Mk4i
        // public static final double angleGearRatio = (12.8 / 1.0); // published angle gear reduction on Mk3

        //Swerve Current Limiting
        public static int angleContinuousCurrentLimit = 25;
        public static int anglePeakCurrentLimit = 40;
        public static double anglePeakCurrentDuration = 0.1;
        public static boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60; //CHECK WITH RL
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        //Neutral Modes
        public static NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static NeutralMode driveNeutralMode = NeutralMode.Brake;

        //Loop Ramps
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
        
        // public static int encoderCPR = 4096;
        // public static double wheelDiameter = Units.inchesToMeters(3);
        // public static double driveEncoderDistancePerPulse = (wheelDiameter * Math.PI) / (double) encoderCPR;

        // public static double turningEncoderDistancePerPulse = (2 * Math.PI) / (double) encoderCPR;

    }
    public static class Conversions {
        /**
         * @param positionCounts CANCoder Position Counts
         * @param gearRatio Gear Ratio between CANCoder and Mechanism
         * @return Degrees of Rotation of Mechanism
         */
        public static double CANcoderToDegrees(double positionCounts, double gearRatio) {
            return positionCounts * (360.0 / (gearRatio * 4096.0));
        }

        /**
         * @param degrees Degrees of rotation of Mechanism
         * @param gearRatio Gear Ratio between CANCoder and Mechanism
         * @return CANCoder Position Counts
         */
        public static double degreesToCANcoder(double degrees, double gearRatio) {
            return degrees / (360.0 / (gearRatio * 4096.0));
        }

        /**
         * @param counts Falcon Position Counts
         * @param gearRatio Gear Ratio between Falcon and Mechanism
         * @return Degrees of Rotation of Mechanism
         */
        public static double falconToDegrees(double positionCounts, double gearRatio) {
            return positionCounts * (360.0 / (gearRatio * 2048.0));
        }

        /**
         * @param degrees Degrees of rotation of Mechanism
         * @param gearRatio Gear Ratio between Falcon and Mechanism
         * @return Falcon Position Counts
         */
        public static double degreesToFalcon(double degrees, double gearRatio) {
            return degrees / (360.0 / (gearRatio * 2048.0));
        }

        /**
         * @param velocityCounts Falcon Velocity Counts
         * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
         * @return RPM of Mechanism
         */
        public static double falconToRPM(double velocityCounts, double gearRatio) {
            double motorRPM = velocityCounts * (600.0 / 2048.0);        
            double mechRPM = motorRPM / gearRatio;
            return mechRPM;
        }

        /**
         * @param RPM RPM of mechanism
         * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
         * @return RPM of Mechanism
         */
        public static double RPMToFalcon(double RPM, double gearRatio) {
            double motorRPM = RPM * gearRatio;
            double sensorCounts = motorRPM * (2048.0 / 600.0);
            return sensorCounts;
        }

        /**
         * @param velocitycounts Falcon Velocity Counts
         * @param circumference Circumference of Wheel
         * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
         * @return Falcon Velocity Counts
         */
        public static double falconToMPS(double velocitycounts, double circumference, double gearRatio){
            double wheelRPM = falconToRPM(velocitycounts, gearRatio);
            double wheelMPS = (wheelRPM * circumference) / 60;
            return wheelMPS;
        }

        /**
         * @param velocity Velocity MPS
         * @param circumference Circumference of Wheel
         * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon MPS)
         * @return Falcon Velocity Counts
         */
        public static double MPSToFalcon(double velocity, double circumference, double gearRatio){
            double wheelRPM = ((velocity * 60) / circumference);
            double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
            return wheelVelocity;
        }

        /**
         * @param positionCounts Falcon Position Counts
         * @param circumference Circumference of Wheel
         * @param gearRatio Gear Ratio between Falcon and Wheel
         * @return Meters
         */
        public static double falconToMeters(double positionCounts, double circumference, double gearRatio){
            return positionCounts * (circumference / (gearRatio * 2048.0));
        }

        /**
         * @param meters Meters
         * @param circumference Circumference of Wheel
         * @param gearRatio Gear Ratio between Falcon and Wheel
         * @return Falcon Position Counts
         */
        public static double MetersToFalcon(double meters, double circumference, double gearRatio){
            return meters / (circumference / (gearRatio * 2048.0));
        }
    }
    
    public static boolean motorTuningMode = true;
}
