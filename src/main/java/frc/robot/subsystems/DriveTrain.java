package frc.robot.subsystems;

import java.util.Arrays;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveConstants;

public class DriveTrain extends SubsystemBase{
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] swerveMods;
    public Pigeon2 gyro;

    public DriveTrain() {
        gyro = new Pigeon2(SwerveConstants.pigeonID);
        gyro.configFactoryDefault();
        zeroGyro();

        //Create Swerve Modules
        swerveMods = new SwerveModule[] {
            new SwerveModule(0, "frontLeft", 30, 31, 32, 0),
            new SwerveModule(1, "frontRight", 40, 41, 42, 0),
            new SwerveModule(2, "backLeft", 50, 51, 52, 0),
            new SwerveModule(3, "backRight", 60, 61, 62, 0)
        };
        
        Timer.delay(1.0);
        resetModulesToAbsolute();

        swerveOdometry = new SwerveDriveOdometry(SwerveConstants.swerveKinematics, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            SwerveConstants.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), 
                    translation.getY(),
                    rotation,
                    getYaw())
                : new ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation)
            );
        
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, ModuleConstants.maxSpeed);

        for(SwerveModule mod : swerveMods) {
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }
    }

    //To set states in auto:
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.maxSpeed);

        for(SwerveModule mod : swerveMods) {
            mod.setDesiredState(desiredStates[mod.getModuleNumber()], false);
        }
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : swerveMods) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : swerveMods) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getYaw() {
        return (SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    public void resetModulesToAbsolute() {
        for(SwerveModule mod: swerveMods) {
            mod.resetToAbsolute();
        }
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getModulePositions());

        for(SwerveModule mod : swerveMods) {
            SmartDashboard.putNumber(mod.getModuleName() + ": Cancoder", mod.getCanCoder().getDegrees());
            SmartDashboard.putNumber(mod.getModuleName() + ": Integrated", mod.getPosition().angle.getDegrees());
            SmartDashboard.putNumber(mod.getModuleName() + ": Velocity", mod.getState().speedMetersPerSecond);
        }
    }
}



