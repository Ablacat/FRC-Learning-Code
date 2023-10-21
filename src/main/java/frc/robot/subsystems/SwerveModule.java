package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.util.TunableNumber;
import frc.robot.Constants.Conversions;
import frc.robot.Constants.ModuleConstants;
import frc.robot.util.CTREModuleState;

public class SwerveModule {
    private int moduleNumber;
    private String moduleName;

    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private final TalonFX driveMotor;
    private final TalonFX angleMotor;
    private TalonFXConfiguration swerveAngleFXConfig;
    private TalonFXConfiguration swerveDriveFXConfig;
    
    private final CANCoder angleEncoder;
    private CANCoderConfiguration swerveCanCoderConfig;

    private final TunableNumber driveP = new TunableNumber("driveP", 0);
    private final TunableNumber driveI = new TunableNumber("driveI", 0);
    private final TunableNumber driveD = new TunableNumber("driveD", 0);
    private final TunableNumber driveF = new TunableNumber("driveF", 0);

    private final TunableNumber turnP = new TunableNumber("TurnP", 0);

    //NEED TO CHANGE #s
    private final TunableNumber driveKs = new TunableNumber("driveKs", .07 / 12.0);
    private final TunableNumber driveKv = new TunableNumber("driveKv", .27 / 12.0);
    private final TunableNumber driveKa = new TunableNumber("driveKa", .15 / 12.0);

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(driveKs.get(), driveKv.get(), driveKa.get());

    /**
     * Swerve Module Constants to be used when creating swerve modules.
     * @param moduleNumber
     * @param moduleName
     * @param driveMotorID
     * @param angleMotorID
     * @param canCoderID
     * @param angleOffset
     */
    public SwerveModule(int moduleNumber, String moduleName, int driveMotorID, int angleMotorID, int canCoderID, double offset) {
        this.moduleNumber = moduleNumber;
        angleOffset = Rotation2d.fromDegrees(offset);

        angleEncoder = new CANCoder(canCoderID);
        configAngleEncoder();

        angleMotor = new TalonFX(angleMotorID);
        configAngleMotor();

        driveMotor = new TalonFX(driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public String getModuleName() {
		return moduleName;
	}

	public void setModuleName(String moduleName) {
		this.moduleName = moduleName;
	}

	public int getModuleNumber() {
        return moduleNumber;
    }

    public void setModuleNumber(int moduleNumber) {
        this.moduleNumber = moduleNumber;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
        desiredState = CTREModuleState.optimize(desiredState, getState().angle);
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    
    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
        if(isOpenLoop) {
            double percentOutput = desiredState.speedMetersPerSecond / ModuleConstants.maxSpeed;
            driveMotor.set(ControlMode.PercentOutput, percentOutput);
        } else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, ModuleConstants.wheelCircumference, ModuleConstants.driveGearRatio);
            driveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState) {
        //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (ModuleConstants.maxSpeed * 0.01)) ? lastAngle : desiredState.angle;

        angleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), ModuleConstants.angleGearRatio));
        lastAngle = angle;
    }

    private Rotation2d getAngle() {
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(angleMotor.getSelectedSensorPosition(), ModuleConstants.angleGearRatio));
    }

    public Rotation2d getCanCoder() {
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
    }

    public void resetToAbsolute() {
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), ModuleConstants.angleGearRatio);
        angleMotor.setSelectedSensorPosition(absolutePosition);
    }

    private void configAngleEncoder() {
        angleEncoder.configFactoryDefault();
        angleEncoder.configAllSettings(swerveCanCoderConfig);
        swerveCanCoderConfig = new CANCoderConfiguration();

        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = false; //FOR MK4i/MK3
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
//ASK ABOUT LIVE PHOENIX TUNER...
    private void configAngleMotor() {
        angleMotor.configFactoryDefault();
        angleMotor.configAllSettings(swerveAngleFXConfig);
        swerveAngleFXConfig = new TalonFXConfiguration();

        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            ModuleConstants.angleEnableCurrentLimit,
            ModuleConstants.angleContinuousCurrentLimit,
            ModuleConstants.anglePeakCurrentLimit,
            ModuleConstants.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = turnP.get();
        swerveAngleFXConfig.slot0.kI = 0;
        swerveAngleFXConfig.slot0.kD = 0;
        swerveAngleFXConfig.slot0.kF = 0;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        angleMotor.setInverted(true); //FOR MK4i
        // angleMotor.setInverted(false); //FOR MK3
        angleMotor.setNeutralMode(ModuleConstants.angleNeutralMode);
        resetToAbsolute();
    }

    private void configDriveMotor() {
        driveMotor.configFactoryDefault();
        driveMotor.configAllSettings(swerveDriveFXConfig);
        swerveDriveFXConfig = new TalonFXConfiguration();

        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            ModuleConstants.driveEnableCurrentLimit,
            ModuleConstants.driveContinuousCurrentLimit,
            ModuleConstants.drivePeakCurrentLimit,
            ModuleConstants.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = driveP.get();
        swerveDriveFXConfig.slot0.kI = driveI.get();
        swerveDriveFXConfig.slot0.kD = driveD.get();
        swerveDriveFXConfig.slot0.kF = driveF.get();        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = ModuleConstants.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = ModuleConstants.closedLoopRamp;

        driveMotor.setInverted(false); //FOR MK4i/MK3
        driveMotor.setNeutralMode(ModuleConstants.driveNeutralMode);
        driveMotor.setSelectedSensorPosition(0);
    }
    public SwerveModuleState getState() {
        return new SwerveModuleState(
            Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), ModuleConstants.wheelCircumference, ModuleConstants.driveGearRatio),
            getAngle()
        );
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(), ModuleConstants.wheelCircumference, ModuleConstants.driveGearRatio),
            getAngle()
        );
    }
}
