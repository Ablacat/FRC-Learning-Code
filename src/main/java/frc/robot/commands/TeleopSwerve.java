package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DriveTrain;

public class TeleopSwerve extends CommandBase{
    private DriveTrain swerveDriveTrain;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public TeleopSwerve(DriveTrain swerveDriveTrain, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.swerveDriveTrain = swerveDriveTrain;
        addRequirements(swerveDriveTrain);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        //Get values/deadband
        double tranlationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
    
        swerveDriveTrain.drive(
            new Translation2d(tranlationVal, strafeVal).times(ModuleConstants.maxSpeed), 
            rotationVal * SwerveConstants.getMaxAngularSpeed(), 
            !robotCentricSup.getAsBoolean(), 
            true);
    }

}
