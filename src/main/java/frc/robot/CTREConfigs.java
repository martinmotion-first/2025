package frc.robot;

// import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
// import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
// import com.ctre.phoenix.sensors.AbsoluteSensorRange;
// import com.ctre.phoenix.sensors.CANCoderConfiguration;
// import com.ctre.phoenix.sensors.SensorInitializationStrategy;
// import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class CTREConfigs {
    // public TalonFXConfiguration swerveAngleFXConfig;
    // public TalonFXConfiguration swerveDriveFXConfig;
    // public CANcoderConfiguration swerveCanCoderConfig;
    public TalonFXConfiguration swerveAngleConfigs;
    public TalonFXConfiguration swerveDriveConfigs;
    public CANcoderConfiguration swerveCanCoderConfigs;

    public CTREConfigs(){
        swerveAngleConfigs = new TalonFXConfiguration();
        swerveDriveConfigs = new TalonFXConfiguration();
        swerveCanCoderConfigs = new CANcoderConfiguration();

        /* Swerve Angle Motor Configurations */
        // SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
        //     Constants.Swerve.angleEnableCurrentLimit, 
        //     Constants.Swerve.angleContinuousCurrentLimit, 
        //     Constants.Swerve.anglePeakCurrentLimit, 
        //     Constants.Swerve.anglePeakCurrentDuration);
        CurrentLimitsConfigs angleSupplyLimit = new CurrentLimitsConfigs();
        angleSupplyLimit.SupplyCurrentLimitEnable = Constants.Swerve.angleEnableCurrentLimit;
        angleSupplyLimit.withSupplyCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
        angleSupplyLimit.withStatorCurrentLimit(Constants.Swerve.anglePeakCurrentLimit);
        
        // swerveAngleFXConfig.slot0.kP = Constants.Swerve.angleKP;
        // swerveAngleFXConfig.slot0.kI = Constants.Swerve.angleKI;
        // swerveAngleFXConfig.slot0.kD = Constants.Swerve.angleKD;
        // swerveAngleFXConfig.slot0.kF = Constants.Swerve.angleKF;
        // swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleConfigs.Slot0.kP = Constants.Swerve.angleKP;
        swerveAngleConfigs.Slot0.kI = Constants.Swerve.angleKI;
        swerveAngleConfigs.Slot0.kD = Constants.Swerve.angleKD;
        swerveAngleConfigs.Slot0.kV = Constants.Swerve.angleKF;  //NOTE! kV is not the same as kF but is the closest equivalent as far as I can tell...
        swerveAngleConfigs.withCurrentLimits(angleSupplyLimit); 

        /* Swerve Drive Motor Configuration */
        // SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
        //     Constants.Swerve.driveEnableCurrentLimit, 
        //     Constants.Swerve.driveContinuousCurrentLimit, 
        //     Constants.Swerve.drivePeakCurrentLimit, 
        //     Constants.Swerve.drivePeakCurrentDuration);
        CurrentLimitsConfigs driveSupplyLimit = new CurrentLimitsConfigs();
        driveSupplyLimit.SupplyCurrentLimitEnable = Constants.Swerve.driveEnableCurrentLimit;
        driveSupplyLimit.withSupplyCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
        driveSupplyLimit.withStatorCurrentLimit(Constants.Swerve.drivePeakCurrentLimit);

        // swerveDriveFXConfig.slot0.kP = Constants.Swerve.driveKP;
        // swerveDriveFXConfig.slot0.kI = Constants.Swerve.driveKI;
        // swerveDriveFXConfig.slot0.kD = Constants.Swerve.driveKD;
        // swerveDriveFXConfig.slot0.kF = Constants.Swerve.driveKF;        
        swerveDriveConfigs.Slot0.kP = Constants.Swerve.driveKP;
        swerveDriveConfigs.Slot0.kI = Constants.Swerve.driveKI;
        swerveDriveConfigs.Slot0.kD = Constants.Swerve.driveKD;
        swerveDriveConfigs.Slot0.kV = Constants.Swerve.driveKF;   //NOTE! kV is not the same as kF but is the closest equivalent as far as I can tell...
        // swerveDriveConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveConfigs.withCurrentLimits(driveSupplyLimit);
        // swerveDriveConfig.openloopRamp = Constants.Swerve.openLoopRamp;
        swerveDriveConfigs.OpenLoopRamps = new OpenLoopRampsConfigs(); 
        swerveAngleConfigs.OpenLoopRamps.withDutyCycleOpenLoopRampPeriod(Constants.Swerve.openLoopRamp);
        // swerveDriveConfig.closedloopRamp = Constants.Swerve.closedLoopRamp;
        swerveDriveConfigs.ClosedLoopRamps = new ClosedLoopRampsConfigs();
        swerveDriveConfigs.ClosedLoopRamps.withDutyCycleClosedLoopRampPeriod(Constants.Swerve.closedLoopRamp);
        
        /* Swerve CANCoder Configuration */
        // swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        // swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
        // swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        // swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        MagnetSensorConfigs sensorConfigs = new MagnetSensorConfigs();
        sensorConfigs.AbsoluteSensorDiscontinuityPoint = 1;
        sensorConfigs.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        swerveCanCoderConfigs.withMagnetSensor(sensorConfigs);
        // NOTE! This is as close as I could get to a phoenix6 version of the sensor config
    }
}