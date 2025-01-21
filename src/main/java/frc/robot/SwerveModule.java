package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.DemandType;
// import com.ctre.phoenix.motorcontrol.TalonFX;
// import com.ctre.phoenix.sensors.CANCoder;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d lastAngle;

    private TalonFX mAngleMotor;
    private TalonFX mDriveMotor;
    private CANcoder angleEncoder;

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        configAngleEncoder();

        /* Angle Motor Config */
        mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        configDriveMotor();

        lastAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            // mDriveMotor.set(ControlMode.PercentOutput, percentOutput);
            mDriveMotor.setControl(new DutyCycleOut(percentOutput));
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            // mDriveMotor.set(ControlMode.Velocity, velocity, DemandType.ArbitraryFeedForward, feedforward.calculate(desiredState.speedMetersPerSecond));
            mDriveMotor.setControl(new VelocityDutyCycle(velocity));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        // mAngleMotor.set(ControlMode.Position, Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio));
        mAngleMotor.setControl(new PositionDutyCycle(angle.getMeasure()));
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        // return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getSelectedSensorPosition(), Constants.Swerve.angleGearRatio));
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getPosition().getValueAsDouble(), Constants.Swerve.angleGearRatio));
    }

    public Rotation2d getCanCoder(){
        // return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
        return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        // mAngleMotor.setSelectedSensorPosition(absolutePosition);
        mAngleMotor.setPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        // angleEncoder.configFactoryDefault();
        // angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfigs);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCanCoderConfigs);
    }

    private void configAngleMotor(){
        // mAngleMotor.configFactoryDefault();
        // mAngleMotor.configAllSettings(Robot.ctreConfigs.swerveAngleFXConfig);
        // mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        // mAngleMotor.setNeutralMode(Constants.Swerve.angleNeutralMode);
        mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleConfigs);
        MotorOutputConfigs motorOutput = new MotorOutputConfigs();
        motorOutput.Inverted = Constants.Swerve.angleMotorInvert;
        motorOutput.NeutralMode = Constants.Swerve.angleNeutralMode;
        Robot.ctreConfigs.swerveAngleConfigs.MotorOutput = motorOutput; 
        mAngleMotor.getConfigurator().apply(motorOutput);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        // mDriveMotor.configFactoryDefault();
        // mDriveMotor.configAllSettings(Robot.ctreConfigs.swerveDriveFXConfig);
        // mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        // mDriveMotor.setNeutralMode(Constants.Swerve.driveNeutralMode);
        // mDriveMotor.setSelectedSensorPosition(0);
        mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveConfigs);
        MotorOutputConfigs motorOutput = new MotorOutputConfigs();
        motorOutput.Inverted = Constants.Swerve.driveMotorInvert;
        motorOutput.NeutralMode = Constants.Swerve.driveNeutralMode;
        Robot.ctreConfigs.swerveAngleConfigs.MotorOutput = motorOutput; 
        mDriveMotor.getConfigurator().apply(motorOutput);
    }

    public SwerveModuleState getState(){
        // return new SwerveModuleState(
        //     Conversions.falconToMPS(mDriveMotor.getSelectedSensorVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), getAngle()); 
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getVelocity().getValueAsDouble(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), getAngle()); 
    }

    public SwerveModulePosition getPosition(){
        // return new SwerveModulePosition(
        //     Conversions.falconToMeters(mDriveMotor.getSelectedSensorPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), getAngle());
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getPosition().getValueAsDouble(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), getAngle());
    }
}