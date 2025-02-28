package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.rusthounds_util.PositionTracker;
import frc.robot.rusthounds_util.Utils;
import frc.robot.Constants.Arm.ArmPosition;
import frc.robot.Constants.IntakeArm.IntakeArmPosition;
import frc.robot.rusthounds_util.GlobalStates;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeArm.*;


public class IntakeArm extends SubsystemBase {
    private final SparkMax motor;

    private SparkMaxConfig motorConfig;

    private final ProfiledPIDController pidController = new ProfiledPIDController(kP, kI, kD, MOVEMENT_CONSTRAINTS);

    private final ArmFeedforward feedforwardController = new ArmFeedforward(kS,
            kG, kV, kA);

    /**
     * The representation of the "elevator" for simulation. (even though this is a
     * rotational mechanism w.r.t. its setpoints, we still control it as a linear
     * mechanism since that is the cloest physical mechanism to this)
     */
    private final SingleJointedArmSim armSim = new SingleJointedArmSim(
            MOTOR_GEARBOX_REPR,
            GEARING,
            MOI,
            COM_DISTANCE_METERS,
            MIN_ANGLE_RADIANS,
            MAX_ANGLE_RADIANS,
            true,
            ArmPosition.TOP.value);

    private double feedbackVoltage = 0;
    private double feedforwardVoltage = 0;
    private boolean initialized;

    public IntakeArm() {

        motorConfig = new SparkMaxConfig();
        motorConfig
                .inverted(MOTOR_INVERTED)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(CURRENT_LIMIT);
        motorConfig.encoder
                .positionConversionFactor(ENCODER_ROTATIONS_TO_METERS)
                .velocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0);

        motor = new SparkMax(MOTOR_ID, MotorType.kBrushless);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        resetPosition();
    }

    @Override
    public void simulationPeriodic() {
        armSim.setInput(motor.getAppliedOutput());
        armSim.update(0.020);
        motor.getEncoder().setPosition(armSim.getAngleRads());
    }

    public boolean getInitialized() {
        return initialized;
    }

    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    public double getAbsolutePositionMaybe(){
        return motor.getAbsoluteEncoder().getPosition();
    }

    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    public void resetPosition() {
        motor.getEncoder().setPosition(IntakeArmPosition.BOTTOM.value);
        initialized = true;
    }

    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        voltage = Utils.applySoftStops(voltage, getPosition(), IntakeArmPosition.TOP.value, IntakeArmPosition.BOTTOM.value); //TOP value is negative, so is minimum
        motor.setVoltage(voltage);
    }

    public Command testSetVoltage(double voltage){
        return Commands.sequence(
            Commands.runOnce(() -> motor.setVoltage(voltage), this)
        ).finallyDo(() -> motor.setVoltage(0));
    }

    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            feedbackVoltage = pidController.calculate(getPosition());
            // not the setpoint position, as smart people found that using the current
            // position for kG works best
            feedforwardVoltage = feedforwardController.calculate(getPosition(), pidController.getSetpoint().velocity);
            setVoltage(feedbackVoltage + feedforwardVoltage);
        }).withName("intakeArm.moveToCurrentGoal");
    }

    public Command moveToPositionCommand(Supplier<IntakeArmPosition> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get().value)),
                moveToCurrentGoalCommand()
                        .until(() -> pidController.atGoal()))
                .withTimeout(3)
                .withName("intakeArm.moveToPosition");
    }

    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get())),
                moveToCurrentGoalCommand().until(this::atGoal)).withName("arm.moveToArbitraryPosition");
    }

    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> pidController.getGoal().position + delta.get())
                .withName("intakeArm.movePositionDelta");
    }

    public Command holdCurrentPositionCommand() {
        return runOnce(() -> pidController.setGoal(getPosition())).andThen(moveToCurrentGoalCommand())
                .withName("intakeArm.holdCurrentPosition");
    }

    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("intakeArm.resetPosition");
    }

    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("intakeArm.setOverriddenSpeed");
    }

    public Command coastMotorsCommand() {
        return runOnce(motor::stopMotor)
                .andThen(() -> {
                    motorConfig.idleMode(IdleMode.kCoast);
                    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                })
                .finallyDo((d) -> {
                    motorConfig.idleMode(IdleMode.kBrake);
                    motor.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                    pidController.reset(getPosition());
                }).withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("intakeArm.coastMotorsCommand");
    }

    // public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
    //     return sysIdRoutine.quasistatic(direction).withName("elevator.sysIdQuasistatic");
    // }

    // public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
    //     return sysIdRoutine.dynamic(direction).withName("elevator.sysIdDynamic");
    // }

    public Command resetControllersCommand() {
        return Commands.runOnce(() -> pidController.reset(getPosition()))
                .andThen(Commands.runOnce(() -> pidController.setGoal(getPosition())));
    }

    public boolean atGoal() {
        return pidController.atGoal();
    }
}
