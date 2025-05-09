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
import frc.robot.rusthounds_util.GlobalStates;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Arm.*;


public class Arm extends SubsystemBase implements BaseSingleJointedArm<ArmPosition> {
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

    private double simVelocity = 0.0;

    private final MutVoltage sysidAppliedVoltageMeasure = Volts.mutable(0);
    private final MutAngle sysidPositionMeasure = Radians.mutable(0);
    private final MutAngularVelocity sysidVelocityMeasure = RadiansPerSecond.mutable(0);

    private final SysIdRoutine sysIdRoutine;

    private final PositionTracker positionTracker;
    private final MechanismLigament2d ligament;
    private final Supplier<Pose3d> carriagePoseSupplier;

    private boolean initialized;

    public Arm(PositionTracker positionTracker, MechanismLigament2d ligament, Supplier<Pose3d> carriagePoseSupplier) {

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

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(1).per(Second), Volts.of(3), null, null),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> setVoltage(volts.magnitude()),
                        log -> {
                            log.motor("primary")
                                    .voltage(sysidAppliedVoltageMeasure.mut_replace(motor.getAppliedOutput(), Volts))
                                    .angularPosition(sysidPositionMeasure.mut_replace(getPosition(), Radians))
                                    .angularVelocity(sysidVelocityMeasure.mut_replace(getVelocity(), RadiansPerSecond));
                        },
                        this));

        this.positionTracker = positionTracker;
        this.ligament = ligament;
        this.carriagePoseSupplier = carriagePoseSupplier;

        positionTracker.setArmAngleSupplier(this::getPosition);


        // setDefaultCommand(holdCurrentPositionCommand());
        resetPosition();
    }

    @Override
    public void simulationPeriodic() {
        armSim.setInput(motor.getAppliedOutput());
        armSim.update(0.020);
        motor.getEncoder().setPosition(armSim.getAngleRads());
        simVelocity = armSim.getVelocityRadPerSec();
        ligament.setAngle(Units.radiansToDegrees(getPosition()) + 270);
    }

    public boolean getInitialized() {
        return initialized;
    }

    // return new Pose3d(0.168, 0, 0.247, new Rotation3d());
    // -0.083

    public Pose3d getArmComponentPose() {
        return carriagePoseSupplier.get()
                .plus(new Transform3d(0.083, 0, 0, new Rotation3d()))
                .plus(new Transform3d(0, 0, 0, new Rotation3d(0, -getPosition(), 0)));
    }

    public Pose3d getClawComponentPose() {
        return getArmComponentPose().plus(new Transform3d(0.2585, 0, 0, new Rotation3d()));
    }

    @Override
    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    public double getVelocity() {
        return motor.getEncoder().getVelocity();
    }

    @Override
    public void resetPosition() {
        // motor.getEncoder().setPosition(ArmPosition.TOP.value);
        initialized = true;
    }

    @Override
    public void setVoltage(double voltage) {
        voltage = MathUtil.clamp(voltage, -12, 12);
        // voltage = Utils.applySoftStops(voltage, getPosition(), ArmPosition.BOTTOM.value, ArmPosition.TOP.value);

        // if (getPosition() > Constants.Arm.MAX_ARM_EXTENSION_TO_ALLOW_ELEVATOR_DESCENT
        //         && positionTracker.getElevatorPosition() < Constants.Elevator.MIN_HEIGHT_TO_ALLOW_ARM_EXTENSION) {
        //     System.out.println("Arm safety break fired");
        //     voltage = 0;
        // }

        // if (!GlobalStates.INITIALIZED.enabled()) {
        //     voltage = 0.0;
        // }

        motor.setVoltage(voltage);
    }

    @Override
    public Command moveToCurrentGoalCommand() {
        return run(() -> {
            feedbackVoltage = pidController.calculate(getPosition());
            // not the setpoint position, as smart people found that using the current
            // position for kG works best
            feedforwardVoltage = feedforwardController.calculate(getPosition(), pidController.getSetpoint().velocity);
            setVoltage(feedbackVoltage + feedforwardVoltage);
        }).withName("arm.moveToCurrentGoal");
    }

    public Command moveToCurrentGoalForScoreDoubleSpeed() {
        return run(() -> {
            feedbackVoltage = pidController.calculate(getPosition());
            // not the setpoint position, as smart people found that using the current
            // position for kG works best
            feedforwardVoltage = feedforwardController.calculate(getPosition(), pidController.getSetpoint().velocity * 2);
            setVoltage(feedbackVoltage + feedforwardVoltage);
        }).withName("arm.moveToCurrentGoal");
    }
 
    @Override
    public Command moveToPositionCommand(Supplier<ArmPosition> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get().value)),
                moveToCurrentGoalCommand()
                        .until(() -> pidController.atGoal()))
                // .andThen(holdCurrentPositionCommand())
                // .withTimeout(3)
                .withName("arm.moveToPosition");
    }

    public Command moveToPositionCommandAlternate(ArmPosition armPosition) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(armPosition.value)),
                moveToCurrentGoalCommand()
                        .until(() -> pidController.atGoal()))
                // .withTimeout(3)
                .withName("arm.moveToPositionCommandAlternate");
    }

    public Command moveToScoreCoral(ArmPosition armPosition) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(armPosition.value)),
                moveToCurrentGoalForScoreDoubleSpeed()
                        .until(() -> pidController.atGoal()))
                // .withTimeout(3)
                .withName("arm.moveToPositionCommandAlternate");
    }

    @Override
    public Command moveToArbitraryPositionCommand(Supplier<Double> goalPositionSupplier) {
        return Commands.sequence(
                runOnce(() -> pidController.reset(getPosition())),
                runOnce(() -> pidController.setGoal(goalPositionSupplier.get())),
                moveToCurrentGoalCommand().until(this::atGoal)).withName("arm.moveToArbitraryPosition");
    }

    @Override
    public Command movePositionDeltaCommand(Supplier<Double> delta) {
        return moveToArbitraryPositionCommand(() -> pidController.getGoal().position + delta.get())
                .withName("arm.movePositionDelta");
    }

    @Override
    public Command holdCurrentPositionCommand() {
        return runOnce(() -> pidController.setGoal(getPosition())).andThen(moveToCurrentGoalCommand())
                .withName("arm.holdCurrentPosition");
    }

    @Override
    public Command resetPositionCommand() {
        return runOnce(this::resetPosition).withName("arm.resetPosition");
    }

    @Override
    public Command setOverridenSpeedCommand(Supplier<Double> speed) {
        return runEnd(() -> setVoltage(12.0 * speed.get()), () -> setVoltage(0))
                .withName("arm.setOverriddenSpeed");
    }

    @Override
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
                .withName("arm.coastMotorsCommand");
    }

    public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction).withName("elevator.sysIdQuasistatic");
    }

    public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction).withName("elevator.sysIdDynamic");
    }

    public Command resetControllersCommand() {
        return Commands.runOnce(() -> pidController.reset(getPosition()))
                .andThen(Commands.runOnce(() -> pidController.setGoal(getPosition())));
    }

    public boolean atGoal() {
        return pidController.atGoal();
    }

    public Command testSetVoltage(double voltage){
        return Commands.sequence(
            Commands.run(() -> motor.setVoltage(voltage), this)
        );
    }
}
