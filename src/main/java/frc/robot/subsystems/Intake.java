package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import static frc.robot.Constants.Intake.*;


public class Intake extends SubsystemBase implements BaseIntake {
    private final SparkFlex motor;

    private SparkMaxConfig motorConfig;

    public Intake() {
        motorConfig = new SparkMaxConfig();
        motorConfig
                .inverted(MOTOR_INVERTED)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(CURRENT_LIMIT);

        motor = new SparkFlex(MOTOR_ID, MotorType.kBrushless);
        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setRollerVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public Command runRollersCommand() {
        return Commands.startEnd(
                () -> setRollerVoltage(Constants.Intake.FORWARD_VOLTAGE),
                () -> setRollerVoltage(0))
                .withName("intake.runRollers");
    }
    @Override
    public Command reverseRollersCommand() {
        return Commands.startEnd(
                () -> setRollerVoltage(Constants.Intake.REVERSE_VOLTAGE),
                () -> setRollerVoltage(0))
                .withName("intake.reverseRollers");
    }
}
