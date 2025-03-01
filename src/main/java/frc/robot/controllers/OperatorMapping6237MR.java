package frc.robot.controllers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.rusthounds_util.CoralSim;
import frc.robot.rusthounds_util.ScoreLevel;
import frc.robot.rusthounds_util.GlobalStates;
import frc.robot.commands.RobotCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeArm;

public class OperatorMapping6237MR {
    public static void mapXboxController(CommandXboxController controller, CommandSwerveDrivetrain drivetrain, Elevator elevator, Arm arm, Intake intake,
    Climber climber, IntakeArm intakeArm, CoralSim coralSim) {
        
        controller.back().whileTrue(RobotCommands.kill(elevator, arm, intake, climber, intakeArm));
        controller.a().whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L1, elevator, arm, coralSim));
        controller.x().whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L2, elevator, arm, coralSim));
        controller.b().whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L3, elevator, arm, coralSim));
        controller.y().whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L4, elevator, arm, coralSim));
        controller.start().whileTrue(RobotCommands.scoreCoralCommand(drivetrain, elevator, arm, coralSim));

        controller.povUp().whileTrue(RobotCommands.prepareIntakeCoralCommand(elevator, arm, coralSim));
        controller.povDown().whileTrue(RobotCommands.intakeCoralCommand(elevator, arm, coralSim));

        controller.povLeft().whileTrue(RobotCommands.prepareAlgaeL2RemoveCommand(elevator, arm));
        controller.povRight().whileTrue(RobotCommands.prepareAlgaeL3RemoveCommand(elevator, arm));
        controller.leftStick().whileTrue(RobotCommands.algaeRemoveCommand(elevator, arm));
        
        //TEMP TEMP TEMP
        // controller.rightBumper().whileTrue(RobotCommands.intakeArmGivePositiveVoltage(intakeArm));
        // controller.leftBumper().whileTrue(RobotCommands.intakeArmGiveNegativeVoltage(intakeArm));

        controller.rightBumper().whileTrue(RobotCommands.intakeAlgaeCommand(intakeArm, intake));
        controller.leftBumper().whileTrue(RobotCommands.scoreAlgaeCommand(intakeArm, intake));
        
        climber.setDefaultCommand(Commands
        .run(() -> climber.setVoltage(MathUtil
                        .applyDeadband((controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()) * 4,
                                0.1)),
                        climber));
        
    }

    public static void mapXboxControllerManualOnlyControl(CommandXboxController controller, CommandSwerveDrivetrain drivetrain, Elevator elevator, Arm arm, Intake intake,
            Climber climber, IntakeArm intakeArm, CoralSim coralSim) {

        controller.back().whileTrue(RobotCommands.kill(elevator, arm, intake, climber, intakeArm));

        controller.x().whileTrue(RobotCommands.intakeArmGiveNegativeVoltage(intakeArm));
        controller.y().whileTrue(RobotCommands.intakeArmGivePositiveVoltage(intakeArm));

        controller.povLeft().whileTrue(RobotCommands.armOnlyGivePositiveVoltage(arm));
        controller.povRight().whileTrue(RobotCommands.armOnlyGiveNegativeVoltage(arm));

        controller.povDown().whileTrue(RobotCommands.elevatorOnlyGivePositiveVoltage(elevator));
        controller.povUp().whileTrue(RobotCommands.elevatorOnlyGiveNegativeVoltage(elevator));

        controller.rightBumper().whileTrue(intake.runRollersCommand());
        controller.leftBumper().whileTrue(intake.reverseRollersCommand());
        
        climber.setDefaultCommand(Commands
        .run(() -> climber.setVoltage(MathUtil
                        .applyDeadband((controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()) * 4,
                                0.1)),
                        climber));
        
    }
}
