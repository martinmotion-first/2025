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

        // controller.a().whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L1, elevator, arm, coralSim));
        // controller.x().whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L2, elevator, arm, coralSim));
        // controller.b().whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L3, elevator, arm, coralSim));
        // controller.y().whileTrue(RobotCommands.prepareCoralScoreCommand(ScoreLevel.L4, elevator, arm, coralSim));
        controller.start().whileTrue(RobotCommands.scoreCoralCommand(drivetrain, elevator, arm, coralSim));

        controller.povUp().whileTrue(RobotCommands.prepareIntakeCoralCommand(elevator, arm, coralSim));
        controller.povDown().whileTrue(RobotCommands.intakeCoralCommand(elevator, arm, coralSim));

        controller.povLeft().whileTrue(RobotCommands.prepareAlgaeL2RemoveCommand(elevator, arm));
        controller.povRight().whileTrue(RobotCommands.prepareAlgaeL3RemoveCommand(elevator, arm));
        // controller.leftStick().whileTrue(RobotCommands.algaeRemoveCommand(drivetrain, elevator, arm));
        
        //TEMP TEMP TEMP
        // controller.rightBumper().whileTrue(RobotCommands.intakeArmGivePositiveVoltage(intakeArm));
        // controller.leftBumper().whileTrue(RobotCommands.intakeArmGiveNegativeVoltage(intakeArm));

        controller.rightBumper().whileTrue(RobotCommands.elevatorOnlyGivePositiveVoltage(elevator));
        controller.leftBumper().whileTrue(RobotCommands.elevatorOnlyGiveNegativeVoltage(elevator));

        controller.a().whileTrue(RobotCommands.armOnlyGivePositiveVoltage(arm));
        controller.x().whileTrue(RobotCommands.armOnlyGiveNegativeVoltage(arm));
        controller.b().whileTrue(RobotCommands.elevatorOnlyGiveZero(elevator));

        controller.back().whileTrue(RobotCommands.kill(elevator, arm, intake, climber, intakeArm));
        // controller.povRight().onTrue(GlobalStates.INITIALIZED.enableCommand());


        // controller.rightBumper().whileTrue(RobotCommands.elevatorOnlyMoveToBottom(elevator, arm, coralSim));
        // controller.leftBumper().whileTrue(RobotCommands.elevatorOnlyMoveToL3(elevator, arm, coralSim));

        // controller.rightBumper().whileTrue(RobotCommands.armOnlyMoveToBottom(elevator, arm, coralSim));
        // controller.leftBumper().whileTrue(RobotCommands.armOnlyMoveToHorizontal(elevator, arm, coralSim));


        // controller.rightBumper().whileTrue(intake.runRollersCommand());
        // controller.leftBumper().whileTrue(intake.reverseRollersCommand());
        //END TEMP TEMP TEMP
        
        climber.setDefaultCommand(Commands
                .run(() -> climber.setVoltage(MathUtil
                        .applyDeadband((controller.getRightTriggerAxis() - controller.getLeftTriggerAxis()) * 4,
                                0.1)),
                        climber));

        // controller.back().toggleOnTrue(
        //         Commands.parallel(
        //                 elevator.setOverridenSpeedCommand(() -> -controller.getLeftY() * 0.25),
        //                 arm.setOverridenSpeedCommand(() -> -controller.getRightY() * 0.25),
        //                 Commands.run(drivetrain::stop, drivetrain))

        //                 .finallyDo(() -> {
        //                     elevator.resetControllersCommand().schedule();
        //                     arm.resetControllersCommand().schedule();
        //                 }));
    }
}
