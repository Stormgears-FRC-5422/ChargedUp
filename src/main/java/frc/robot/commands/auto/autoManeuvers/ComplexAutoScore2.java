package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.arm.pathFollowing.StowArm;
import frc.robot.commands.drive.pathFollowing.DriveToPose;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drive.DrivetrainBase;
import frc.utils.joysticks.DriveJoystick;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import static frc.robot.constants.FieldConstants.Grids.ScoringNode;

public class ComplexAutoScore2 extends SequentialCommandGroup {

    public ComplexAutoScore2(DrivetrainBase drivetrain, Arm arm, Compression compression,
                             Supplier<ScoringNode> nodeSupplier, DriveJoystick joystick, BooleanSupplier confirm) {

        // use quick method if it is hybrid or a cube everything is automatic
        BooleanSupplier useDirect = () -> ((nodeSupplier.get().type == ScoringNode.NodeType.CUBE) ||
                nodeSupplier.get().height == ScoringNode.NodeHeight.HYBRID);

        addCommands(
                new ConditionalCommand(
                        getDirectCommand(drivetrain, arm, compression, nodeSupplier),
                        getNormalCommand(drivetrain, arm, compression, nodeSupplier, joystick, confirm),
                        useDirect
                ));

        handleInterrupt(
                () -> new StowArm(arm).schedule());
    }

    private CommandBase getDirectCommand(DrivetrainBase drivetrain, Arm arm, Compression compression,
                                         Supplier<ScoringNode> nodeSupplier) {
        return new SequentialCommandGroup(
                new DriveToPose(drivetrain, () -> nodeSupplier.get().scoringPosition, 4, 3),
                new ArmToNode(arm, nodeSupplier),
                compression.getReleaseCommand(),
                new StowArm(arm));
    }

    private CommandBase getNormalCommand(DrivetrainBase drivetrain, Arm arm, Compression compression,
                                         Supplier<ScoringNode> nodeSupplier, DriveJoystick joystick, BooleanSupplier confirm) {
        return new ParallelCommandGroup(
                new AlignToNode(drivetrain, joystick, nodeSupplier),
                new SequentialCommandGroup(
                        new WaitUntilCommand(confirm),
                        new ArmToNode(arm, nodeSupplier),
                        new WaitUntilCommand(confirm),
                        compression.getReleaseCommand(),
                        new StowArm(arm))
        );
    }
}