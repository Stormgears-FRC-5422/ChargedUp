package frc.utils.joysticks;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.autoScoring.NodeSelector;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.NeoPixel;
import frc.robot.commands.autoScoring.PlaceGamePiece;
import frc.robot.commands.autoScoring.PlaceGamePiece.Level;
import frc.robot.commands.autoScoring.PlaceGamePiece.gamePiece;

public class ButtonBoardConfig {

  ButtonBoard m_buttonboard1;
  ButtonBoard m_buttonboard2;
  public Level m_Level;
  public gamePiece m_gamePiece;

  NeoPixel neoPixel;
  public int grid;

  public NodeSelector nodeSelector;

  Compression compression;

  public ButtonBoardConfig(NeoPixel neoPixel, NodeSelector nodeSelector, Compression compression) {
    m_buttonboard1 = new ButtonBoard(1);
    m_buttonboard2 = new ButtonBoard(2);
    this.neoPixel = neoPixel;
    this.nodeSelector = nodeSelector;
    this.compression = compression;
  }

  public void buttonBoardSetup(){
    int[] segments1 = {1, 2};

    System.out.println("buttonBoardSetup starting");

    if (!m_buttonboard2.jumper()) {
      System.out.println("Switching ButtonBoard ports");
      m_buttonboard1 = new ButtonBoard(2);
      m_buttonboard2 = new ButtonBoard(1);
    } else {
      System.out.println("Not Switching ButtonBoard ports");
    }

    new Trigger(m_buttonboard1::kill).onTrue(new InstantCommand(() -> System.exit(0)));

    new Trigger(m_buttonboard1::floor)
        .onTrue(new InstantCommand(() -> System.out.println("Floor Selected")))
        .and(new Trigger(m_buttonboard1::cubeCone).onTrue(
             new InstantCommand(() -> neoPixel.setSpecificSegmentColor(segments1, NeoPixel.PURPLE_COLOR))));

    new Trigger(m_buttonboard1::floor)
            .onTrue(new InstantCommand(() -> System.out.println("Floor Selected")))
            .and(new Trigger(m_buttonboard1::cubeCone).onTrue(
                    new InstantCommand(() -> neoPixel.setSpecificSegmentColor(segments1, NeoPixel.YELLOW_COLOR))));

    new Trigger(m_buttonboard1::leftSub)
            .onTrue(new InstantCommand(() -> System.out.println("Left Sub Selected")))
            .and(new Trigger(m_buttonboard1::cubeCone).onTrue(
                    new InstantCommand(() -> neoPixel.setColor(3, NeoPixel.PURPLE_COLOR))));

    new Trigger(m_buttonboard1::leftSub)
            .onTrue(new InstantCommand(() -> System.out.println("Let Sub Selected")))
            .and(new Trigger(m_buttonboard1::cubeCone).onTrue(
                    new InstantCommand(() -> neoPixel.setColor(3, NeoPixel.YELLOW_COLOR))));

    new Trigger(m_buttonboard1::rightSub)
            .onTrue(new InstantCommand(() -> System.out.println("Right Sub Selected")))
            .and(new Trigger(m_buttonboard1::cubeCone).onTrue(
                    new InstantCommand(() -> neoPixel.setColor(3, NeoPixel.PURPLE_COLOR))));

    new Trigger(m_buttonboard1::rightSub)
            .onTrue(new InstantCommand(() -> System.out.println("Right Sub Selected")))
            .and(new Trigger(m_buttonboard1::cubeCone).onTrue(
                    new InstantCommand(() -> neoPixel.setColor(3, NeoPixel.YELLOW_COLOR))));

//
//    new Trigger(m_buttonboard1::leftSub)
//        .onTrue(new InstantCommand(() -> System.out.println("Left Sub Selected")))
//        .and(new Trigger(m_buttonboard1::cubeCone).onTrue(
//             new InstantCommand(() -> neoPixel.setAllColor( NeoPixel.PURPLE_COLOR))))
//        .or( new Trigger(m_buttonboard1::cubeCone).onFalse(
//             new InstantCommand(() -> neoPixel.setAllColor( NeoPixel.YELLOW_COLOR))));
//
//    new Trigger(m_buttonboard1::rightSub)
//        .onTrue(new InstantCommand(() -> System.out.println("Left Sub Selected")))
//        .and(new Trigger(m_buttonboard1::cubeCone).onTrue(
//             new InstantCommand(() -> neoPixel.setAllColor( NeoPixel.PURPLE_COLOR))))
//        .or( new Trigger(m_buttonboard1::cubeCone).onFalse(
//             new InstantCommand(() -> neoPixel.setAllColor( NeoPixel.YELLOW_COLOR))));

    new Trigger(m_buttonboard1::store).onTrue(new InstantCommand(() -> System.out.println("Store Selected")));

//    new Trigger(m_buttonboard1::manualOverride).onTrue(new InstantCommand(() -> System.out.println("Manuel Arm Override")));



//    new Trigger(m_buttonboard1::cubeCone).onTrue(new InstantCommand(() -> m_gamePiece = gamePiece.CUBE));
//    new Trigger(m_buttonboard1::cubeCone).onFalse(new InstantCommand(() -> m_gamePiece = gamePiece.CONE));

    new Trigger(m_buttonboard1::gripper).onTrue(new InstantCommand(compression::grabCubeOrCone));
    new Trigger(m_buttonboard1::gripper).onFalse(new InstantCommand(compression::release));

    // set row of node selector
    new Trigger(() -> m_buttonboard1.getRawButton(5) && !m_buttonboard1.getRawButton(6))
            .onTrue(new InstantCommand(() -> nodeSelector.setSelectedRow(2)));
    new Trigger(() -> !m_buttonboard1.getRawButton(5) && !m_buttonboard1.getRawButton(6))
            .onTrue(new InstantCommand(() -> nodeSelector.setSelectedRow(1)));
    new Trigger(() -> m_buttonboard1.getRawButton(6) && !m_buttonboard1.getRawButton(5))
            .onTrue(new InstantCommand(() -> nodeSelector.setSelectedRow(0)));

    int offset = 4;
    for (int i = 8; i >= 0; i--) {
      int temp = i;
      new Trigger(() -> m_buttonboard2.getRawButton(temp + offset))
              .onTrue(new InstantCommand(() -> nodeSelector.setSelectedCol(temp)));
    }


//    new Trigger(m_buttonboard1::cancel).onTrue(new InstantCommand(() -> { grid = 0;}));

  }

  public boolean confirm() {
    return m_buttonboard1.getRawButton(3);
  }

  public boolean stow() {
    return m_buttonboard1.store();
  }

  public boolean cancel() {
    return m_buttonboard1.cancel();
  }

  public boolean pickFloor() {
    return m_buttonboard1.floor();
  }
}
