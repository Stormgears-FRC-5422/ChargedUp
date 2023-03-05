package frc.utils.joysticks;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBoardConfig {

  ButtonBoard m_buttonboard1;
  ButtonBoard m_buttonboard2;

  public ButtonBoardConfig() {
    m_buttonboard1 = new ButtonBoard(1);
    m_buttonboard2 = new ButtonBoard(2);
  }

  public void buttonBoardSetup() {
    System.out.println("buttonBoardSetup starting");

    if (!m_buttonboard1.jumper()) {
      System.out.println("Switching ButtonBoard ports");
      m_buttonboard1 = new ButtonBoard(2);
      m_buttonboard2 = new ButtonBoard(1);
    } else {
      System.out.println("Not Switching ButtonBoard ports");
    }

    new Trigger(m_buttonboard1::leftSub).onTrue(new InstantCommand(() -> System.out.println("Left Sub Selected")));
    new Trigger(m_buttonboard1::rightSub).onTrue(new InstantCommand(() -> System.out.println("Right Sub Selected")));
    new Trigger(m_buttonboard1::floor).onTrue(new InstantCommand(() -> System.out.println("Floor Selected")));
    new Trigger(m_buttonboard1::store).onTrue(new InstantCommand(() -> System.out.println("Store Selected")));
    new Trigger(m_buttonboard1::grid1).onTrue(new InstantCommand(() -> System.out.println("Grid1 Selected")));
    new Trigger(m_buttonboard1::grid2).onTrue(new InstantCommand(() -> System.out.println("Grid2 Selected")));
    new Trigger(m_buttonboard1::grid3).onTrue(new InstantCommand(() -> System.out.println("Grid3 Selected")));
    new Trigger(m_buttonboard1::grid4).onTrue(new InstantCommand(() -> System.out.println("Grid4 Selected")));
    new Trigger(m_buttonboard1::grid5).onTrue(new InstantCommand(() -> System.out.println("Grid5 Selected")));
    new Trigger(m_buttonboard1::grid6).onTrue(new InstantCommand(() -> System.out.println("Grid6 Selected")));
    new Trigger(m_buttonboard1::grid7).onTrue(new InstantCommand(() -> System.out.println("Grid7 Selected")));
    new Trigger(m_buttonboard2::grid8).onTrue(new InstantCommand(() -> System.out.println("Grid8 Selected")));
    new Trigger(m_buttonboard2::grid9).onTrue(new InstantCommand(() -> System.out.println("Grid9 Selected")));
    new Trigger(m_buttonboard2::confirm).onTrue(new InstantCommand(() -> System.out.println("Confirm")));
    new Trigger(m_buttonboard2::cancel).onTrue(new InstantCommand(() -> System.out.println("Cancel")));
  }
}

