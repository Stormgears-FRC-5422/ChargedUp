package frc.utils.joysticks;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBoardConfig {

  ButtonBoard m_buttonboard1;
  ButtonBoard m_buttonboard4;

  public ButtonBoardConfig() {
    m_buttonboard1 = new ButtonBoard(1);
    m_buttonboard4 = new ButtonBoard(4);
  }


  public void buttonBoardSetup() throws KillSwitch {
    System.out.println("buttonBoardSetup starting");

    if (!m_buttonboard1.jumper()) {
      System.out.println("Switching ButtonBoard ports");
      m_buttonboard1 = new ButtonBoard(4);
      m_buttonboard4 = new ButtonBoard(1);
    } else {
      System.out.println("Not Switching ButtonBoard ports");
    }



    if (m_buttonboard4.kill()) {
      System.out.println("Killing robot");
      throw new KillSwitch("Killing robot");

    }

    new Trigger(m_buttonboard1::leftSub).onTrue(new InstantCommand(() -> System.out.println("Left Sub Selected")));
    new Trigger(m_buttonboard1::rightSub).onTrue(new InstantCommand(() -> System.out.println("Right Sub Selected")));
    new Trigger(m_buttonboard1::floor).onTrue(new InstantCommand(() -> System.out.println("Floor Selected")));
    new Trigger(m_buttonboard1::store).onTrue(new InstantCommand(() -> System.out.println("Store Selected")));
    new Trigger(m_buttonboard4::grid1).onTrue(new InstantCommand(() -> System.out.println("Grid1 Selected")));
    new Trigger(m_buttonboard4::grid2).onTrue(new InstantCommand(() -> System.out.println("Grid2 Selected")));
    new Trigger(m_buttonboard4::grid3).onTrue(new InstantCommand(() -> System.out.println("Grid3 Selected")));
    new Trigger(m_buttonboard4::grid4).onTrue(new InstantCommand(() -> System.out.println("Grid4 Selected")));
    new Trigger(m_buttonboard4::grid5).onTrue(new InstantCommand(() -> System.out.println("Grid5 Selected")));
    new Trigger(m_buttonboard4::grid6).onTrue(new InstantCommand(() -> System.out.println("Grid6 Selected")));
    new Trigger(m_buttonboard4::grid7).onTrue(new InstantCommand(() -> System.out.println("Grid7 Selected")));
    new Trigger(m_buttonboard4::grid8).onTrue(new InstantCommand(() -> System.out.println("Grid8 Selected")));
    new Trigger(m_buttonboard4::grid9).onTrue(new InstantCommand(() -> System.out.println("Grid9 Selected")));
    new Trigger(m_buttonboard1::confirm).onTrue(new InstantCommand(() -> System.out.println("Confirm")));
    new Trigger(m_buttonboard1::cancel).onTrue(new InstantCommand(() -> System.out.println("Cancel")));
  }
}

