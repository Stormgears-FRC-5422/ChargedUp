package frc.utils.joysticks;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ButtonBoardConfig {

  ButtonBoard m_buttonboard1;
  ButtonBoard m_buttonboard4;

  private int grid1, grid2, grid3, grid4, grid5, grid6, grid7, grid8, grid9;
  int[] gridState = {grid1, grid2, grid3, grid4, grid5, grid6, grid7, grid8, grid9};

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
    new Trigger(m_buttonboard4::grid1).onTrue(new InstantCommand(() -> grid1 = 1));
    new Trigger(m_buttonboard4::grid2).onTrue(new InstantCommand(() -> grid2 = 2));
    new Trigger(m_buttonboard4::grid3).onTrue(new InstantCommand(() -> grid3 = 3));
    new Trigger(m_buttonboard4::grid4).onTrue(new InstantCommand(() -> grid4 = 4));
    new Trigger(m_buttonboard4::grid5).onTrue(new InstantCommand(() -> grid5 = 5));
    new Trigger(m_buttonboard4::grid6).onTrue(new InstantCommand(() -> grid6 = 6));
    new Trigger(m_buttonboard4::grid7).onTrue(new InstantCommand(() -> grid7 = 7));
    new Trigger(m_buttonboard4::grid8).onTrue(new InstantCommand(() -> grid8 = 8));
    new Trigger(m_buttonboard4::grid9).onTrue(new InstantCommand(() -> grid9 = 9));
    /*
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
    */
    new Trigger(m_buttonboard1::confirm).onTrue(new InstantCommand(() ->(
    for(int i = 0; i < 9; i++){
      if(gridState[i] == 1){
        new InstantCommand(() -> System.out.println("Grid1 Running"));
        gridState[i] = 0;
      }
      else if(gridState[i] == 2){
        new InstantCommand(() -> System.out.println("Grid2 Running"));
        gridState[i] = 0;
      }
      else if(gridState[i] == 3){
        new InstantCommand(() -> System.out.println("Grid3 Running"));
        gridState[i] = 0;
      }
      else if(gridState[i] == 4){
        new InstantCommand(() -> System.out.println("Grid4 Running"));
        gridState[i] = 0;
      }
      else if(gridState[i] == 5){
        new InstantCommand(() -> System.out.println("Grid5 Running"));
        gridState[i] = 0;
      }
      else if(gridState[i] == 6){
        new InstantCommand(() -> System.out.println("Grid6 Running"));
        gridState[i] = 0;
      }
      else if(gridState[i] == 7){
        new InstantCommand(() -> System.out.println("Grid7 Running"));
        gridState[i] = 0;
      }
      else if(gridState[i] == 8){
        new InstantCommand(() -> System.out.println("Grid8 Running"));
        gridState[i] = 0;
      }
      else if(gridState[i] == 9){
        new InstantCommand(() -> System.out.println("Grid9 Running"));
        gridState[i] = 0;
      }
      else if(gridState[i] == 0){
        new InstantCommand(() -> System.out.println("Nothing Running"));
      }
    }
    new Trigger(m_buttonboard1::cancel).onTrue(new InstantCommand(() -> {}
            for(int i = 0; i < 9; i++) {
              gridState[i] = 0;
            }
    }
  }
}

