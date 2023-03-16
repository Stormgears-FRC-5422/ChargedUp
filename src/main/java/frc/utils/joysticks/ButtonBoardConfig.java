package frc.utils.joysticks;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.NeoPixel;
import frc.robot.commands.autoScoring.placeGamePiece;

public class ButtonBoardConfig {

  ButtonBoard m_buttonboard1;
  ButtonBoard m_buttonboard2;

  enum Level {
    LOW,
    MEDIUM,
    HIGH
  }

  enum gamePiece {
    CONE,
    CUBE
  }

  Level m_Level;

  gamePiece m_gamePiece;

  Exception killSwitch;

  NeoPixel neoPixel;

  private int grid;


  public ButtonBoardConfig(NeoPixel neoPixel) {
    m_buttonboard1 = new ButtonBoard(1);
    m_buttonboard2 = new ButtonBoard(2);
    this.neoPixel = neoPixel;
  }


  public void buttonBoardSetup(){
    int[] segments = {1, 2};
    killSwitch =  new Exception("Kill Switch");
    System.out.println("buttonBoardSetup starting");



    if (!m_buttonboard2.jumper()) {
      System.out.println("Switching ButtonBoard ports");
      m_buttonboard1 = new ButtonBoard(2);
      m_buttonboard2 = new ButtonBoard(1);
    } else {
      System.out.println("Not Switching ButtonBoard ports");
    }





<<<<<<< Updated upstream
//    new Trigger(m_buttonboard2::grid1).onTrue(new InstantCommand(() -> grid1 = 1));
//    new Trigger(m_buttonboard2::grid2).onTrue(new InstantCommand(() -> grid2 = 2));
//    new Trigger(m_buttonboard2::grid3).onTrue(new InstantCommand(() -> grid3 = 3));
//    new Trigger(m_buttonboard2::grid4).onTrue(new InstantCommand(() -> grid4 = 4));
//    new Trigger(m_buttonboard2::grid5).onTrue(new InstantCommand(() -> grid5 = 5));
//    new Trigger(m_buttonboard2::grid6).onTrue(new InstantCommand(() -> grid6 = 6));
//    new Trigger(m_buttonboard2::grid7).onTrue(new InstantCommand(() -> grid7 = 7));
//    new Trigger(m_buttonboard2::grid8).onTrue(new InstantCommand(() -> grid8 = 8));
//    new Trigger(m_buttonboard2::grid9).onTrue(new InstantCommand(() -> grid9 = 9));
=======

//    if (m_buttonboard4.kill()) {
//      System.out.println("Killing robot");
//      throw new KillSwitch("Killing robot");
//
//    }

    new Trigger(m_buttonboard1::leftSub).onTrue(new InstantCommand(() -> System.out.println("Left Sub Selected")));
    new Trigger(m_buttonboard1::rightSub).onTrue(new InstantCommand(() -> System.out.println("Right Sub Selected")));
    new Trigger(m_buttonboard2::grid1).onTrue(new InstantCommand(() -> grid = 1));
    new Trigger(m_buttonboard2::grid2).onTrue(new InstantCommand(() -> grid = 2));
    new Trigger(m_buttonboard2::grid3).onTrue(new InstantCommand(() -> grid = 3));
    new Trigger(m_buttonboard2::grid4).onTrue(new InstantCommand(() -> grid = 4));
    new Trigger(m_buttonboard2::grid5).onTrue(new InstantCommand(() -> grid = 5));
    new Trigger(m_buttonboard2::grid6).onTrue(new InstantCommand(() -> grid = 6));
    new Trigger(m_buttonboard2::grid7).onTrue(new InstantCommand(() -> grid = 7));
    new Trigger(m_buttonboard2::grid8).onTrue(new InstantCommand(() -> grid = 8));
    new Trigger(m_buttonboard2::grid9).onTrue(new InstantCommand(() -> grid = 9));
    new Trigger(m_buttonboard1::High).onTrue(new InstantCommand(() -> m_Level = Level.HIGH));
    new Trigger(m_buttonboard1::Medium).onTrue(new InstantCommand(() -> m_Level = Level.MEDIUM));
    new Trigger(m_buttonboard1::Low).onTrue(new InstantCommand(() -> m_Level = Level.LOW));
    new Trigger(m_buttonboard1::Cone).onTrue(new InstantCommand(() -> m_gamePiece = gamePiece.CONE));
    new Trigger(m_buttonboard1::Cone).onTrue(new InstantCommand(() -> m_gamePiece = gamePiece.CUBE));
>>>>>>> Stashed changes
    new Trigger(m_buttonboard1::kill).onTrue(new InstantCommand(() -> System.exit(0)));
    new Trigger(m_buttonboard1::floor).onTrue(new InstantCommand(() -> System.out.println("Floor Selected"))).and( new Trigger(m_buttonboard1::cubeCone).onTrue(new InstantCommand(() -> neoPixel.setAllColor( NeoPixel.PURPLE_COLOR)))).or(new Trigger(m_buttonboard1::cubeCone).onFalse(new InstantCommand(() -> neoPixel.setAllColor( NeoPixel.YELLOW_COLOR))););
    new Trigger(m_buttonboard1::leftSub).onTrue(new InstantCommand(() -> System.out.println("Left Sub Selected"))).and( new Trigger(m_buttonboard1::cubeCone).onTrue(new InstantCommand(() -> neoPixel.setAllColor( NeoPixel.PURPLE_COLOR)))).or(new Trigger(m_buttonboard1::cubeCone).onFalse(new InstantCommand(() -> neoPixel.setAllColor( NeoPixel.YELLOW_COLOR))););
    new Trigger(m_buttonboard1::rightSub).onTrue(new InstantCommand(() -> System.out.println("Left Sub Selected"))).and( new Trigger(m_buttonboard1::cubeCone).onTrue(new InstantCommand(() -> neoPixel.setAllColor( NeoPixel.PURPLE_COLOR)))).or(new Trigger(m_buttonboard1::cubeCone).onFalse(new InstantCommand(() -> neoPixel.setAllColor( NeoPixel.YELLOW_COLOR))););
    new Trigger(m_buttonboard1::store).onTrue(new InstantCommand(() -> System.out.println("Store Selected")));
    new Trigger(m_buttonboard1::manualOverride).onTrue(new InstantCommand(() -> System.out.println("Manuel Arm Override")));
    new Trigger(m_buttonboard2::grid1).onTrue(new InstantCommand(() -> System.out.println("Grid1 Selected")));
    new Trigger(m_buttonboard2::grid2).onTrue(new InstantCommand(() -> System.out.println("Grid2 Selected")));
    new Trigger(m_buttonboard2::grid3).onTrue(new InstantCommand(() -> System.out.println("Grid3 Selected")));
    new Trigger(m_buttonboard2::grid4).onTrue(new InstantCommand(() -> System.out.println("Grid4 Selected")));
    new Trigger(m_buttonboard2::grid5).onTrue(new InstantCommand(() -> System.out.println("Grid5 Selected")));
    new Trigger(m_buttonboard2::grid6).onTrue(new InstantCommand(() -> System.out.println("Grid6 Selected")));
    new Trigger(m_buttonboard2::grid7).onTrue(new InstantCommand(() -> System.out.println("Grid7 Selected")));
    new Trigger(m_buttonboard2::grid8).onTrue(new InstantCommand(() -> System.out.println("Grid8 Selected")));
    new Trigger(m_buttonboard2::grid9).onTrue(new InstantCommand(() -> System.out.println("Grid9 Selected")));
    new Trigger(m_buttonboard1::gripper).onTrue(new InstantCommand(() -> System.out.println("Gripper Closed")));
    new Trigger(m_buttonboard1::gripper).onFalse(new InstantCommand(() -> System.out.println("Gripper Open")));


//    new Trigger(m_buttonboard1::confirm).onTrue(new InstantCommand(() ->(
//    for(int i = 0; i < 9; i++){
//      if(gridState[i] == 1){
//        new InstantCommand(() -> System.out.println("Grid1 Running"));
//        gridState[i] = 0;
//      }
//      else if(gridState[i] == 2){
//        new InstantCommand(() -> System.out.println("Grid2 Running"));
//        gridState[i] = 0;
//      }
//      else if(gridState[i] == 3){
//        new InstantCommand(() -> System.out.println("Grid3 Running"));
//        gridState[i] = 0;
//      }
//      else if(gridState[i] == 4){
//        new InstantCommand(() -> System.out.println("Grid4 Running"));
//        gridState[i] = 0;
//      }
//      else if(gridState[i] == 5){
//        new InstantCommand(() -> System.out.println("Grid5 Running"));
//        gridState[i] = 0;
//      }
//      else if(gridState[i] == 6){
//        new InstantCommand(() -> System.out.println("Grid6 Running"));
//        gridState[i] = 0;
//      }
//      else if(gridState[i] == 7){
//        new InstantCommand(() -> System.out.println("Grid7 Running"));
//        gridState[i] = 0;
//      }
//      else if(gridState[i] == 8){
//        new InstantCommand(() -> System.out.println("Grid8 Running"));
//        gridState[i] = 0;
//      }
//      else if(gridState[i] == 9){
//        new InstantCommand(() -> System.out.println("Grid9 Running"));
//        gridState[i] = 0;
//      }
//      else if(gridState[i] == 0){
//        new InstantCommand(() -> System.out.println("Nothing Running"));
//      }
//    }
//    new Trigger(m_buttonboard1::cancel).onTrue(new InstantCommand(() ->
//            for(int i = 0; i < 9; i++) {
//              gridState[i] = 0;
//            }
//    }
<<<<<<< Updated upstream
//
//
//
//    new Trigger(m_buttonboard2::grid1).onTrue(new InstantCommand(() -> System.out.println("Grid1 Selected")));
//    new Trigger(m_buttonboard2::grid2).onTrue(new InstantCommand(() -> System.out.println("Grid2 Selected")));
//    new Trigger(m_buttonboard2::grid3).onTrue(new InstantCommand(() -> System.out.println("Grid3 Selected")));
//    new Trigger(m_buttonboard2::grid4).onTrue(new InstantCommand(() -> System.out.println("Grid4 Selected")));
//    new Trigger(m_buttonboard2::grid5).onTrue(new InstantCommand(() -> System.out.println("Grid5 Selected")));
//    new Trigger(m_buttonboard2::grid6).onTrue(new InstantCommand(() -> System.out.println("Grid6 Selected")));
//    new Trigger(m_buttonboard2::grid7).onTrue(new InstantCommand(() -> System.out.println("Grid7 Selected")));
//    new Trigger(m_buttonboard2::grid8).onTrue(new InstantCommand(() -> System.out.println("Grid8 Selected")));
//    new Trigger(m_buttonboard2::grid9).onTrue(new InstantCommand(() -> System.out.println("Grid9 Selected")));
//    new Trigger(m_buttonboard1::confirm).onTrue(new InstantCommand(() -> System.out.println("Confirm")));
//    new Trigger(m_buttonboard1::cancel).onTrue(new InstantCommand(() -> System.out.println("Cancel")));
//
//  }
=======

    if (grid != 0) {
        new InstantCommand(() -> new placeGamePiece(m_Level, m_gamePiece));
      grid = 0;
    } else {
        new InstantCommand(() -> System.out.println("No Grid Running"));
    }

    new Trigger(m_buttonboard1::cancel).onTrue(new InstantCommand(() -> { grid = 0;}));

    /*
    new Trigger(m_buttonboard2::grid1).onTrue(new InstantCommand(() -> System.out.println("Grid1 Selected")));
    new Trigger(m_buttonboard2::grid2).onTrue(new InstantCommand(() -> System.out.println("Grid2 Selected")));
    new Trigger(m_buttonboard2::grid3).onTrue(new InstantCommand(() -> System.out.println("Grid3 Selected")));
    new Trigger(m_buttonboard2::grid4).onTrue(new InstantCommand(() -> System.out.println("Grid4 Selected")));
    new Trigger(m_buttonboard2::grid5).onTrue(new InstantCommand(() -> System.out.println("Grid5 Selected")));
    new Trigger(m_buttonboard2::grid6).onTrue(new InstantCommand(() -> System.out.println("Grid6 Selected")));
    new Trigger(m_buttonboard2::grid7).onTrue(new InstantCommand(() -> System.out.println("Grid7 Selected")));
    new Trigger(m_buttonboard2::grid8).onTrue(new InstantCommand(() -> System.out.println("Grid8 Selected")));
    new Trigger(m_buttonboard2::grid9).onTrue(new InstantCommand(() -> System.out.println("Grid9 Selected")));
    new Trigger(m_buttonboard1::confirm).onTrue(new InstantCommand(() -> System.out.println("Confirm")));
    new Trigger(m_buttonboard1::cancel).onTrue(new InstantCommand(() -> System.out.println("Cancel")));
     */
>>>>>>> Stashed changes
  }
}
