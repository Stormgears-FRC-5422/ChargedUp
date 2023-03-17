package frc.robot.commands.autoScoring;

import edu.wpi.first.wpilibj2.command.CommandBase;
public class PlaceGamePiece extends CommandBase {
  public enum Level {
    LOW,
    MEDIUM,
    HIGH
  }

  public enum gamePiece {
    CUBE,
    CONE
  }

  Level level;
  gamePiece gamePiece;

  int grid;

  public PlaceGamePiece(Level l, gamePiece p, int grid) {
    this.level = l;
    this.gamePiece = p;
    this.grid = grid;
    System.out.println(String.format("In Constructor Grid: %d", grid));

  }

  @Override
  public void initialize() {
    System.out.println("Initializing placeGamePiece command");
    pickUpGamePiece();
  }

  private void pickUpGamePiece() {
    System.out.println(String.format("In pickUpGamePiece Grid: %d", grid));
    if (grid != 0) {
      System.out.println("Running PLace Game Piece");
      if (gamePiece.equals(gamePiece.CUBE)) {
        cubePickUp();
      } else {
        conePickUp();
      }
    } else {
      System.out.println("NOT Running Command, Grid not set");
    }

  }

  private void cubePickUp() {
    switch (level) {
      case LOW:
        System.out.println(String.format("Placing Cube on Low Level , Grid: %d , Row: %d", grid, 2));
        break;
      case MEDIUM:
        System.out.println(String.format("Placing Cube on Medium Level , Grid: %d , Row: %d", grid, 1));
        break;
      case HIGH:
        System.out.println(String.format("Placing Cube on High Level , Grid: %d , Row: %d", grid, 0));
        break;
    }

  }


  private void conePickUp() {
    switch (level) {
      case LOW:
        System.out.println(String.format("Placing Cone on Low Level , Grid: %d , Row: %d", grid, 2));
        break;
      case MEDIUM:
        System.out.println(String.format("Placing Cone on Medium Level , Grid: %d , Row: %d", grid, 1));
        break;
      case HIGH:
        System.out.println(String.format("Placing Cone on High Level , Grid: %d , Row: %d", grid, 0));
        break;
    }


  }
}
