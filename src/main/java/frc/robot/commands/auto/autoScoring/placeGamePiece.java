package frc.robot.commands.autoScoring;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class placeGamePiece extends CommandBase{
    public enum Level{
        LOW,
        MEDIUM,
        HIGH
    }

    public enum gamePiece{
        CUBE,
        CONE
    }

    Level m_Level;
    gamePiece m_gamePiece;

    public placeGamePiece(Level l, gamePiece p){
        m_Level = l;
        m_gamePiece = p;
    }

    @Override
    public void initialize(){
        switch(m_gamePiece){
            case CUBE:
                switch(m_Level){
                    case LOW:
                        System.out.println("Placing Cube on Low Level");
                        break;
                    case MEDIUM:
                        System.out.println("Placing Cube on Medium Level");
                        break;
                    case HIGH:
                        System.out.println("Placing Cube on High Level");
                }
            case CONE:
                switch(m_Level){
                    case LOW:
                        System.out.println("Placing Cone on Low Level");
                        break;
                    case MEDIUM:
                        System.out.println("Placing Cone on Medium Level");
                        break;
                    case HIGH:
                        System.out.println("Placing Cone on High Level");
                }
        }
    }
}
