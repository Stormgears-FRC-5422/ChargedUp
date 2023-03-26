package frc.robot.commands.auto.autoManeuvers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.RobotState;
import frc.robot.constants.ShuffleboardConstants;

import java.util.Map;

import static frc.robot.constants.FieldConstants.Grids.*;

public class NodeSelector {

    private int logCounter = 0;
    private int selectedRow = 0;



    private int selectedCol = 0;

    public NodeSelector() {
        ScoringNode[][] currentGrid = getGrid();
        boolean[] CUBE_COLUMNS = new boolean[]{false, true, false, false, true, false, false, true, false};
        for (int col = 0; col < 9; col++) {
            // make the gui be opposite to array in field constants
            // bc drivers will see the opposite
            int colTransformed = currentGrid.length - 1 - col;
            Color color = CUBE_COLUMNS[colTransformed]? Color.kPurple : Color.kYellow;
            for (int row = 0; row < 3; row++) {
                // again the top most row in the gui will be the hybrid not top
                int rowTransformed = currentGrid[colTransformed].length - 1 - row;
                ShuffleboardConstants.getInstance().gridLayout
                        .addBoolean("Column " + col + " Row " + row,
                                () -> colTransformed == selectedCol && rowTransformed == selectedRow)
                        .withProperties(Map.of("color when true", Color.kLightGreen.toHexString(),
                                "color when false", (rowTransformed == 2)?
                                        Color.kLightGray.toHexString() : color.toHexString()))
                        .withPosition(col, row);
            }
        }
    }

    public ScoringNode getSelectedNode() {
//        System.out.println("Selected Col: " + selectedCol + " Selected Row: " + selectedRow);
//        System.out.println(getGrid()[selectedCol][selectedRow]);
        int col = selectedCol;
        // red team will have to mirror their nodes again
        if (RobotState.getInstance().getCurrentAlliance() == DriverStation.Alliance.Red)
            col = 8 - selectedCol;
        return getGrid()[col][selectedRow];
    }

    public void setSelectedGridRow(int col, int row) {
        setSelectedCol(col);
        setSelectedRow(row);
    }

    public void setSelectedCol(int col) {
        selectedCol = MathUtil.clamp(col, 0, 8);
    }

    public void setSelectedRow(int row) {
        selectedRow = MathUtil.clamp(row, 0, 2);
    }

    public void moveSelectedCol(int moveDist) {
        setSelectedCol(selectedCol - moveDist);
    }

    public void moveSelectedRow(int moveDist) {
        setSelectedRow(selectedRow - moveDist);
    }

    public int getSelectedCol() {
        return selectedCol;
    }
}
