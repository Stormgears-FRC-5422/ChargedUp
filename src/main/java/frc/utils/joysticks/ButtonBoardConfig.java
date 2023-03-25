package frc.utils.joysticks;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.auto.autoManeuvers.DriveToDoubleSubstation;
import frc.robot.commands.auto.autoManeuvers.NodeSelector;
import frc.robot.constants.Constants;
import frc.robot.subsystems.Compression;
import frc.robot.subsystems.NeoPixel;
import frc.robot.subsystems.arm.Arm;

import javax.swing.text.Position;

import java.util.function.Supplier;

import static frc.robot.constants.Constants.kXYArmManualSpeed;

public class ButtonBoardConfig {

  ButtonBoard m_buttonboard1;
  ButtonBoard m_buttonboard2;
  NeoPixel neoPixel;
  Arm arm;
  public NodeSelector nodeSelector;

  Compression compression;

  public CubeCone m_cubeCone = CubeCone.CONE;

  public DriveToDoubleSubstation.Position m_Position = DriveToDoubleSubstation.Position.NONE;

  public boolean confirmCondition;

  public ButtonBoardConfig(NeoPixel neoPixel, NodeSelector nodeSelector, Compression compression, Arm arm) {

    m_buttonboard1 = new ButtonBoard(1);
    m_buttonboard2 = new ButtonBoard(2);
    this.neoPixel = neoPixel;
    this.nodeSelector = nodeSelector;
    this.compression = compression;
  }

  public void buttonBoardSetup(){
    int[] segments1 = {1, 2};
    int[] allRingSegments = {1,2,3};
    int[] segmentsoff = {3, 4};
    int[] segmentsoff2 = {1, 2, 4};
    int[] segmentsoff3 = {1, 2, 3};


    System.out.println("buttonBoardSetup starting");

    if (!m_buttonboard2.jumper()) {
      System.out.println("Switching ButtonBoard ports");
      m_buttonboard1 = new ButtonBoard(2);
      m_buttonboard2 = new ButtonBoard(1);
    } else {
      System.out.println("Not Switching ButtonBoard ports");
    }

    new Trigger(m_buttonboard1::kill).onTrue(new InstantCommand(() -> System.exit(0)));

//    new Trigger(() -> m_buttonboard1.getRawButton(11) && m_buttonboard1.getRawButton(7))
//            .onTrue(new InstantCommand(() -> {neoPixel.setSpecificSegmentColor(segments1, NeoPixel.PURPLE_COLOR); neoPixel.setSpecificSegmentColor();}));
//    new Trigger(() -> m_buttonboard1.getRawButton(11) && !m_buttonboard1.getRawButton(7))
//            .onTrue(new InstantCommand(() -> neoPixel.setSpecificSegmentColor(segments1, NeoPixel.YELLOW_COLOR)));
//
//    new Trigger(() -> m_buttonboard1.getRawButton(12) && m_buttonboard1.getRawButton(7))
//            .onTrue(new InstantCommand(() -> neoPixel.setColor(3, NeoPixel.PURPLE_COLOR)));
//    new Trigger(() -> m_buttonboard1.getRawButton(12) && !m_buttonboard1.getRawButton(7))
//            .onTrue(new InstantCommand(() -> neoPixel.setColor(3, NeoPixel.YELLOW_COLOR)));
//
//    new Trigger(() -> m_buttonboard1.getRawButton(10) && m_buttonboard1.getRawButton(7))
//            .onTrue(new InstantCommand(() -> neoPixel.setColor(4, NeoPixel.PURPLE_COLOR)));
//    new Trigger(() -> m_buttonboard1.getRawButton(10) && !m_buttonboard1.getRawButton(7))
//            .onTrue(new InstantCommand(() -> neoPixel.setColor(4, NeoPixel.YELLOW_COLOR)));

//    CODE BELOW CAN BE USED WHEN WE FLOOR,LEFT, and RIGHT SUBS WORKS

//    new Trigger(() -> m_buttonboard1.getRawButton(12) && m_buttonboard1.getRawButton(7))
//            .onTrue(new InstantCommand(() -> {neoPixel.setSpecificSegmentColor(allRingSegments, NeoPixel.PURPLE_COLOR);
//              neoPixel.setSpecificSegmentColor(segmentsoff, NeoPixel.NO_COLOR);}));
//    new Trigger(() -> m_buttonboard1.getRawButton(12) && !m_buttonboard1.getRawButton(7))
//            .onTrue(new InstantCommand(() -> {neoPixel.setSpecificSegmentColor(allRingSegments, NeoPixel.YELLOW_COLOR);
//              neoPixel.setSpecificSegmentColor(segmentsoff, NeoPixel.NO_COLOR);}));
//
//    new Trigger(() -> m_buttonboard1.getRawButton(11) && m_buttonboard1.getRawButton(7))
//            .onTrue(new InstantCommand(() -> {neoPixel.setSpecificSegmentColor(allRingSegments, NeoPixel.PURPLE_COLOR);
//              neoPixel.setSpecificSegmentColor(segmentsoff2, NeoPixel.NO_COLOR);}));
//    new Trigger(() -> m_buttonboard1.getRawButton(11) && !m_buttonboard1.getRawButton(7))
//            .onTrue(new InstantCommand(() -> {neoPixel.setSpecificSegmentColor(allRingSegments, NeoPixel.YELLOW_COLOR);
//              neoPixel.setSpecificSegmentColor(segmentsoff2, NeoPixel.NO_COLOR);}));
//
//    new Trigger(() -> m_buttonboard1.getRawButton(10) && m_buttonboard1.getRawButton(7))
//            .onTrue(new InstantCommand(() -> {neoPixel.setSpecificSegmentColor(allRingSegments, NeoPixel.PURPLE_COLOR);
//              neoPixel.setSpecificSegmentColor(segmentsoff3, NeoPixel.NO_COLOR);}));
//    new Trigger(() -> m_buttonboard1.getRawButton(10) && !m_buttonboard1.getRawButton(7))
//            .onTrue(new InstantCommand(() -> {neoPixel.setSpecificSegmentColor(allRingSegments, NeoPixel.YELLOW_COLOR);
//              neoPixel.setSpecificSegmentColor(segmentsoff3, NeoPixel.NO_COLOR);}));

    if (Constants.Toggles.useStatusLights) {
      new Trigger(m_buttonboard1::cubeCone).whileTrue(new InstantCommand(() -> {
        neoPixel.setSpecificSegmentColor(allRingSegments, NeoPixel.PURPLE_COLOR);
        m_cubeCone = CubeCone.CUBE;
      }));
      new Trigger(m_buttonboard1::cubeCone).whileFalse(new InstantCommand(() -> {
        neoPixel.setSpecificSegmentColor(allRingSegments, NeoPixel.YELLOW_COLOR);
        m_cubeCone = CubeCone.CONE;
      }));
    }
//    new Trigger(m_buttonboard1::floor).onTrue(new InstantCommand(() -> m_Position = Position.FLOOR));

    new Trigger(m_buttonboard1::leftSub).onTrue(new InstantCommand(() -> {
      m_Position = DriveToDoubleSubstation.Position.LEFT;
    }));
    new Trigger(m_buttonboard1::rightSub).onTrue(new InstantCommand(() -> {
      m_Position = DriveToDoubleSubstation.Position.RIGHT;
    }));




    if (Constants.Toggles.usePneumatics) {
      new Trigger(m_buttonboard2::gripper).whileTrue(new InstantCommand(compression::grabCubeOrCone));
      new Trigger(m_buttonboard2::gripper).whileFalse(new InstantCommand(compression::release));
    }

    // set row of node selector
    new Trigger(this::topGrid)
            .onTrue(new InstantCommand(() -> nodeSelector.setSelectedRow(2)));
    new Trigger(this::middleGrid )
            .onTrue(new InstantCommand(() -> nodeSelector.setSelectedRow(1)));
    new Trigger(this::bottomGrid)
            .onTrue(new InstantCommand(() -> nodeSelector.setSelectedRow(0)));


    int offset = 4;
    for (int i = 8; i >= 0; i--) {
      int temp = i;
      new Trigger(() -> m_buttonboard2.getRawButton(temp + offset))
              .onTrue(new InstantCommand(() -> {
                nodeSelector.setSelectedCol(temp);
                m_Position = DriveToDoubleSubstation.Position.NONE;
              }));
    }
  }


  public boolean topGrid() {
    return m_buttonboard1.getRawButton(5) && !m_buttonboard1.getRawButton(6);
  }

  public boolean middleGrid() {
    return !m_buttonboard1.getRawButton(5) && !m_buttonboard1.getRawButton(6);
  }

  public boolean bottomGrid() {
    return m_buttonboard1.getRawButton(6) && !m_buttonboard1.getRawButton(5);
  }
  public boolean confirm() {
    return m_buttonboard1.confirm();
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

  public boolean pickLeftSub() {
    return m_buttonboard1.leftSub();
  }

  public boolean pickRightSub() {
    return m_buttonboard1.rightSub();
  }

  public double armUpDown() {
    double y = m_buttonboard2.getY();
    double dy = 0;
    // There is a slight voltage bias that causes the joystick to report != 0 at rest
    // any actual motion sets it to -1.0 or 1.0 so we just need some reasonable number in the middle here
    if (Math.abs(y) > 0.5 ) {
      if (y < 0)
        dy = kXYArmManualSpeed;
      else
        dy = -kXYArmManualSpeed;
    }

//    if (dy != 0) System.out.println("armUpDown: dy = " + dy);
    return dy;
  }

  public double armInOut() {
    double x = m_buttonboard1.getX();
    double dx = 0;
    // There is a slight voltage bias that causes the joystick to report != 0 at rest
    // any actual motion sets it to -1.0 or 1.0 so we just need some reasonable number in the middle here
    if (Math.abs(x) > 0.5 ) {
      if (x < 0)
        dx = kXYArmManualSpeed;
      else
        dx = -kXYArmManualSpeed;
    }

//    if (dx != 0) System.out.println("armInOut: dx = " + dx);
    return dx;
  }

//  pub
}
