package frc.robot.subsystems;

public interface IEnabledDisabled {
    default void onEnable() {};
    default void onDisable() {};
}
