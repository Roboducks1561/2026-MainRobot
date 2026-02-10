package frc.robot.subsystems.defaultSystems.position;

public interface PositionIO {
    /**
     * this is a constant call, so once you call it, it will keep going toward that position
     */
    public void setPosition(double position);

    public void stop();

    /**
     * of mechanism rotations, not encoder or motor
     * @return
     */
    public double getPosition();

    public double getTarget();

    public void setVoltage(double volts);

    public void setZero();
}