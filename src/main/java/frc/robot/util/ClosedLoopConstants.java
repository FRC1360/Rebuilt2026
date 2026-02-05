package frc.robot.util;

public class ClosedLoopConstants {
    public double kP;
    public double kI;
    public double kD;
    public double maxVelocity;
    public double maxAcceleration;
    public double kS;
    public double kV;
    public double kA;
    public double kG;

    public ClosedLoopConstants(double kP, double kI, double kD, double maxVelocity, double maxAcceleration, double kS, double kV, double kA, double kG) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
        this.kG = kG;
    }
}