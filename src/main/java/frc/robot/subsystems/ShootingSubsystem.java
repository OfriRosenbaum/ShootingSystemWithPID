package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.spikes2212.command.genericsubsystem.GenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystem;
import com.spikes2212.command.genericsubsystem.commands.MoveGenericSubsystemWithPID;
import com.spikes2212.control.PIDSettings;
import com.spikes2212.control.noise.ExponentialFilter;
import com.spikes2212.control.noise.NoiseReducer;
import com.spikes2212.dashboard.*;
import com.spikes2212.util.TalonEncoder;
import frc.robot.RobotMap;

import java.util.function.Supplier;

public class ShootingSubsystem extends GenericSubsystem {

    private static ShootingSubsystem instance = new ShootingSubsystem();
    public static final double distancePerPulse = 10 / 4096.0;
    private static RootNamespace shooterNamespace = new RootNamespace("shooter");
    private static Namespace PID = shooterNamespace.addChild("PID");
    private static Supplier<Double> maxSpeed = shooterNamespace.addConstantDouble("Max Speed", 0.6);
    private static Supplier<Double> minSpeed = shooterNamespace.addConstantDouble("Min Speed", 0);
    public static Supplier<Double> shootSpeed = shooterNamespace.addConstantDouble("Speed", 0.4);

    private static Supplier<Double> kP = PID.addConstantDouble("kP", 0);
    private static Supplier<Double> kI = PID.addConstantDouble("kI", 0);
    private static Supplier<Double> kD = PID.addConstantDouble("kD", 0);
    private static Supplier<Double> tolerance = PID.addConstantDouble("Tolerance", 0);
    private static Supplier<Double> waitTime = PID.addConstantDouble("Wait Time", 0);

    private static Supplier<Double> targetSpeed = PID.addConstantDouble("Target Speed", 0);

    public static PIDSettings velocityPIDSettings = new PIDSettings(kP, kI, kD, tolerance, waitTime);

    private WPI_TalonSRX master;
    private WPI_VictorSPX slave;
    private NoiseReducer noiseReducer;
    private TalonEncoder encoder;

    private ShootingSubsystem() {
        super(minSpeed, maxSpeed);
        master = new WPI_TalonSRX(RobotMap.CAN.SHOOTER_MASTER);
        slave = new WPI_VictorSPX(RobotMap.CAN.SHOOTER_SLAVE);
//        master.setNeutralMode(NeutralMode.Brake);
        encoder = new TalonEncoder(master);
//        master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        master.setInverted(true);
//        slave.setNeutralMode(NeutralMode.Brake);
        slave.follow(master);
        noiseReducer = new NoiseReducer(() -> encoder.getVelocity() * distancePerPulse,
                new ExponentialFilter(0.05));
    }

    public static ShootingSubsystem getInstance() {
        return instance;
    }

    @Override
    public void apply(double speed) {
        master.set(speed);
    }

    @Override
    public boolean canMove(double speed) {
        return true;
    }

    @Override
    public void stop() {
        master.stopMotor();
    }

    @Override
    public void periodic() {
        shooterNamespace.update();
    }

    public double getMotorSpeed() {
        return noiseReducer.get();
    }

    @Override
    public void configureDashboard() {
        shooterNamespace.putNumber("shooter velocity - filtered", noiseReducer);
        shooterNamespace.putNumber("shooter velocity", () -> encoder.getVelocity() * distancePerPulse);
        shooterNamespace.putData("shoot", new MoveGenericSubsystem(this, shootSpeed));
        shooterNamespace.putData("pid shoot",
                new MoveGenericSubsystemWithPID(this, targetSpeed, this::getMotorSpeed, velocityPIDSettings));
    }

    public void setAccelerated(boolean isAccelerated) {
        shooterNamespace.putBoolean("is accelerated", isAccelerated);
    }
}
