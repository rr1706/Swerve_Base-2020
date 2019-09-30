package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.*;
import  com.ctre.phoenix.*;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.Pair;

class SwerveModule {
    private VictorSP translationMotor;
    private TalonSRX rotationMotor;
    private Encoder encoder;

    private double ticksPerRevolution;
    private double offset;
    private double speedCommand;
    private double angleCommand;
    private double distance;
    private double angleError;
    private boolean wheelReversed;
    private Pair <Double, Double> position;

    private double defence = 0.0;

    SwerveModule(int translationPort, int rotationPort, int encoderPort1, int encoderPort2, double potentiometerOffset, String position, int tpr) {
        super();

        translationMotor = new VictorSP(translationPort);
        rotationMotor = new TalonSRX(rotationPort);

        rotationMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 0);
        rotationMotor.config_kF(0, 0.0, 0);
        rotationMotor.config_kP(0, 5.0, 0);
        rotationMotor.config_kI(0, 0.0, 0);
        rotationMotor.config_kD(0, 0.0, 0);
        rotationMotor. configAllowableClosedloopError (0, 2, 0);
        rotationMotor.setSensorPhase(true);

        encoder = new Encoder(encoderPort1, encoderPort2, false, Encoder.EncodingType.k4X);
        encoder.setDistancePerPulse(1.0);

        offset = potentiometerOffset;

        String[] pos = position.split(",");
        this.position = new Pair<>(Double.valueOf(pos[0]), Double.valueOf(pos[1]));
        ticksPerRevolution = tpr;
    }

    void drive() {
        double rotationCount;
        double unwrappedAngleCommand;
        double unwrappedAngleError;
        double finalAngleCommand;

        distance = encoder.getDistance();
        angleError = rotationMotor.getClosedLoopError(0);

        // 359 --> 361 instead of 359 --> 1, so module does not reverse rotation when crossing 0 degrees
        rotationCount = Math.floor((rotationMotor.getSensorCollection().getAnalogIn()) / ticksPerRevolution);
        unwrappedAngleCommand = angleCommand + rotationCount * ticksPerRevolution;
        unwrappedAngleError = unwrappedAngleCommand - rotationMotor.getSensorCollection().getAnalogIn();

        if (Math.abs(unwrappedAngleError) > ticksPerRevolution * 0.5) {
            finalAngleCommand = -(unwrappedAngleCommand - Math.signum(unwrappedAngleError) * ticksPerRevolution);
        } else {
            finalAngleCommand = -unwrappedAngleCommand;
        }

        // If wheel direction has to move over 45 degrees, go 180 off the command and reverse translation
        if (Math.abs(-finalAngleCommand - rotationMotor.getSensorCollection().getAnalogIn()) > ticksPerRevolution * 0.25) {
            finalAngleCommand += Math.signum(-finalAngleCommand - rotationMotor.getSensorCollection().getAnalogIn()) * ticksPerRevolution * 0.5;
            wheelReversed = true;
            speedCommand *= -1.0;
        } else {
            wheelReversed = false;
        }

        // Translate unless the wheel has to move over 22.5 degrees
        if (Math.abs(angleError) < ticksPerRevolution * 0.125) {
            translationMotor.set(speedCommand);
        } else {
            translationMotor.set(0.0);
        }

        // Turn only when there is a translation or lock command
        if (Math.abs(speedCommand) > 0.1 || defence != 0.0) {
            rotationMotor.set(ControlMode.Position, finalAngleCommand);
        } else {
            rotationMotor.set(ControlMode.Current, 0.0);
        }
    }

    void setDefenceMode(double angle) {
        defence = angle;
    }

    void setSpeedCommand(double speedCommand) {
        this.speedCommand = speedCommand;
    }

    void setAngleCommand(double angleCommand) {
        this.angleCommand = angleCommand + defence;
    }

    Pair<Double, Double> getPosition() {
        return position;
    }

    public void setPosition(Pair<Double, Double> position) {
        this.position = position;
    }

    double getSpeedCommand() {
        return this.speedCommand;
    }

    public double getAngle() {
        return MathUtils.convertRange(0.0, ticksPerRevolution, 0.0, 360.0, rotationMotor.getSensorCollection().getAnalogIn());
    }

    public double getDistance() {
        return distance;
    }

    double getOffset() {
        return offset;
    }
}