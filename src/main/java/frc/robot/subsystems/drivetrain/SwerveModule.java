package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.PIDController;
import frc.robot.utilities.Pair;

class SwerveModule {
    private static final int CAN_TIMEOUT = 20;

    private CANSparkMax translationMotor;

    private CANSparkMax rotationMotor;
    private Potentiometer potentiometer;
    private PIDController rotationPID;

    private double offset;
    private double rotationPower;
    private double speedCommand;
    private double angleCommand ;
    private double distance;
    private double actualAngle;
    private boolean wheelReversed = false;
    private Pair <Double, Double> position;
    private double prevAngleCommand = 0.0;

    private double testRotationMotor = 0.0;
    private double testTranslationMotor = 0.0;

    private double defence = 0.0;

    SwerveModule(int translationPort, int rotationPort, int potentiometerPort, double potentiometerOffset, Pair<Double, Double> position) {
        super();
        double rotationP = 6.7e-3;

        translationMotor = new CANSparkMax(translationPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        translationMotor.setInverted(false);
        translationMotor.setSmartCurrentLimit(40, 20);
        translationMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 5);
        translationMotor.setCANTimeout(CAN_TIMEOUT);

        translationMotor.burnFlash();


        rotationMotor = new CANSparkMax(rotationPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        rotationMotor.setInverted(false);
        rotationMotor.setSmartCurrentLimit(40, 20);
        rotationMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 5);
        translationMotor.setCANTimeout(CAN_TIMEOUT);

        rotationMotor.burnFlash();

        rotationPID = new PIDController(rotationP, 0.0, 0.0);
        rotationPID.setInputRange(0.0, 360.0);
        rotationPID.setContinuous(true);
        rotationPID.setOutputRange(-1.0, 1.0);

        rotationPID.enable();

        potentiometer = new AnalogPotentiometer(potentiometerPort, (360.0), 0.0);

        offset = potentiometerOffset;
        setPosition(position);
    }

    void drive() {
        actualAngle = potentiometer.get();

        //If translation isn't being commanded, the wheel angle shouldn't reset to zero
        if (Math.abs(speedCommand ) < 0.04) {
            angleCommand = prevAngleCommand;
        }


        // If wheel direction has to move over 45 degrees, go 180 off the command and reverse translation
        if (Math.abs(angleCommand - actualAngle) > 90.0) {
            angleCommand = MathUtils.resolveAngle(angleCommand + 180.0);
            wheelReversed = true;
            speedCommand *= -1.0;
        } else {
            wheelReversed = false;
        }

        rotationPID.setInput(actualAngle);

        rotationPID.setSetpoint(angleCommand);

        translationMotor.set(speedCommand);


        //This should be negative, otherwise it gets angry
        rotationPower = -rotationPID.performPID();
        rotationMotor.set(rotationPower);

        prevAngleCommand = angleCommand;

        // Translate unless the wheel has to move over 22.5 degrees
//        if (Math.abs(angleError) < ticksPerRevolution * 0.125) {
//            translationMotor.set(speedCommand);
//        } else {
//            translationMotor.set(0.0);
//        }

        // Turn only when there is a translation or lock command
//        if (Math.abs(speedCommand) > 0.1 || defence != 0.0) {
//            rotationMotor.set(ControlMode.Position, finalAngleCommand);
//        } else {
//            rotationMotor.set(ControlMode.Current, 0.0);
//        }
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
        return MathUtils.resolveAngle(potentiometer.get() - offset);
    }

    public double getAngleCommand() {
        return angleCommand;
    }

    public double getDistance() {
        return distance;
    }

    double getOffset() {
        return offset;
    }

    double getRotationPower()
    {
        return rotationPower;
    }

    //To be used for diagnostics only. Disable any other settings of the chosen motor variables
    void setTestRotationMotor(double rightJoystick)
    {
        this.testRotationMotor = rightJoystick;
    }

    void setTestTranslationMotor(double leftJoystick)
    {
        this.testTranslationMotor = leftJoystick;
    }
}