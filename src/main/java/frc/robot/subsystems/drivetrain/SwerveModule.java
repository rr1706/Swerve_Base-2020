package frc.robot.subsystems.drivetrain;

import edu.wpi.first.wpilibj.AnalogPotentiometer;
import com.revrobotics.*;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utilities.MathUtils;
import frc.robot.utilities.Pair;

class SwerveModule {
//    private static final int CAN_TIMEOUT = 20;

    private CANSparkMax translationMotor;
    private CANEncoder translationEncoder;

    private CANSparkMax rotationMotor;
    private Potentiometer potentiometer;

    private double ticksPerRevolution;
    private double offset;
    private double speedCommand;
    private double angleCommand;
    private double distance;
    private double angleError;
    private double actualAngle;
    private boolean wheelReversed;
    private Pair <Double, Double> position;

    private double testRotationMotor = 0.0;
    private double testTranslationMotor = 0.0;

    private double defence = 0.0;

    SwerveModule(int translationPort, int rotationPort, int potentiometerPort, double potentiometerOffset, String position, int tpr) {
        super();

        translationMotor = new CANSparkMax(translationPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        translationMotor.setInverted(false);
        translationMotor.setSmartCurrentLimit(40, 20);
        translationMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 5);
        translationMotor.burnFlash();


        rotationMotor = new CANSparkMax(rotationPort, CANSparkMaxLowLevel.MotorType.kBrushless);
        rotationMotor.setInverted(false);
        rotationMotor.setSmartCurrentLimit(40, 20);
        rotationMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 5);
        rotationMotor.burnFlash();

        potentiometer = new AnalogPotentiometer(potentiometerPort, (360.0), 0.0);

        offset = potentiometerOffset;

        String[] pos = position.split(",");
        this.position = new Pair<>(Double.valueOf(pos[0]), Double.valueOf(pos[1]));
        ticksPerRevolution = tpr;
    }

    void drive() {
//        double rotationCount;
//        double unwrappedAngleCommand;
//        double unwrappedAngleError;
//        double finalAngleCommand;


        actualAngle = potentiometer.get();
        angleError = MathUtils.calculateContinuousError(angleCommand, actualAngle, 360, 0);

        // 359 --> 361 instead of 359 --> 1, so module does not reverse rotation when crossing 0 degrees
//        rotationCount = Math.floor(rotationMotor.getSensorCollection().getAnalogIn() / ticksPerRevolution);
//        unwrappedAngleCommand = angleCommand + rotationCount * ticksPerRevolution;
//        unwrappedAngleError = unwrappedAngleCommand - rotationMotor.getSensorCollection().getAnalogIn();

//        if (Math.abs(unwrappedAngleError) > ticksPerRevolution * 0.5) {
//            finalAngleCommand = -(unwrappedAngleCommand - Math.signum(unwrappedAngleError) * ticksPerRevolution);
//        } else {
//            finalAngleCommand = -unwrappedAngleCommand;
//        }

        // If wheel direction has to move over 45 degrees, go 180 off the command and reverse translation
//        if (Math.abs(-finalAngleCommand - rotationMotor.getSensorCollection().getAnalogIn()) > ticksPerRevolution * 0.25) {
//            finalAngleCommand += Math.signum(-finalAngleCommand - rotationMotor.getSensorCollection().getAnalogIn()) * ticksPerRevolution * 0.5;
//            wheelReversed = true;
//            speedCommand *= -1.0;
//        } else {
//            wheelReversed = false;
//        }

        // Translate unless the wheel has to move over 22.5 degrees
//        angleError = 0.0;
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
        return 0.0;
//                MathUtils.convertRange(0.0, ticksPerRevolution, 0.0, 360.0, rotationMotor.getSensorCollection().getAnalogIn());
    }

    public double getDistance() {
        return distance;
    }

    double getOffset() {
        return offset;
    }

    void print()
    {
        SmartDashboard.putNumber("Test Angle", potentiometer.get());
//        System.out.println(testTranslationMotor + "  |      |  " + testRotationMotor);
    }

    //To be used for diagnostics only. Disable any other settings of the chosen motor variables
    void setTestRotationMotor(double rightJoystick)
    {
        this.testRotationMotor = rightJoystick;
        rotationMotor.set(rightJoystick);
    }

    void setTestTranslationMotor(double leftJoystick)
    {
        this.testTranslationMotor = leftJoystick;
        translationMotor.set(leftJoystick);
    }
}