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
    private CANEncoder translationEncoder;

    private CANSparkMax rotationMotor;
    private Potentiometer potentiometer;
    private PIDController rotationPID;
    private double rotationP = 6.7e-3;

    private double offset;
    private double speedCommand;
    private double angleCommand;
    private double distance;
    private double angleError;
    private double actualAngle;
    private boolean wheelReversed = false;
    private Pair <Double, Double> position;
    private double prevAngleCommand = 0.0;

    private double testRotationMotor = 0.0;
    private double testTranslationMotor = 0.0;

    private double defence = 0.0;

    SwerveModule(int translationPort, int rotationPort, int potentiometerPort, double potentiometerOffset, String position) {
        super();

        translationMotor = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
        translationMotor.setInverted(false);
        translationMotor.setSmartCurrentLimit(40, 20);
        translationMotor.setPeriodicFramePeriod(CANSparkMaxLowLevel.PeriodicFrame.kStatus2, 5);
        translationMotor.setCANTimeout(CAN_TIMEOUT);

        translationMotor.burnFlash();


        rotationMotor = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);
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

        String[] pos = position.split(",");
        this.position = new Pair<>(Double.valueOf(pos[0]), Double.valueOf(pos[1]));
    }

    void drive() {
//        double rotationCount;
//        double unwrappedAngleCommand;
//        double unwrappedAngleError;
//        double finalAngleCommand;


        actualAngle = SmartDashboard.getNumber("Test Angle", 0.0);

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


        System.out.println(wheelReversed  + "  |      |  " + angleCommand + "  |      |  " + actualAngle + "  |      |  " + rotationPID.performPID() + "  |      |  " + rotationPID.getError());


        translationMotor.set(speedCommand);


        //This should be negative, otherwise it gets angry
        rotationMotor.set(-rotationPID.performPID());

        prevAngleCommand = angleCommand;

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
        return actualAngle;
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
//        System.out.println(speedCommand + "  |      |  " + angleCommand + "  |      |  " + potentiometer.get() + "  |      |  " + rotationPID.performPID());
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