package org.firstinspires.ftc.teamcode.modules.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.modules.statemachines.IntakeSM;
import org.firstinspires.ftc.teamcode.modules.util.AxonEncoder;

@Config
public class Intake implements Subsystem {

    public DcMotorImplEx intakeMotor;
    public ServoImplEx flipLeft;
    public AxonEncoder flipLeftEncoder;
    public ServoImplEx flipRight;
    public AxonEncoder flipRightEncoder;
    public ServoImplEx lockLower;
    public ServoImplEx lockHigher;

    public IntakeSM statemachine;

    public Boolean PIXELS_LOCKED = false;

    public Intake(HardwareMap hardwareMap) {

        intakeMotor = hardwareMap.get(DcMotorImplEx.class, "IntakeMotor");

        flipLeft = hardwareMap.get(ServoImplEx.class, "flipLeft");
        flipRight = hardwareMap.get(ServoImplEx.class, "flipRight");
        flipLeft.setDirection(Servo.Direction.REVERSE);

        //flipLeftEncoder = new AxonEncoder(hardwareMap.get(AnalogInput.class, "flipLeftEncoder"));
        //flipRightEncoder = new AxonEncoder(hardwareMap.get(AnalogInput.class, "flipRightEncoder"));

        lockLower = hardwareMap.get(ServoImplEx.class, "lockLower");
        lockHigher = hardwareMap.get(ServoImplEx.class, "lockHigher");

        statemachine = new IntakeSM(this);
    }

    /**
     * Sets the power to the rolling intake manually.
     * @param power
     */
    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    static double lockLowerClearPos = 0;
    static double lockLowerLockPos = 0.55;
    static double lockHigherClearPos = 0.3;
    static double lockHigherLockPos = 0;

    /**
     * Engages the servos to lock pixels in the deposit.
     * @param lower Lock the lower section
     * @param higher Lock the higher section
     */
    public void engageLock(boolean lower, boolean higher) {
        if (lower) {
            lockLower.setPosition(lockLowerLockPos);
        }
        if (higher) {
            lockHigher.setPosition(lockHigherLockPos);
        }

        if (
                lockLower.getPosition() == lockLowerLockPos &&
                lockHigher.getPosition() == lockHigherLockPos
        ) {
            PIXELS_LOCKED = true;
        }
    }

    public void clearLowerLock() {
        lockLower.setPosition(lockLowerClearPos);
        PIXELS_LOCKED = false;
    }

    public void clearHigherLock() {
        lockHigher.setPosition(lockHigherClearPos);
        PIXELS_LOCKED = false;
    }

    static double leftFlipDepositPos = 1.00;
    static double rightFlipDepositPos = 0.95;

    /**
     * Sets the deposit box to deposit position.
     */
    public void flipDeposit() {
        setFlipPosition(leftFlipDepositPos, rightFlipDepositPos);
    }

    static double leftFlipIntakePos = 0.08;
    static double rightFlipIntakePos = 0.03;

    /**
     * Sets the deposit box to intake position.
     */
    public void flipIntake() {
        setFlipPosition(leftFlipIntakePos, rightFlipIntakePos);
    }

    public void incrementFlip(double step) {
        setFlipPosition(
                flipLeft.getPosition() + step,
                flipRight.getPosition() + step
        );
    }

    public void setFlipPosition(double leftPos, double rightPos) {
        flipLeft.setPosition(leftPos);
        flipRight.setPosition(rightPos);
    }

    public void flipSafe() {
        flipLeft.setPosition(0.12);
        flipRight.setPosition(0.10);
    }


    /**
     * Checks if the deposit box in at or near the deposit position
     * @return True if in position, false if not
     */
    public boolean inDepositPosition() {
        double tolerance = 1.5;
        double rightFlipDepositEncoderPos = 0;
        double leftFlipDepositEncoderPos = 0;
        // If both encoder readings are in tolerance, return true
        if (
                Math.abs(rightFlipDepositEncoderPos - flipRightEncoder.getReadingDegrees()) < tolerance &&
                Math.abs(leftFlipDepositEncoderPos - flipLeftEncoder.getReadingDegrees()) < tolerance
        ) {
            return true;
        } else return false;
    }

    /**
     * Checks if the deposit box in at or near the intake position
     * @return True if in position, false if not
     */
    public boolean inIntakePosition() {
        double tolerance = 1.5;

        // If both encoder readings are in tolerance, return true
        double rightFlipIntakeEncoderPos = 0;
        double leftFlipIntakeEncoderPos = 0;
        if (
                Math.abs(rightFlipIntakeEncoderPos - flipRightEncoder.getReadingDegrees()) < tolerance &&
                        Math.abs(leftFlipIntakeEncoderPos - flipLeftEncoder.getReadingDegrees()) < tolerance
        ) {
            return true;
        } else return false;
    }

    @Override
    public void update() {
        statemachine.update();
    }
}
