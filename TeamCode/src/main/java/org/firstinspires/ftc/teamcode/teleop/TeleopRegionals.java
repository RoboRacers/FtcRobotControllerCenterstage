package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.autonomous.AprilTagDrive;
import org.firstinspires.ftc.teamcode.modules.statemachines.SlidesSM;

@Config
@TeleOp(name = "Telop for Regionals", group = "16481-Centerstage")
public class TeleopRegionals extends LinearOpMode {

    RobotCore robot;

    public static double speedMultiplier = .8;
    public static double strafeMultiplier = .8;
    public static double turnMultiplier = .8;
    public static double retractionSpeed = .5;
    public static double extensionSpeed = 1;
    public static double rtpLockPower = 0.3;
    public static double rtpRunPower = 0.8;
    private boolean presetEnabled = false;

    AprilTagDrive aprilTagDrive;

    @Override
    public void runOpMode() throws InterruptedException {

        robot = new RobotCore(hardwareMap);

        Gamepad previousGamepad1 = gamepad1;
        Gamepad previousGamepad2 = gamepad2;

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad currentGamepad2 = new Gamepad();

        boolean test = false;

        aprilTagDrive = new AprilTagDrive();
        aprilTagDrive.initAprilTag(hardwareMap);


        robot.slides.statemachine.transition(
                SlidesSM.EVENT.ENABLE_MANUAL
        );

        while (opModeInInit()) {
        }

        while (!isStopRequested()) {

            previousGamepad1 = currentGamepad1;
            previousGamepad2 = currentGamepad2;

            currentGamepad1 = gamepad1;
            currentGamepad2 = gamepad2;

            // Drive control
            if (gamepad1.left_bumper) {
                aprilTagDrive.update(true, robot.drive);
            } else {
                robot.drive.setWeightedDrivePower(
                        new Pose2d(
                                gamepad1.left_stick_y * speedMultiplier,
                                gamepad1.left_stick_x * strafeMultiplier,
                                -gamepad1.right_stick_x * turnMultiplier
                        )
                );
            }


            // Slides control
            if (gamepad2.right_stick_y > 0.1 && gamepad2.left_bumper) {
                presetEnabled = false;
                robot.slides.setManualPower(gamepad2.right_stick_y);
            } else if (gamepad2.right_stick_y < -0.1) {
                presetEnabled = false;
                robot.slides.setManualPower(gamepad2.right_stick_y*extensionSpeed);
            } else if (gamepad2.right_stick_y > 0.1) {
                presetEnabled = false;
                robot.slides.setManualPower(gamepad2.right_stick_y*retractionSpeed);
            } else if (!presetEnabled){
                /*
                robot.slides.setTargetPosition(
                        robot.slides.getCurrentPosition()
                );
                robot.slides.statemachine.transition(
                        SlidesSM.EVENT.ENABLE_RTP
                );
                robot.slides.setPower(rtpLockPower);

                 */
                robot.slides.setPower(0);
            }

            // Presets
            if (gamepad2.triangle) {
                presetEnabled = true;
                if (robot.intake.PIXELS_LOCKED) {
                    robot.intake.flipDeposit();
                    robot.slides.setTargetPosition(
                            -1500
                    );
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_RTP
                    );
                }
                robot.slides.setPower(rtpRunPower);
            } else if (gamepad2.square) {
                presetEnabled = true;
                if (robot.intake.PIXELS_LOCKED) {
                    robot.intake.flipDeposit();
                    robot.slides.setTargetPosition(
                            -700
                    );
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_RTP
                    );
                }
                robot.slides.setPower(rtpRunPower);
            } else if (gamepad2.cross) {
                presetEnabled = true;
                robot.intake.flipIntake();
                robot.intake.engageLock(false, true);
                robot.slides.setTargetPosition(
                        0
                );
                robot.slides.statemachine.transition(
                        SlidesSM.EVENT.ENABLE_RTP
                );
                robot.slides.setPower(rtpRunPower);
            } else if (gamepad2.circle) {
                robot.slides.setManualPower(0);
                presetEnabled = true;
                robot.slides.leftmotor.setMode(
                        DcMotor.RunMode.STOP_AND_RESET_ENCODER
                );
                robot.slides.rightmotor.setMode(
                        DcMotor.RunMode.STOP_AND_RESET_ENCODER
                );
            }

            // Intake control
            if (gamepad1.right_trigger > 0.1 && !robot.intake.PIXELS_LOCKED) {
                robot.intake.setIntakePower(gamepad1.right_trigger*.9);
            } else if (gamepad1.left_trigger > 0.1) {
                robot.intake.setIntakePower(-gamepad1.left_trigger*.9);
            } else {
                robot.intake.setIntakePower(0);
            }

            // Lock deposit`
            if (gamepad2.right_bumper) {
                gamepad1.rumble(500);
                gamepad2.rumble(500);
                robot.intake.engageLock(true, true);
            }
            if(gamepad2.left_bumper){
                gamepad2.rumble(500);
                robot.intake.engageLock(false, true);
            }

            // Release deposit
            if (gamepad2.right_trigger > 0.7) {
                robot.intake.clearLowerLock();
            }
            if (gamepad2.left_trigger > 0.7) {
                robot.intake.clearHigherLock();
            }

            // Flip deposit
            if (gamepad2.dpad_up) {
                if (robot.intake.PIXELS_LOCKED) {
                    robot.intake.flipDeposit();
                }
            } else if (gamepad2.dpad_down) {
                robot.intake.flipIntake();
                robot.intake.engageLock(false, true);
            }

            // Increment Deposit
            if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
                robot.intake.incrementFlip(-0.1);
            }
            if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
                robot.intake.incrementFlip(0.1);
            }

            if (gamepad1.dpad_up) {
                robot.drone.actuationServo.setPwmEnable();
                robot.drone.fireDrone(true);
            } else if (gamepad1.dpad_down) {
                robot.drone.actuationServo.setPwmEnable();
                robot.drone.fireDrone(false);
            } else if (gamepad1.dpad_left) {
                robot.drone.actuationServo.setPwmDisable();
            }


            // Update all state machines
            robot.update();

            // Telemetry
            telemetry.addLine("\uD83C\uDFCE RoboRacers Teleop for Regionals \uD83E\uDEE1");
            telemetry.addData("Slides RunMode", robot.slides.statemachine.getState());
            telemetry.addData("Slides Power", robot.slides.leftmotor.getPower());
            telemetry.addData("Slides Target Position", robot.slides.getTargetPosition());
            telemetry.addData("Intake Power", robot.intake.intakeMotor.getPower());
            telemetry.addLine(String.valueOf(test));
            telemetry.update();
        }
    }
}
