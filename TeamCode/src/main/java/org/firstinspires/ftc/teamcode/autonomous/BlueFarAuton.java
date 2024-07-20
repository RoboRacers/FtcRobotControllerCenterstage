package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.modules.drive.ThreeTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.modules.statemachines.SlidesSM;
import org.firstinspires.ftc.teamcode.modules.subsystems.Vision;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.modules.util.SpikeMarkerLocation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

// Localization is doesn't show drift, follower if it does

@Config
@Autonomous(name = "Blue Far Side Auton", group = "16481-Centerstage")
public class BlueFarAuton extends LinearOpMode{

    RobotCore robot;

    Vision.TeamPropPipeline teamPropDetectionPipeline = null;

    SpikeMarkerLocation spikeMarkerLocation = SpikeMarkerLocation.CENTER; // Defaults to center

    TrajectorySequence LeftSpikeMarker;
    TrajectorySequence LeftDeposit1;
    TrajectorySequence CenterSpikeMarker;
    TrajectorySequence CenterDeposit1;
    TrajectorySequence CenterDeposit2;
    TrajectorySequence CenterCycle;
    TrajectorySequence RightSpikeMarker;
    TrajectorySequence RightDeposit1;

    public static double backBoardX = 47.66;

    public static double wait = 0.1;



    @Override
    public void runOpMode() {

        OpenCvCamera camera;

        robot = new RobotCore(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap
                .get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
                teamPropDetectionPipeline = new Vision.TeamPropPipeline();
                camera.setPipeline(teamPropDetectionPipeline);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        Pose2d startLocation = new Pose2d(-43, 62.00, Math.toRadians(-90));
        robot.drive.setPoseEstimate(startLocation);

        if (isStopRequested()) return;

        LeftSpikeMarker = robot.drive.trajectorySequenceBuilder(startLocation)
                .waitSeconds(wait)
                .addDisplacementMarker(() -> {
                    robot.intake.engageLock(false,true);
                    robot.intake.clearLowerLock();
                    robot.intake.flipIntake();
                })
                .splineToConstantHeading(new Vector2d(-51.64, 37.91), Math.toRadians(270.00))
                //.splineToConstantHeading(new Vector2d(-40.5, -40.16), Math.toRadians(-270.00))
                //move to starter stack
                .setReversed(true)
                .splineTo(new Vector2d(-50.64, 48.50), Math.toRadians(-270))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntakePower(0.35);
                })
                .splineTo(new Vector2d(-62.00, 40.53), Math.toRadians(180))
                // Sweep the stack
                .splineToConstantHeading(new Vector2d(-62.58, 31.53), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-62.58, 35.53), Math.toRadians(180))
                // wait to intake
                .waitSeconds(0.75)
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    robot.intake.engageLock(true,true);
                })
                .splineToConstantHeading(new Vector2d(-62.30, 31.53), Math.toRadians(270))
                // wait and them outtake
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.intake.setIntakePower(-0.8);
                })
                .splineToConstantHeading(new Vector2d(-60.30, 15.53), Math.toRadians(270))

                //move to backdrop

                .splineToConstantHeading(new Vector2d(-43.43, 8.77), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-20.16, 8.77), Math.toRadians(0))
                .waitSeconds(3)
                .splineToConstantHeading(new Vector2d(38.41, 8.73), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                    robot.intake.setIntakePower(0);
                    robot.intake.flipDeposit();


                    robot.drive.setQueuedTrajectorySequence(
                            LeftDeposit1
                    );
                    robot.drive.setWaitConstraints(30, 8000, MecanumDrive.Side.LEFT);
                    robot.drive.startWaiting();
                })
                .build();

        LeftDeposit1 = robot.drive.trajectorySequenceBuilder(LeftSpikeMarker.end())
                .splineToConstantHeading(new Vector2d(backBoardX, 32.99), Math.toRadians(0))//in front of backdrop
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_RTP
                    );
                    robot.slides.setTargetPosition(-610);
                    robot.slides.setPower(0.8);

                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.clearHigherLock();
                    robot.intake.clearLowerLock();
                })
                .splineToConstantHeading(new Vector2d(backBoardX-3, 32.99), Math.toRadians(0))//in front of backdrop
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    robot.slides.setTargetPosition(-10);
                    robot.slides.setPower(0.8);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.33, () -> {
                    //robot.drive.followTrajectorySequenceAsync(CenterCycle);
                })
                .build();


        if (isStopRequested()) return;

        CenterSpikeMarker = robot.drive.trajectorySequenceBuilder(startLocation)
                .waitSeconds(wait)
                .addDisplacementMarker(() -> {
                    robot.intake.engageLock(false,true);
                    robot.intake.clearLowerLock();
                    robot.intake.flipIntake();
                })
                .splineToConstantHeading(new Vector2d(-40.11, 30.75), Math.toRadians(270.00))
                //.splineToConstantHeading(new Vector2d(-40.5, -40.16), Math.toRadians(-270.00))
                //move to starter stack
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntakePower(0.35);
                })
                .splineTo(new Vector2d(-62.00, 40.53), Math.toRadians(180))
                // Sweep the stack
                .splineToConstantHeading(new Vector2d(-62.58, 31.53), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-62.58, 35.53), Math.toRadians(180))
                // wait to intake
                .waitSeconds(0.75)
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    robot.intake.engageLock(true,true);
                })
                .splineToConstantHeading(new Vector2d(-58.30, 31.53), Math.toRadians(270))
                // wait and them outtake
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.intake.setIntakePower(-0.8);
                })
                .splineToConstantHeading(new Vector2d(-58.30, 15.53), Math.toRadians(270))

                //move to backdrop

                .splineToConstantHeading(new Vector2d(-43.43, 8.13), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-20.16, 8.7), Math.toRadians(0))
                .waitSeconds(3)
                .splineToConstantHeading(new Vector2d(38.41, 8.73), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                    robot.intake.setIntakePower(0);
                    robot.intake.flipDeposit();


                    robot.drive.setQueuedTrajectorySequence(
                            CenterDeposit1
                    );
                    robot.drive.setWaitConstraints(30, 8000, MecanumDrive.Side.LEFT);
                    robot.drive.startWaiting();
                })
                .build();

        CenterDeposit1 = robot.drive.trajectorySequenceBuilder(CenterSpikeMarker.end())
                .splineToConstantHeading(new Vector2d(backBoardX, 31.25), Math.toRadians(0))//in front of backdrop
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_RTP
                    );
                    robot.slides.setTargetPosition(-610);
                    robot.slides.setPower(0.8);

                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.clearHigherLock();
                    robot.intake.clearLowerLock();
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    robot.intake.flipIntake();
                    robot.intake.engageLock(false, true);
                    robot.slides.setTargetPosition(0);
                    robot.slides.setPower(0.8);

                })
                .splineToConstantHeading(new Vector2d(backBoardX-3, 35.55), Math.toRadians(0))//in front of backdrop
                .UNSTABLE_addTemporalMarkerOffset(0.33, () -> {
                    //robot.drive.followTrajectorySequenceAsync(CenterCycle);
                })
                .build();

        if (isStopRequested()) return;

        CenterCycle = robot.drive.trajectorySequenceBuilder(CenterDeposit1.end())

                .setReversed(true)
                // Come back to stack
                .splineTo(new Vector2d(29.72, 14.82), Math.toRadians(-170.54))
                .addDisplacementMarker(() -> {
                    robot.intake.flipIntake();
                })
                .splineTo(new Vector2d(-19.72, 11.61), Math.toRadians(-178.68))
                .splineTo(new Vector2d(-48.79, 13.31), Math.toRadians(-180.00))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntakePower(0.55);
                })
                .splineTo(new Vector2d(-62.58, 39.53), Math.toRadians(-180))
                // Stack round 2
                .splineToConstantHeading(new Vector2d(-60.75, 31.53), Math.toRadians(-180))
                .splineToConstantHeading(new Vector2d(-65.75, 24.53), Math.toRadians(-180))
                .waitSeconds(0.75)
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    robot.intake.engageLock(true,true);
                })

                // .splineToConstantHeading(new Vector2d(-58.30, -31.53), Math.toRadians(-270)) buuged
                .splineToConstantHeading(new Vector2d(-60.00, 26.51), Math.toRadians(-180))
                .UNSTABLE_addTemporalMarkerOffset(.65, () -> {
                    robot.intake.setIntakePower(-0.8);
                })

                .splineToConstantHeading(new Vector2d(-58.30, 15.53), Math.toRadians(-180))
                //move to backdrop 2
                .setReversed(false)
                .splineTo(new Vector2d(-43.43, -10.13), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-20.16, 10.7), Math.toRadians(0))

                .splineToConstantHeading(new Vector2d(18.41, 8.73), Math.toRadians(0))
                .addDisplacementMarker(() -> {
                    robot.intake.setIntakePower(0);
                    robot.intake.engageLock(true,true);
                    robot.intake.flipDeposit();
                    robot.intake.setIntakePower(0);
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_RTP
                    );

                    robot.slides.setTargetPosition(-750);
                    robot.slides.setPower(0.8);

                    robot.drive.setQueuedTrajectorySequence(
                            CenterDeposit2
                    );
                    robot.drive.setWaitConstraints(30, 8000, MecanumDrive.Side.LEFT);
                    robot.drive.startWaiting();
                })
                .build();

        CenterDeposit2 = robot.drive.trajectorySequenceBuilder(CenterCycle.end())
                .splineToConstantHeading(new Vector2d(backBoardX, 37.14), Math.toRadians(0))//in front of backdrop
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {


                })
                .waitSeconds(0.25)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.clearHigherLock();
                    robot.intake.clearLowerLock();
                })
                .splineToConstantHeading(new Vector2d(42.5, 37.14), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.intake.flipIntake();
                    robot.slides.setTargetPosition(-10);
                    robot.slides.setPower(0.8);

                })

                //.splineToConstantHeading(new Vector2d(55.00, -37.4), Math.toRadians(0.00))
                .build();

        if (isStopRequested()) return;

        RightSpikeMarker = robot.drive.trajectorySequenceBuilder(startLocation)
                .waitSeconds(wait)
                .addDisplacementMarker(() -> {
                    robot.intake.engageLock(false,true);
                    robot.intake.clearLowerLock();
                    robot.intake.flipIntake();
                })
                .splineTo(new Vector2d(-32.93, 31.82), Math.toRadians(-38.66))
                //.splineToConstantHeading(new Vector2d(-40.5, -40.16), Math.toRadians(-270.00))
                //move to starter stack
                .setReversed(true)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntakePower(0.5);
                })
                .splineTo(new Vector2d(-62.00, 40.53), Math.toRadians(180))
                // Sweep the stack
                .splineToConstantHeading(new Vector2d(-62.58, 31.53), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-62.58, 35.53), Math.toRadians(180))
                // wait to intake
                .waitSeconds(0.75)
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    robot.intake.engageLock(true,true);
                })
                .splineToConstantHeading(new Vector2d(-58.30, 31.53), Math.toRadians(270))
                // wait and them outtake
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.intake.setIntakePower(-0.8);
                })
                .splineToConstantHeading(new Vector2d(-58.30, 15.53), Math.toRadians(270))

                //move to backdrop

                .splineToConstantHeading(new Vector2d(-43.43, 8.13), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-20.16, 8.7), Math.toRadians(0))
                .waitSeconds(3)
                .splineToConstantHeading(new Vector2d(38.41, 8.73), Math.toRadians(0))
                .UNSTABLE_addTemporalMarkerOffset(3, () -> {
                    robot.intake.setIntakePower(0);
                    robot.intake.flipDeposit();

                    robot.drive.setQueuedTrajectorySequence(
                            RightDeposit1
                    );
                    robot.drive.setWaitConstraints(30, 7000, MecanumDrive.Side.LEFT);
                    robot.drive.startWaiting();
                })
                .build();

        RightDeposit1 = robot.drive.trajectorySequenceBuilder(LeftSpikeMarker.end())
                .splineToConstantHeading(new Vector2d(backBoardX, 38.90), Math.toRadians(0))//in front of backdrop
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_RTP
                    );
                    robot.slides.setTargetPosition(-610);
                    robot.slides.setPower(0.8);
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.clearHigherLock();
                    robot.intake.clearLowerLock();
                })
                .waitSeconds(2)
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    robot.intake.flipIntake();
                    robot.intake.engageLock(false, true);
                    robot.slides.setTargetPosition(0);
                    robot.slides.setPower(0.8);

                })
                .splineToConstantHeading(new Vector2d(backBoardX-3, 34.90), Math.toRadians(0))//in front of backdrop
                .UNSTABLE_addTemporalMarkerOffset(0.33, () -> {
                    //robot.drive.followTrajectorySequenceAsync(CenterCycle);
                })
                .build();

        boolean manualPropControl = true;

        while(!isStopRequested() && !opModeIsActive()) {
            telemetry.addLine("SELF CHECK -----");

            // Checks if the positions of the encoders to make sure they are not unplugged
            robot.drive.updatePoseEstimate();
            ThreeTrackingWheelLocalizer localizer = (ThreeTrackingWheelLocalizer) robot.drive.getLocalizer();
            List<Double> deadwheelPositions = localizer.getWheelPositions();

            telemetry.addData("Left Encoder Pos", deadwheelPositions.get(0));
            telemetry.addData("Right Encoder Pos", deadwheelPositions.get(1));
            telemetry.addData("Perpendicular Encoder Pos", deadwheelPositions.get(2));

            if (deadwheelPositions.get(0) == 0) {
                telemetry.addLine("LEFT ENCODER UNPLUGGED, Check wiring of Port x");
            }
            if (deadwheelPositions.get(1) == 0) {
                telemetry.addLine("RIGHT ENCODER UNPLUGGED, Check wiring of Port x");
            }
            if (deadwheelPositions.get(2) == 0) {
                telemetry.addLine("PERPENDICULAR ENCODER UNPLUGGED, Check wiring of Port x");
            }


            // Vision code here
            telemetry.addLine("VISION -----");

            // Switch between manual and automatic vision control
            if (gamepad1.left_bumper) {
                manualPropControl = true;
            } else if (gamepad1.right_bumper) {
                manualPropControl = false;
            }

            if (!manualPropControl) {
                telemetry.addLine("Prop Detection mode is AUTOMATIC");
                if (teamPropDetectionPipeline != null) {
                    spikeMarkerLocation = teamPropDetectionPipeline.getDirection();
                    telemetry.addData("Spike Marker Location", spikeMarkerLocation);
                } else {
                    telemetry.addLine("Camera not initialized");
                }
            } else {
                telemetry.addLine("Prop Detection is MANUAL");
                if (gamepad1.square) {
                    spikeMarkerLocation = SpikeMarkerLocation.LEFT;
                } else if (gamepad1.circle) {
                    spikeMarkerLocation = SpikeMarkerLocation.CENTER;
                } else if (gamepad1.triangle) {
                    spikeMarkerLocation = SpikeMarkerLocation.RIGHT;
                }
                telemetry.addData("Spike Marker Location", spikeMarkerLocation);
            }

            telemetry.update();
        }

        camera.stopStreaming();

        waitForStart();

        if (isStopRequested()) return;

        // Runs the trajectory based on the start location
        switch (spikeMarkerLocation) {
            // Reversed
            case LEFT:
                robot.drive.followTrajectorySequenceAsync(RightSpikeMarker);
                break;
            case CENTER:
                robot.drive.followTrajectorySequenceAsync(CenterSpikeMarker);
                break;
            case RIGHT:
                robot.drive.followTrajectorySequenceAsync(LeftSpikeMarker);
                break;
        }

        while (opModeIsActive() && !isStopRequested()) {
            robot.update();

            telemetry.addData("Setpoint", robot.slides.getTargetPosition());
            telemetry.addData("Right Slide Motor", robot.slides.rightmotor.getCurrentPosition());
            telemetry.addData("Left Slide Motor", robot.slides.leftmotor.getCurrentPosition());
            telemetry.addData("Right Target", robot.slides.rightmotor.getTargetPosition());
            telemetry.addData("Left Target", robot.slides.leftmotor.getTargetPosition());
            telemetry.addData("Left Ultrasonic", robot.drive.leftUltrasonic.getDistance(DistanceUnit.INCH));
            telemetry.addData("Right Ultrasonic", robot.drive.rightUltrasonic.getDistance(DistanceUnit.INCH));
            telemetry.addData("Waiting?", robot.drive.waiting);
            telemetry.update();
        }

    }

}
