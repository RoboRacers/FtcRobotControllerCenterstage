package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.modules.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.modules.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.modules.drive.ThreeTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.modules.statemachines.SlidesSM;
import org.firstinspires.ftc.teamcode.modules.subsystems.Vision;
import org.firstinspires.ftc.teamcode.modules.util.SpikeMarkerLocation;
import org.firstinspires.ftc.teamcode.modules.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

// Localization is doesn't show drift, follower if it does

@Config
@Autonomous(name = "Blue Close close auton", group = "16481-Centerstage")
public class BlueCloseAuton extends LinearOpMode{

    RobotCore robot;

    Vision.TeamPropPipeline teamPropDetectionPipeline = null;

    SpikeMarkerLocation spikeMarkerLocation = SpikeMarkerLocation.CENTER; // Defaults to center

    double backBoardX = 53.2;

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

        Pose2d startLocation = new Pose2d(15.85, 62.00, Math.toRadians(-90));
        robot.drive.setPoseEstimate(startLocation);

        TrajectorySequence LeftCycle = robot.drive.trajectorySequenceBuilder(startLocation)

                .addDisplacementMarker(() -> {
                    robot.intake.engageLock(true,true);
                    robot.intake.flipDeposit();
                })


                .splineToConstantHeading(new Vector2d(24.5, 40.00), Math.toRadians(-90))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(27.30, 50.00), Math.toRadians(-90))
                .waitSeconds(0.1)
                .splineTo(new Vector2d(24, 45), Math.toRadians(0.00))//go to drone shoot
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.drone.actuationServo.setPwmEnable();
                    robot.drone.fireDrone(true);
                })
                .waitSeconds(1)
                // Go to backboard
                .splineTo(new Vector2d(backBoardX-2, 38.5), Math.toRadians(0.00))       //CHANGE BACKBOARD X BECAUSE TOO CLOSE TO BACKBOARD

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntakePower(0);
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_RTP
                    );

                    robot.slides.setTargetPosition(-570);
                    robot.slides.setPower(0.8);

                })

                .waitSeconds(0.5)

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.clearHigherLock();
                    robot.intake.clearLowerLock();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    robot.intake.flipIntake();
                    robot.slides.setTargetPosition(0);
                    robot.slides.setPower(0.8);
                    robot.intake.engageLock(false, true);

                })


                .splineToConstantHeading(new Vector2d(backBoardX-10, 38.5 ), Math.toRadians(0.00))
                .waitSeconds(0.2)
                .splineToConstantHeading(new Vector2d(30, 5), Math.toRadians(0.00))//retreat to middle of field
                //add code to intake it here
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntakePower(0.75);
                })


                .splineToConstantHeading(new Vector2d(-56.0, 12.25), Math.toRadians(0.00)) // Go into starter stack
                .waitSeconds(0.75)
                .splineToConstantHeading(new Vector2d(-58.00, 8.50), Math.toRadians(0.00)) // go left and right at stack
//                .splineToConstantHeading(new Vector2d(-58, 12.5), Math.toRadians(0.00))
                .waitSeconds(0.75)
                .splineToConstantHeading(new Vector2d(-40.00, 8.50), Math.toRadians(0.00)) // Reverse from  starter stack
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    robot.intake.engageLock(true,true);
                    robot.intake.setIntakePower(-0.75);
                })
                .splineToConstantHeading(new Vector2d(15.00, 9.00), Math.toRadians(0.00))  // Align to the center of the field
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.intake.flipDeposit();
                    robot.intake.setIntakePower(0);
                })
                //.splineToConstantHeading(new Vector2d(20, 10), Math.toRadians(0.00)) // go to middle-ish of field
                .splineToConstantHeading(new Vector2d(backBoardX-2.3, 30), Math.toRadians(0.00))//go to right side of field
                //add code to drop pixals on backdrop
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    robot.slides.setTargetPosition(-570);
                    robot.slides.setPower(0.8);
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.intake.clearLowerLock();
                    robot.intake.clearHigherLock();

                })
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(backBoardX-10, 35), Math.toRadians(0.00))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.flipIntake();
                })
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.15 , () -> {
                    robot.slides.setTargetPosition(0);
                    robot.slides.setPower(0.8);
                })
                //.splineToConstantHeading(new Vector2d(46.50, 43.0), Math.toRadians(0.00)) //go to park
                .splineToConstantHeading(new Vector2d(53.43, 55.83), Math.toRadians(0.00)) //go to park more

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    //make the deposit optimal for teleopt
                    robot.intake.engageLock(false, true);
                    // Unpower slides
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_MANUAL
                    );
                    robot.slides.setPower(0);
                })

                .build();


        TrajectorySequence CenterWithCycle = robot.drive.trajectorySequenceBuilder(startLocation)

                .addDisplacementMarker(() -> {
                    robot.intake.engageLock(true,true);
                    robot.intake.flipDeposit();
                })


                .splineToConstantHeading(new Vector2d(14.00, 33.55), Math.toRadians(-90))   // Drop pixel
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(20.00, 41), Math.toRadians(-90))   // Reverse
                .splineTo(new Vector2d(26, 45), Math.toRadians(0.00))//go to drone shoot
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    robot.drone.actuationServo.setPwmEnable();
                    robot.drone.fireDrone(true);
                })
                .waitSeconds(0.5)

                .splineTo(new Vector2d(backBoardX-2, 35.00), Math.toRadians(0.00)) // Go to backboard

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntakePower(0);
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_RTP
                    );
                    robot.slides.setTargetPosition(-560);
                    robot.slides.setPower(0.8);
                    robot.drone.fireDrone(false);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.clearHigherLock();
                    robot.intake.clearLowerLock();
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    robot.intake.flipIntake();
                })
                //.waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(1.35, () -> {
                    robot.intake.engageLock(false, true);
                    robot.slides.setTargetPosition(0);
                    robot.slides.setPower(0.75);
                })

                .splineToConstantHeading(new Vector2d(backBoardX-10, 35.00), Math.toRadians(0.00)) // Reverse from backdrop
                .splineToConstantHeading(new Vector2d(30.00, 10.00), Math.toRadians(0.00)) // Reverse from backdrop
                //  .splineToConstantHeading(new Vector2d(46.00, 43.0), Math.toRadians(0.00))
                //.splineToConstantHeading(new Vector2d(53.43, 58.83), Math.toRadians(0.00))   // Park At backdrop

                //Go to starter stack
                // .splineToConstantHeading(new Vector2d(18.00, 58.83), Math.toRadians(0.00))   // Reverse from backdrop
                //.splineToConstantHeading(new Vector2d(18.00, 8.00), Math.toRadians(0.00))  // Align to the center of the field
                // .splineToConstantHeading(new Vector2d(-32.00, 8.00), Math.toRadians(0.00))  // Go past the trusses
                //.splineToConstantHeading(new Vector2d(-43.00, 9.00), Math.toRadians(0.00))  // Align with starter stack

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntakePower(0.75);
                })


                .splineToConstantHeading(new Vector2d(-56.0, 10), Math.toRadians(0.00)) // Go into starter stack
                .waitSeconds(0.75)
                .splineToConstantHeading(new Vector2d(-58.00, 8.50), Math.toRadians(0.00)) // go left and right at stack
                .waitSeconds(0.75)
                //.splineToConstantHeading(new Vector2d(-58, 12.5), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(-45.00, 8.50), Math.toRadians(0.00)) // Reverse from  starter stack
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    robot.intake.engageLock(true,true);
                    robot.intake.setIntakePower(-0.8);
                })
                .splineToConstantHeading(new Vector2d(15.00, 9.00), Math.toRadians(0.00))  // Align to the center of the field

                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.flipDeposit();
                })

                .splineTo(new Vector2d(backBoardX-2, 35.00), Math.toRadians(0.00))
                .waitSeconds(0.25)

                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    robot.intake.setIntakePower(0);
                    robot.slides.setTargetPosition(-600);
                    robot.slides.setPower(0.8);
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.intake.clearHigherLock();
                    robot.intake.clearLowerLock();
                })
                .waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.intake.flipIntake();              //Flip Intake before retracting slides
                })
                .UNSTABLE_addTemporalMarkerOffset(1.25, () -> {
                    robot.slides.setTargetPosition(0);
                    robot.slides.setPower(0.8);
                    robot.intake.engageLock(false, true);
                })

                .splineToConstantHeading(new Vector2d(45.50, 35.0), Math.toRadians(0.00)) //go to park
                .splineToConstantHeading(new Vector2d(53.43, 55.83), Math.toRadians(0.00)) //go to park more

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Unpower slides
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_MANUAL
                    );
                    robot.slides.setPower(0);
                })


                .build();


        TrajectorySequence RightNoCycle = robot.drive.trajectorySequenceBuilder(startLocation)

                .addDisplacementMarker(() -> {
                    robot.intake.engageLock(true,true);
                    robot.intake.flipDeposit();
                })


                .splineTo(new Vector2d(6, 33), Math.toRadians(-135))
                .waitSeconds(0.1)
                .setReversed(true)
                .splineTo(new Vector2d(30.00, 45), Math.toRadians(180.0))
                .setReversed(false)
                .waitSeconds(0.25)
                //.splineTo(new Vector2d(30, 48), Math.toRadians(0.00))//go to drone shoot
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.drone.actuationServo.setPwmEnable();
                    robot.drone.fireDrone(true);
                })
                .waitSeconds(0.5)

                // Go to backboard
                .splineToConstantHeading(new Vector2d(backBoardX-0.5, 30), Math.toRadians(0.00))        //CHANGE THE BACKBOARD X BECAUSE GOING TO FORWARD

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntakePower(0);
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_RTP
                    );

                    robot.slides.setTargetPosition(-570);
                    robot.slides.setPower(0.8);
                    robot.drone.fireDrone(false);

                })
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.clearHigherLock();
                    robot.intake.clearLowerLock();
                })
                .waitSeconds(.5)
                .UNSTABLE_addTemporalMarkerOffset(.5, () -> {
                    robot.intake.flipIntake();
                    robot.intake.engageLock(false, true);
                    robot.slides.setTargetPosition(0);
                    robot.slides.setPower(0.8);

                })


                .splineToConstantHeading(new Vector2d(backBoardX-10, 30), Math.toRadians(0.00))
                //this is my change to the program
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(20, 5), Math.toRadians(0.00))//retreat to middle of field
                //add code to intake it here
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntakePower(0.75);
                })
                .splineToConstantHeading(new Vector2d(-55.0, 10), Math.toRadians(0.00)) // Go into starter stack
                .waitSeconds(0.75)
                .splineToConstantHeading(new Vector2d(-57.00, 8.50), Math.toRadians(0.00)) // go left and right at stack
                .waitSeconds(0.75)
                //.splineToConstantHeading(new Vector2d(-53, 12.5), Math.toRadians(0.00))
                .splineToConstantHeading(new Vector2d(-45.00, 8.50), Math.toRadians(0.00)) // Reverse from  starter stack
                //add code to lock
                //add code here to outtake extra
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    robot.intake.engageLock(true,true);
                    robot.intake.setIntakePower(-0.8);
                })
                .splineToConstantHeading(new Vector2d(15.00, 9.00), Math.toRadians(0.00))  // Align to the center of the field
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.intake.flipDeposit();
                    robot.intake.setIntakePower(0);
                })

                .splineToConstantHeading(new Vector2d(backBoardX-0.5, 30), Math.toRadians(0.00))//go to right side of field

                .UNSTABLE_addTemporalMarkerOffset(0, ()-> {
                    robot.slides.setTargetPosition(-570);
                    robot.slides.setPower(0.8);
                })
                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    robot.intake.clearLowerLock();
                    robot.intake.clearHigherLock();
                })
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(backBoardX-14, 35), Math.toRadians(0.00))
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    robot.intake.engageLock(false, true);
                    robot.slides.setTargetPosition(0);
                    robot.slides.setPower(0.8);
                    robot.intake.flipIntake();
                })
                //.splineToConstantHeading(new Vector2d(44, 43.0), Math.toRadians(0.00)) //park
                .splineToConstantHeading(new Vector2d(50.43, 55.83), Math.toRadians(0.00)) //park more

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Unpower slides
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_MANUAL
                    );
                    robot.slides.setPower(0);
                })
                .build();
        // Close claw


        //robot.vision.startPropDetection();

        boolean manualPropControl = false;

        while(!isStopRequested() && !opModeIsActive()) {

            robot.telemetrySelfCheck(telemetry);

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
            case LEFT:
                robot.drive.followTrajectorySequenceAsync(LeftCycle);
                break;
            case CENTER:
                robot.drive.followTrajectorySequenceAsync(CenterWithCycle);
                break;
            case RIGHT:
                robot.drive.followTrajectorySequenceAsync(RightNoCycle);
                break;
        }

        while (opModeIsActive() && !isStopRequested()) {
            robot.update();

            telemetry.addData("Setpoint", robot.slides.getTargetPosition());
            telemetry.addData("Right Slide Motor", robot.slides.rightmotor.getCurrentPosition());
            telemetry.addData("Left Slide Motor", robot.slides.leftmotor.getCurrentPosition());
            telemetry.addData("Right Target", robot.slides.rightmotor.getTargetPosition());
            telemetry.addData("Left Target", robot.slides.leftmotor.getTargetPosition());
            telemetry.update();
        }

    }

}
