package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotCore;
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
@Autonomous(name = "Red Close Side Auton", group = "16481-Centerstage")
public class RedCloseAuton extends LinearOpMode{

    RobotCore robot;

    Vision.TeamPropPipeline teamPropDetectionPipeline = null;

    SpikeMarkerLocation spikeMarkerLocation = SpikeMarkerLocation.CENTER; // Defaults to center

    double backBoardX = 53.00;

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

        Pose2d startLocation = new Pose2d(15.85, -62.00, Math.toRadians(90));
        robot.drive.setPoseEstimate(startLocation);




        TrajectorySequence RightCycle = robot.drive.trajectorySequenceBuilder(startLocation)
                .addDisplacementMarker(() -> {
                    robot.intake.engageLock(true,true);
                    robot.intake.flipDeposit();
                })


                .splineToConstantHeading(new Vector2d(24.5, -40.00), Math.toRadians(90))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(27.30, -50.00), Math.toRadians(90))
                .waitSeconds(0.1)
                // Go to backboard
                .splineTo(new Vector2d(backBoardX-24, -38.5), Math.toRadians(0.00))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.drone.actuationServo.setPwmEnable();
                    robot.drone.fireDrone(true);
                })
                .waitSeconds(0.5)
                .splineTo(new Vector2d(backBoardX-2, -38.5), Math.toRadians(0.00))       //CHANGE BACKBOARD X BECAUSE TOO CLOSE TO BACKBOARD
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


                .splineToConstantHeading(new Vector2d(backBoardX-14, -38.5 ), Math.toRadians(0.00))
                .waitSeconds(0.2)
                .splineToConstantHeading(new Vector2d(20, -5), Math.toRadians(0.00))//retreat to middle of field
                //add code to intake it here
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntakePower(0.75);
                })


                .splineToConstantHeading(new Vector2d(-57.0, -16), Math.toRadians(0.00)) // Go into starter stack
                .waitSeconds(0.75)
//                .splineToConstantHeading(new Vector2d(-58.00, -8.50), Math.toRadians(0.00)) // go left and right at stack
//                .splineToConstantHeading(new Vector2d(-58, -12.5), Math.toRadians(0.00)
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    robot.intake.engageLock(true,true);
                    robot.intake.setIntakePower(-0.75);
                })
                .splineToConstantHeading(new Vector2d(-58.00, -14), Math.toRadians(0.00)) // Reverse from  starter stack
                .waitSeconds(0.5)

                .splineToConstantHeading(new Vector2d(20.00, -10), Math.toRadians(0.00))  // Align to the center of the field
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.intake.flipDeposit();
                    robot.intake.setIntakePower(0);
                })
                //.splineToConstantHeading(new Vector2d(20, -10), Math.toRadians(0.00)) // go to middle-ish of field
                .splineToConstantHeading(new Vector2d(backBoardX-2.3, -30), Math.toRadians(0.00))//go to right side of field
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
                .splineToConstantHeading(new Vector2d(backBoardX-10, -30), Math.toRadians(0.00))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.flipIntake();
                })
                .waitSeconds(0.15)
                .UNSTABLE_addTemporalMarkerOffset(0.15 , () -> {
                    robot.slides.setTargetPosition(0);
                    robot.slides.setPower(0.8);
                })
                //.splineToConstantHeading(new Vector2d(46.50, -43.0), Math.toRadians(0.00)) //go to park
                .splineToConstantHeading(new Vector2d(53.43, -55.83), Math.toRadians(0.00)) //go to park more

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




        TrajectorySequence CenterCycle = robot.drive.trajectorySequenceBuilder(startLocation)
                .addDisplacementMarker(() -> {
                    robot.intake.engageLock(true,true);
                    robot.intake.flipDeposit();
                })


                .splineToConstantHeading(new Vector2d(14.00, -33.5), Math.toRadians(90))   // Drop pixel
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(20.00, -41), Math.toRadians(90))   // Reverse
                .splineTo(new Vector2d(backBoardX-24, -38.5), Math.toRadians(0.00))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.drone.actuationServo.setPwmEnable();
                    robot.drone.fireDrone(true);
                })
                .waitSeconds(0.5)
                .splineTo(new Vector2d(backBoardX-2, -35.00), Math.toRadians(0.00)) // Go to backboard
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntakePower(0);
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_RTP
                    );
                    robot.slides.setTargetPosition(-560);
                    robot.slides.setPower(0.8);
                })
                .waitSeconds(0.3)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.clearHigherLock();
                    robot.intake.clearLowerLock();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    robot.intake.flipIntake();
                })
                .waitSeconds(0.10)
                .UNSTABLE_addTemporalMarkerOffset(1.35, () -> {
                    robot.intake.engageLock(false, true);
                    robot.slides.setTargetPosition(0);
                    robot.slides.setPower(0.75);
                })


                .splineToConstantHeading(new Vector2d(45.00, -35.00), Math.toRadians(0.00)) // Reverse from backdrop
                //  .splineToConstantHeading(new Vector2d(46.00, 43.0), Math.toRadians(0.00))
                //.splineToConstantHeading(new Vector2d(53.43, 58.83), Math.toRadians(0.00))   // Park At backdrop

                //Go to starter stack
                // .splineToConstantHeading(new Vector2d(18.00, 58.83), Math.toRadians(0.00))   // Reverse from backdrop
                .splineToConstantHeading(new Vector2d(24.00, -5.00), Math.toRadians(0.00))  // Align to the center of the field
                // .splineToConstantHeading(new Vector2d(-32.00, 8.00), Math.toRadians(0.00))  // Go past the trusses
               // .splineToConstantHeading(new Vector2d(-43.00, -9.00), Math.toRadians(0.00))  // Align with starter stack

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntakePower(0.75);
                })


                .splineToConstantHeading(new Vector2d(-57, -20), Math.toRadians(0.00)) // Go into starter stack
                .waitSeconds(0.75)
                //.splineToConstantHeading(new Vector2d(-56.00, -8.50), Math.toRadians(0.00)) // go left and right at stack
                //.splineToConstantHeading(new Vector2d(-56, -12.5), Math.toRadians(0.00))
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    robot.intake.engageLock(true,true);
                    robot.intake.setIntakePower(-0.8);
                })
                .splineToConstantHeading(new Vector2d(-57, -18), Math.toRadians(0.00))// Reverse from  starter stack
                .waitSeconds(0.75)

                .splineToConstantHeading(new Vector2d(15.00, -9.00), Math.toRadians(0.00))  // Align to the center of the field

                .waitSeconds(0.5)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.flipDeposit();
                })

                .splineTo(new Vector2d(backBoardX-1, -35.00), Math.toRadians(0.00))
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

                .splineToConstantHeading(new Vector2d(backBoardX-10, -35), Math.toRadians(0.00)) //go to park
                .splineToConstantHeading(new Vector2d(53.43, -55.83), Math.toRadians(0.00)) //go to park more

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Unpower slides
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_MANUAL
                    );
                    robot.slides.setPower(0);
                })


                .build();



        TrajectorySequence LeftCycle = robot.drive.trajectorySequenceBuilder(startLocation)
                .addDisplacementMarker(() -> {
                    robot.intake.engageLock(true,true);
                    robot.intake.flipDeposit();
                })

                .splineTo(new Vector2d(6, -33), Math.toRadians(135))
                .waitSeconds(0.1)
                .setReversed(true)
                .splineTo(new Vector2d(30.00, -45), Math.toRadians(-90))
                .setReversed(false)
                // Go to backboard
                .splineTo(new Vector2d(backBoardX-24, -38.5), Math.toRadians(0.00))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.drone.actuationServo.setPwmEnable();
                    robot.drone.fireDrone(true);
                })
                .waitSeconds(0.5)
                .splineTo(new Vector2d(backBoardX-2, -28), Math.toRadians(0.00))        //CHANGE THE BACKBOARD X BECAUSE GOING TO FORWARD
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntakePower(0);
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_RTP
                    );

                    robot.slides.setTargetPosition(-570);
                    robot.slides.setPower(0.8);

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


                .splineToConstantHeading(new Vector2d(backBoardX-14, -28), Math.toRadians(0.00))
                .waitSeconds(0.1)
                .splineToConstantHeading(new Vector2d(20, -5), Math.toRadians(0.00))//retreat to middle of field
                //add code to intake it here
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.intake.setIntakePower(0.75);
                })
                .splineToConstantHeading(new Vector2d(-56.0, -18), Math.toRadians(0.00)) // Go into starter stack
                .waitSeconds(0.75)
                //.splineToConstantHeading(new Vector2d(-54.00, -9.50), Math.toRadians(0.00)) // go left and right at stack
                //.splineToConstantHeading(new Vector2d(-54, -14.5), Math.toRadians(0.00))
                .UNSTABLE_addTemporalMarkerOffset(0.75, () -> {
                    robot.intake.engageLock(true,true);
                    robot.intake.setIntakePower(-0.8);
                })
                .splineToConstantHeading(new Vector2d(-56.00, -16), Math.toRadians(0.00)) // Reverse from  starter stack
                .waitSeconds(0.75)
                //add code to lock
                //add code here to outtake extra

                .splineToConstantHeading(new Vector2d(15.00, -9.00), Math.toRadians(0.00))  // Align to the center of the field
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    robot.intake.flipDeposit();
                    robot.intake.setIntakePower(0);
                })

                .splineToConstantHeading(new Vector2d(backBoardX-1, -30), Math.toRadians(0.00))//go to right side of field

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
                .splineToConstantHeading(new Vector2d(backBoardX-14, -30), Math.toRadians(0.00))
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    robot.intake.engageLock(false, true);
                    robot.slides.setTargetPosition(0);
                    robot.slides.setPower(0.8);
                    robot.intake.flipIntake();
                })
                //.splineToConstantHeading(new Vector2d(44, -43.0), Math.toRadians(0.00)) //park
                .splineToConstantHeading(new Vector2d(50.43, -55.83), Math.toRadians(0.00)) //park more

                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    // Unpower slides
                    robot.slides.statemachine.transition(
                            SlidesSM.EVENT.ENABLE_MANUAL
                    );
                    robot.slides.setPower(0);
                })
                .build();


        //robot.vision.startPropDetection();

        boolean manualPropControl = false;

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
            case LEFT:
                robot.drive.followTrajectorySequenceAsync(LeftCycle);
                break;
            case CENTER:
                robot.drive.followTrajectorySequenceAsync(CenterCycle);
                break;
            case RIGHT:
                robot.drive.followTrajectorySequenceAsync(RightCycle);
                break;
        }

        while (opModeIsActive() && !isStopRequested()) {
            long loop = System.nanoTime();
            robot.update();
            long loopTime = System.nanoTime();

            long time = loopTime-loop;

            telemetry.addData("Looptime", time/1e+6);
            telemetry.addData("Setpoint", robot.slides.getTargetPosition());
            telemetry.addData("Right Slide Motor", robot.slides.rightmotor.getCurrentPosition());
            telemetry.addData("Left Slide Motor", robot.slides.leftmotor.getCurrentPosition());
            telemetry.addData("Right Target", robot.slides.rightmotor.getTargetPosition());
            telemetry.addData("Left Target", robot.slides.leftmotor.getTargetPosition());
            telemetry.update();
        }

    }

}
