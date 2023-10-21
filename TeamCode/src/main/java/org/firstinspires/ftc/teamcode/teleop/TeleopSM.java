package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.modules.statemachines.IntakeSM;
import org.firstinspires.ftc.teamcode.modules.statemachines.LauncherSM;
import org.firstinspires.ftc.teamcode.modules.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.modules.subsystems.Intake;

@TeleOp(name = "Teleop For League Tournament", group = "16481-Power-Play")
public class TeleopSM extends LinearOpMode {

Launcher launcher;
Intake intake;

    @Override
    public void runOpMode(){
        while (opModeInInit()){
launcher.statemachine.transition(LauncherSM.EVENT.GAME_START);
intake.statemachine.transition(IntakeSM.EVENT.GAME_START);
        }
        while (opModeIsActive()){
            if (gamepad2.dpad_up){
                launcher.statemachine.transition(LauncherSM.EVENT.DRONE_LAUNCH_BUTTON_PRESSED);
            }

            else if (gamepad2.a){

            }
            else if (gamepad2.a){

            }
            else if (gamepad2.a){

            }
            else if (gamepad2.a){

            }
            else if (gamepad2.a){

            }
            else if (gamepad2.a){

            }

        }

    }
}
            // Telemetry