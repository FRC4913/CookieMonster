/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.common;

import static org.firstinspires.ftc.teamcode.common.Constants.*;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp(name = "Arm", group = "TeleOp")
public class Arm extends LinearOpMode {
    boolean endGameRumbled = false;
    boolean finalRumbled = false;
    double armLiftPower = 0.0;
    double armLiftPowerDivider = 4;

    HuskyBot huskyBot = new HuskyBot();
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        huskyBot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        huskyBot.clawLift.setPosition(CLAW_LIFT_START_POSITION);
        huskyBot.clawGrab.setPosition(CLAW_GRAB_OPEN_POSITION);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if ((runtime.seconds() > END_GAME_TIME) && !endGameRumbled) {
                gamepad1.rumble(1000);
                endGameRumbled = true;
            }

            if ((runtime.seconds() > FINAL_TIME) && !finalRumbled) {
                gamepad1.rumble(1000);
                finalRumbled = true;
            }

            // Arm Lift Controls
            armLiftPower = Range.clip(-gamepad2.left_stick_y, -ARM_LIFT_MIN_POWER, ARM_LIFT_MAX_POWER);

            // Arm Lift Motor
            if(huskyBot.armLiftMotor.getCurrentPosition() < ARM_LIFT_MAX_POSITION) {
                huskyBot.armLiftMotor.setPower(armLiftPower == 0 ? ARM_LIFT_POWER_AT_REST : (armLiftPower + ARM_LIFT_POWER_AT_REST));
            }

            telemetry.addLine("ARM");

            telemetry.addData("Lift", "Left Y: (%.2f)", gamepad2.left_stick_y);
            telemetry.addData("Lift", "Power: (%.2f)", huskyBot.armLiftMotor.getPower());
            telemetry.addData("Lift", "Pos: (%d)", huskyBot.armLiftMotor.getCurrentPosition());

            telemetry.addData("Lift Power Divider", armLiftPowerDivider);

            telemetry.addLine(TELEMETRY_SEPARATOR);

            telemetry.update();
        }
    }
}

