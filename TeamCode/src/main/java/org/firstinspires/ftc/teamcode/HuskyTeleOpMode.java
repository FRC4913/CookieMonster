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

package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.HuskyBot.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Husky TeleOpMode", group = "TeleOp")
public class HuskyTeleOpMode extends LinearOpMode {

    HuskyBot huskyBot = new HuskyBot();

    // TODO: test arm lift velocity implementation.
    double armLiftVel;


    @Override
    public void runOpMode() {
        huskyBot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        huskyBot.clawLift.setPosition(CLAW_LIFT_START_POSITION);
        huskyBot.clawGrab.setPosition(CLAW_GRAB_CLOSE_POSITION);
   

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // ALL OTHER MECHANISMS REMOVED FOR ARM LIFT TESTING

            // Arm Lift Controls
//
//            if(gamepad2.left_stick_y < 0)
//            {   // on the way up
//                armLiftPowerDivider = 3.5 - (huskyBot.armLiftMotor.getCurrentPosition()/ARM_LIFT_MAX_POSITION);
//            }
//            else { // on the way down
//                armLiftPowerDivider = 5.5;
//            }
//
//            armLiftPower = -gamepad2.left_stick_y/armLiftPowerDivider;
//            armLiftPower = Range.clip(armLiftPower, -ARM_LIFT_MIN_POWER, ARM_LIFT_MAX_POWER);
//
//            if (armLiftPower == 0) {
//                armLiftPower = ARM_LIFT_POWER_AT_REST;
//            }
//            if (huskyBot.armLiftMotor.getCurrentPosition() > ARM_LIFT_MAX_POSITION && armLiftPower > 0) {
//                armLiftPower = 0;
//            }
//
//            huskyBot.armLiftMotor.setPower(armLiftPower);

            //Arm Lift Motor
//            if(huskyBot.armLiftMotor.getCurrentPosition() < ARM_LIFT_MAX_POSITION)
//            {
//                if (armLiftPower == 0) {
//                    huskyBot.armLiftMotor.setPower(ARM_LIFT_POWER_AT_REST);
//                } else {
//                    huskyBot.armLiftMotor.setPower(armLiftPower);
//                }
//            }


            // Alternative Arm Lift Control: run to position
            // adjusts arm lift angle based on motor position, instead of power.
//            huskyBot.armLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            armLiftPosChange = 20 * gamepad2.left_stick_y;
//            if(huskyBot.armLiftMotor.getCurrentPosition() > ARM_LIFT_MAX_POSITION) {
//                armLiftPosChange = (armLiftPosChange > 0) ? 0 : armLiftPosChange;
//            }
//            huskyBot.armLiftMotor.setTargetPosition( (int)armLiftPosChange + huskyBot.armLiftMotor.getCurrentPosition());



            // TODO: test arm lift velocity implementation.
            // Alternative Arm Lift Control: set velocity
            armLiftVel = -gamepad2.left_stick_y * VELOCITY_CONSTANT/3;
            if(armLiftVel < 0) {
                armLiftVel /= 8;
            }
            if(huskyBot.armLiftMotor.getCurrentPosition() > ARM_LIFT_MAX_POSITION) {
                armLiftVel = (armLiftVel > 0) ? 0 : armLiftVel;
            }

            // give power to hold arm steady if its target velocity is close to 0.
            // otherwise, use built-in velocity PID.
            if ( Math.abs(armLiftVel) < VELOCITY_CONSTANT/55 ) { // target velocity low?
                if (Math.abs(huskyBot.armLiftMotor.getVelocity()) < 200) { // actual velocity low?
                    huskyBot.armLiftMotor.setPower(0.05);
                } else {
                    huskyBot.armLiftMotor.setPower(0);
                }
            } else {
                huskyBot.armLiftMotor.setVelocity(armLiftVel);
            }



            telemetry.addData("Arm Lift", "Left Y: (%.2f), Power: (%.2f), Pos: (%d)",
                    gamepad2.left_stick_y, huskyBot.armLiftMotor.getPower(), huskyBot.armLiftMotor.getCurrentPosition());
            telemetry.addData("Arm Lift Target Vel", armLiftVel);
            telemetry.addData("Arm Lift Current Vel", huskyBot.armLiftMotor.getVelocity());
            telemetry.update();
        }
    }
}

