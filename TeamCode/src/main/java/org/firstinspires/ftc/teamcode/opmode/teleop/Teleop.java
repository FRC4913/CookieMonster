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

package org.firstinspires.ftc.teamcode.opmode.teleop;

import static org.firstinspires.ftc.teamcode.common.Constants.END_GAME_TIME;
import static org.firstinspires.ftc.teamcode.common.Constants.FINAL_TIME;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.HuskyBot;

@Config
@TeleOp(name = "Husky Teleop", group = "TeleOp")
public class Teleop extends LinearOpMode {
    HuskyBot huskyBot = new HuskyBot();

    private final ElapsedTime runtime = new ElapsedTime();

    boolean endGameRumbled = false;
    boolean finalRumbled = false;

    @Override
    public void runOpMode() {

        // region INITIALIZATION
        huskyBot.init(hardwareMap);
        huskyBot.drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // endregion

        // region TELE-OP LOOP
        while (opModeIsActive()) {
            // region CONTROLLER RUMBLE SIGNALS
            if ((runtime.seconds() > END_GAME_TIME) && !endGameRumbled) {
                gamepad1.rumble(1000);
                endGameRumbled = true;
            }

            if ((runtime.seconds() > FINAL_TIME) && !finalRumbled) {
                gamepad1.rumble(1000);
                finalRumbled = true;
            }
            // endregion

            // region DRIVE MECHANISMS
            Pose2d poseEstimate = huskyBot.drive.getPoseEstimate();

            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());
            // uses the left trigger to dynamically shift between different drive speeds.
            // when the trigger is fully released, driveVelocity = 1.
            // when the trigger is fully pressed, driveVelocity = 0.2.
            double driveVelocity = (0.35 + 0.5 * gamepad1.left_trigger);

            huskyBot.drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX() * driveVelocity,
                            input.getY() * driveVelocity,
                            -gamepad1.right_stick_x * driveVelocity
                    )
            );
            huskyBot.drive.update();
            // endregion

            // region TELEMETRY
            telemetry.addData("Status", "Run Time: " + runtime);

            // Drive Mechanism Telemetry
            telemetry.addData("Stick", "y (%.2f), x (%.2f), rx (%.2f)", gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
            telemetry.addData("Heading", poseEstimate.getHeading());

            // Arm Telemetry
            telemetry.addData("Arm Lift", "Left Y: (%.2f), Power: (%.2f), Pos: (%d)", gamepad2.left_stick_y, huskyBot.armLiftMotor.getPower(), huskyBot.armLiftMotor.getCurrentPosition());

            // Claw Telemetry
            telemetry.addData("Claw Lift", "Right Y: (%.2f), Pos: (%.2f)", gamepad2.right_stick_y, huskyBot.clawLift.getPosition());
            telemetry.addData("Claw Grab", "Pos: (%.2f)", huskyBot.clawGrab.getPosition());

            telemetry.addData("current pos ", huskyBot.armLiftMotor.getCurrentPosition());
            telemetry.addData("vel ", huskyBot.armLiftMotor.getVelocity());

            telemetry.update();
            // endregion
        }
        // endregion
    }
}