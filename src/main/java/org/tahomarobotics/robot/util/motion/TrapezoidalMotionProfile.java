/**
 * Copyright 2018 Tahoma Robotics - http://tahomarobotics.org - Bear Metal 2046 FRC Team
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated 
 * documentation files (the "Software"), to deal in the Software without restriction, including without 
 * limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
 * Software, and to permit persons to whom the Software is furnished to do so, subject to the following 
 * conditions:
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions 
 * of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED 
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL 
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF 
 * CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER 
 * DEALINGS IN THE SOFTWARE.
 */
package org.tahomarobotics.robot.util.motion;

public class TrapezoidalMotionProfile extends MotionProfile {

	public TrapezoidalMotionProfile(double startTime, double startPosition, double endPosition, double startVelocity, double endVelocity, double maxVelocity, double maxAcceleration) throws MotionProfileException {
		super(startTime, startPosition, endPosition, startVelocity, endVelocity, maxVelocity, maxAcceleration, 0);
	}

	@Override
	public MotionProfile updateEndTime(double endTime) throws MotionProfileException {

		// close enough
		if (Math.abs(endTime - getEndTime()) < 1E-06)
			return this;

		double scale = (getEndTime() - startTime) / (endTime - startTime);


		return new TrapezoidalMotionProfile(this.startTime, this.startPosition, this.endPosition,
											this.startVelocity, this.endVelocity * scale,
											this.maxVelocity * scale, this.maxAcceleration * scale * scale);

	}

	@Override
	protected MotionState[] generatePhases() {

		// all velocity and acceleration will be corrected for sign
		final double distance = endPosition - startPosition;
		final double abs_distance = Math.abs(distance);
		final double direction = Math.signum(distance);

		double start_velocity = Math.abs(startVelocity);
		double end_velocity = Math.abs(endVelocity);

		double max_velocity = Math.min(
				maxVelocity,
				Math.sqrt(abs_distance*maxAcceleration + startVelocity * startVelocity / 2 	+ endVelocity * endVelocity / 2));


		if (end_velocity > max_velocity) {
			max_velocity = Math.sqrt(abs_distance*maxAcceleration*2 + start_velocity * start_velocity);
			end_velocity = max_velocity;
		}

		final double ta = Math.max(0, (max_velocity - start_velocity) / maxAcceleration);
		final double td = Math.max(0, (max_velocity - end_velocity) / maxAcceleration);
		final double tv = Math.max(0, abs_distance > 0 ? ((abs_distance - 0.5*ta*(max_velocity + start_velocity) - 0.5*td*(max_velocity + end_velocity)) / max_velocity) : 0);

		
		double max_acceleration = direction * maxAcceleration;
		max_velocity *= direction;
		start_velocity *= direction;
		end_velocity *= direction;
		
		MotionState[] phases = new MotionState[4];

		// initial state
		MotionState initial = phases[0] = new MotionState()
				.setTime(startTime)
				.setPosition(startPosition)
				.setVelocity(start_velocity)
				.setAcceleration(max_acceleration);
		
		// end of constant acceleration
		initial = phases[1] = getPhaseSetpoint(ta, initial, new MotionState())
				.setVelocity(max_velocity)
				.setAcceleration(0);
		
		// end of constant velocity
		initial = phases[2] = getPhaseSetpoint(tv, initial, new MotionState())
				.setAcceleration(-max_acceleration);

		// end of constant deceleration
		phases[3] = getPhaseSetpoint(td, initial, new MotionState())
				.setVelocity(end_velocity)
				.setPosition(endPosition)
				.setAcceleration(0);

		return phases;
	}
}
