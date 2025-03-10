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

public abstract class MotionProfile {

	public abstract MotionProfile updateEndTime(double endTime) throws MotionProfileException;

	public static class MotionProfileException extends Exception {

		public MotionProfileException(String message) {
			super(message);
		}
		
	}
	
	private final MotionState[] phases;

	public final double startTime;
	protected final double startPosition;
	protected final double endPosition;
	protected final double startVelocity;
	protected final double endVelocity;
	protected final double maxVelocity;
	protected final double maxAcceleration;
	protected final double maxJerk;
	
	MotionProfile(double startTime, double startPosition, double endPosition, double startVelocity, double endVelocity, double maxVelocity, double maxAcceleration, double maxJerk) throws MotionProfileException {
		this.startTime = startTime;
		this.startPosition = startPosition;
		this.endPosition = endPosition;
		this.startVelocity = startVelocity;
		this.endVelocity = endVelocity;
		this.maxVelocity = maxVelocity;
		this.maxAcceleration = maxAcceleration;
		this.maxJerk = maxJerk;
		
		phases = generatePhases();
	}
	
	protected abstract MotionState[] generatePhases() throws MotionProfileException;
	
	protected MotionState getPhaseSetpoint(final double dt, final MotionState initial, MotionState setpoint) {
		if (setpoint == null) {
			setpoint = new MotionState();
		}
		double jt = initial.jerk * dt;
		double jt2 = jt * dt / 2;
		double jt3 = jt2 * dt / 3;
		double at = initial.acceleration * dt;
		double at2 = at * dt / 2;
		double vt = initial.velocity * dt;
		
		setpoint.time         = initial.time + dt;
		setpoint.jerk         = initial.jerk;
		setpoint.acceleration = initial.acceleration + jt;
		setpoint.velocity     = initial.velocity + at + jt2;
		setpoint.position     = initial.position + vt + at2 + jt3;
		return setpoint;
	}

	public boolean getSetpoint(final double time, final MotionState setpoint) {
		
		for (int i = 1; i < phases.length; i++) {
			if (time < phases[i].time) {
				MotionState initial = phases[i-1];
				getPhaseSetpoint(time - initial.time, initial, setpoint);
				return true;
			}
		}
		
		// copy end setpoint
		setpoint.copy(phases[phases.length-1]);
		
		return false;
	}

	public double getEndTime() {
		return phases[phases.length - 1].time;
	}
	
	public double getEndPosition() {
		return phases[phases.length - 1].position;
	}

	public MotionState getLastMotionState() {
		return phases[phases.length - 1];
	}

	@Override
	public String toString() {
		return String.format("startTime %f, startPosition %f, endPosition %f, startVelocity %f, endVelocity %f, maxVelocity %f, maxAcceleration %f", startTime, startPosition, endPosition, startVelocity, endVelocity, maxVelocity, maxAcceleration);
	}

}
