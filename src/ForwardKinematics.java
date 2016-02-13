import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.EncoderMotor;
import lejos.utility.Delay;


public class ForwardKinematics {

	public static double l1 = 12.25;
	public static double l2 = 10.2;

	public static EV3LargeRegulatedMotor firstM;
	public static EV3LargeRegulatedMotor secondM;
	public static EV3TouchSensor ts;
	
	public void run(EV3LargeRegulatedMotor f, EV3LargeRegulatedMotor s, EV3TouchSensor t) {
		// TODO Auto-generated method stub
		firstM = f;
		secondM = s;
		ts = t;
		
		Q4();
//		Q5(false);
		Button.waitForAnyPress();
	}
	
	public void Q4() {
		// Motor one needs to move backwards to move in the positive coordinate system
		double t1 = 0.8;
		double t2 = -0.5;
		forwardEst(t1, t2);
		
		double thetaD1 = -t1*(180/Math.PI);
		double thetaD2 = t2*(180/Math.PI);
		moveMotor(firstM, thetaD1);
		moveMotor(secondM, thetaD2);
	}
	
	public void Q5(boolean d) {
		if(d) {
			pointDist();
		} else {
			pointAngle();
		}
	}
	
	
	private void pointAngle() {
		// TODO Auto-generated method stub
		// intersection
		double[] p1 = getPoint();
		Delay.msDelay(1000);
		// line 1
		double[] p2 = getPoint();
		Delay.msDelay(1000);
		// line 2
		double[] p3 = getPoint();
		
		//arcos((P12^2 + P13^2 - P23^2) / (2 * P12 * P13))
		double p12 = dist(p1, p2);
		double p13 = dist(p1, p3);
		double p23 = dist(p2, p3);

		double angle = Math.acos( (Math.pow(p12,2) + Math.pow(p13,2) - Math.pow(p23,2)) / (2*p12*p13) );
		System.out.println(angle);
	}

	private void pointDist() {
		// TODO Auto-generated method stub
		double[] d1 = getPoint();
		Delay.msDelay(1000);
		double[] d2 = getPoint();
		double dist = dist(d1,d2);
		System.out.println(dist);
	}
	
	private double dist(double[] d1, double[] d2) {
		return Math.sqrt(Math.pow(d2[0] - d1[0], 2) + Math.pow(d2[1] - d1[1], 2));
	}
	
	private double[] getPoint() {
		float[] sample = new float[ts.sampleSize()];
		do{
			ts.getTouchMode().fetchSample(sample, 0);
		} while (sample[0] == 0);
		int m1T1 = firstM.getTachoCount();
		int m2T1 = secondM.getTachoCount();
		double[] d1 = forwardEst(-m1T1*(Math.PI/180), m2T1*(Math.PI/180));
		return d1;
	}

	public double[] forwardEst(double theta1, double theta2) {
		double x = l1*Math.cos(theta1) + l2*Math.cos(theta1 + theta2);
		double y = l1 * Math.sin(theta1) + l2*Math.sin(theta1 + theta2);
		System.out.format("x: %f y: %f\n", x, y);
		double[] point = new double[2];
		point[0] = x;
		point[1] = y;
		return point;
	}
	
	public void moveMotor(EV3LargeRegulatedMotor m, double theta) {
		m.setSpeed(90);
		m.rotateTo((int)theta);
		m.stop();
		m.resetTachoCount();		
	}
}
