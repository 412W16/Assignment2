import lejos.hardware.Button;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.robotics.EncoderMotor;
import lejos.utility.Delay;
import lejos.utility.Matrix;


public class InverseKinematics {

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
		
//		Q6("a");
		Q7();
		
		Button.waitForAnyPress();
	}

	private void Q7() {
		// TODO Auto-generated method stub
		double[] d1 = getPoint();
		Delay.msDelay(1000);
		double[] d2 = getPoint();
		double[] midpoint = {(d1[0] + d2[0])/2, (d1[1] + d2[1])/2};
		analytical(midpoint, true);
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

	private void Q6(String method) {
		// TODO Auto-generated method stub
		double[] pos = {-5, 15};
		
		if(method == "a"){
			analytical(pos, true);
		} else {
			numerical(pos);
		}
		
	}
	
	private void numerical(double[] pos) {
		// TODO Auto-generated method stub
		
		double theta2=1;
		double theta1=1;
		
		double x = pos[0];
		double y = pos[1];
		
		double[][] A = new double[2][2];
		Matrix J = new Matrix(A);
		
		double[][] F = new double[2][1];
		Matrix f = new Matrix(F);
		
		for(int i=0;i<10;i++) {
			f.set(0, 0, -getX(theta1, theta2, x));
			f.set(1, 0, -getY(theta1, theta2, y));
			
			J.set(0,0, -l1*Math.sin(theta1) - l2 *Math.sin(theta1 + theta2));
			J.set(0, 1, -l2*Math.sin(theta1 + theta2));
			J.set(1, 0, l1 *Math.cos(theta1) + l2*Math.cos(theta1 + theta2));
			J.set(1, 1, l2 * Math.cos(theta1 + theta2));
			J = J.inverse();
			
			Matrix s = J.times(f);
			
			theta1 = theta1 + s.get(0, 0);
			theta2 = theta2 + s.get(1, 0);	
		}
		
		System.out.format("Angle1: %f Angle2: %f\n", -theta1, theta2);
		theta1 = -theta1*(180/Math.PI);
		theta2 = theta2*(180/Math.PI);
		System.out.format("Angle1: %f Angle2: %f\n", -theta1, theta2);
		moveMotor(firstM, theta1);
		moveMotor(secondM, theta2);
	}
	
	public double getX(double theta1, double theta2, double x) {
		return l1*Math.cos(theta1) + l2*Math.cos(theta1 + theta2) - x;
	}
	
	public double getY(double theta1, double theta2, double y) {
		return l1 * Math.sin(theta1) + l2*Math.sin(theta1 + theta2) - y;
	}

	public void moveMotor(EV3LargeRegulatedMotor m, double theta) {
		m.setSpeed(90);
		m.rotateTo((int)theta);
		m.stop();
		m.resetTachoCount();		
	}
	
	public void analytical(double[] pos, boolean interior) {
		double theta2=0;
		double theta1=0;
		
		double x = pos[0];
		double y = pos[1];
		
		double num=0;
		
		double D = (Math.pow(x, 2) + Math.pow(y, 2) - Math.pow(l1, 2) - Math.pow(l2, 2))/(2 * l1 * l2);
		
		
		num = Math.sqrt(1-Math.pow(D, 2));
		
		if(interior) {
			theta2 = Math.atan2(num, D);
		} else {
			theta2 = Math.atan2(-num, D);
		}
		
		theta1 = Math.atan2(y,x) - Math.atan2(l2*Math.sin(theta2), l1 + l2 * Math.cos(theta2));
		
		System.out.format("Angle1: %f Angle2: %f\n", -theta1, theta2);
		theta1 = -theta1*(180/Math.PI);
		theta2 = theta2*(180/Math.PI);
		System.out.format("Angle1: %f Angle2: %f\n", -theta1, theta2);
		moveMotor(firstM, theta1);
		moveMotor(secondM, theta2);
	}
}
