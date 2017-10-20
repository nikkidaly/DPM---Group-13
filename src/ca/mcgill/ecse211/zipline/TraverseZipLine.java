package ca.mcgill.ecse211.zipline;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class TraverseZipLine {
	
	  private static final int FORWARD_SPEED = 300;
	  private static final int FORWARD_SLOW = 100;
	  private static final int ROTATE_SPEED = 50;
	  private static final double tileLength = 30.48;
	  private Odometer odometer;
	  private SampleProvider colorSensor;
	  private float[] colorData;
	  private static double lightDensity = 0.05;
	  private double originalX;
	  private double originalY;
	  private double originalTheta;
	  private boolean isXSet = false;
	  private boolean isYSet = false;
	  private boolean isOriginalThetaSet = false;
	
	public TraverseZipLine(Odometer odometer, SampleProvider colorSensor, float[] colorData){
		this.odometer = odometer;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
	}
	public void traverse(){
		if(!isXSet){
			originalX = this.odometer.getX();
			isXSet = true;
		}
		if(!isYSet){
			originalY = this.odometer.getY();
			isYSet = true;
		}
		if (!isOriginalThetaSet){
			originalTheta = this.odometer.getTheta();
			isOriginalThetaSet = true;
		}
		 // move forward until you arrive on the zipline
		
		ZipLineLab.rightMotor.setSpeed(FORWARD_SLOW);
		ZipLineLab.leftMotor.setSpeed(FORWARD_SLOW);
		
		ZipLineLab.leftMotor.rotate(convertDistance(ZipLineLab.WHEEL_RADIUS, 20), true);
	    ZipLineLab.rightMotor.rotate(convertDistance(ZipLineLab.WHEEL_RADIUS, 20), false);
	    
	    ZipLineLab.pulleyMotor.setSpeed(FORWARD_SPEED);
	    ZipLineLab.pulleyMotor.forward();
	    
	    ZipLineLab.rightMotor.setSpeed(FORWARD_SPEED);
		ZipLineLab.leftMotor.setSpeed(FORWARD_SPEED);
	    
	    ZipLineLab.leftMotor.forward();
	    ZipLineLab.rightMotor.forward();
		
	    while (getColorData() < lightDensity) {
    		//try-catch from ultrasonic poller
            try {
            	Thread.sleep(100);
            } catch (InterruptedException e) {
            	//	Auto-generated catch block
            	
            }
	    }
	    Sound.beep();
	    ZipLineLab.rightMotor.stop(true);
	    ZipLineLab.leftMotor.stop(true);
	    
	    while (getColorData() > lightDensity){
	    	try{
	    		Thread.sleep(100);
	    	} catch (InterruptedException e){
	    		
	    	}
	    }
	    Sound.beep();
	    ZipLineLab.rightMotor.setSpeed(FORWARD_SPEED);
		ZipLineLab.leftMotor.setSpeed(FORWARD_SPEED);
		ZipLineLab.pulleyMotor.setSpeed(FORWARD_SLOW);
		
		ZipLineLab.rightMotor.forward();
		ZipLineLab.leftMotor.forward();
		ZipLineLab.pulleyMotor.forward();
		while(getColorData() <0.3){
			
		} try{
			Thread.sleep(100);
		} catch (InterruptedException e){
			
		}
		Sound.beep();
		ZipLineLab.rightMotor.setSpeed(FORWARD_SLOW);
		ZipLineLab.leftMotor.setSpeed(FORWARD_SLOW);
		ZipLineLab.pulleyMotor.stop(true);
		if(Math.abs(originalTheta-0)<10){
			odometer.setY(originalY+5*tileLength);
		}else if(Math.abs(originalTheta-90)<10){
			odometer.setX(originalX+5*tileLength);
		}
		else if(Math.abs(originalTheta-180)<10){
			odometer.setY(originalY-5*tileLength);
		}else{
			odometer.setX(originalX-5*tileLength);
		}
		ZipLineLab.rightMotor.stop(true);
		ZipLineLab.leftMotor.stop(true);
	}
	
  private static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
  }
  private static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
  private float getColorData() {
	    colorSensor.fetchSample(colorData, 0);
	    float colorBrightnessLevel = (colorData[0] + colorData[1] + colorData[2]);
	    return colorBrightnessLevel;
	  }
}
