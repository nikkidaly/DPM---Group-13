package ca.mcgill.ecse211.zipline;

import java.util.Set;		


import ca.mcgill.ecse211.zipline.UltrasonicPoller;
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class Navigation implements UltrasonicController {
	  private static final int FORWARD_SPEED = 250;
	  private static final int ROTATE_SPEED = 50;
	  private static final double tileLength = 30.48;
	  
	  private  Odometer odometer;
//	  public static  EV3LargeRegulatedMotor ZipLineLab.rightMotor;
//	  public static  EV3LargeRegulatedMotor leftMotor;	  	  
	  private static final Port usPort = LocalEV3.get().getPort("S2");
	  private static final int bandCenter = 38; // Offset from the wall (cm)
	  private static final int bandWidth = 5;
	  private boolean isAvoidingWall;
	  private double wallDistance;
	  private static double minDistance = 30;
	  

	  
	  public Navigation(boolean isAvoidingWall){
		  this.isAvoidingWall = isAvoidingWall;
	  }
	  public void travelTo (SampleProvider usDistance, Odometer odometer, 
			  double leftRadius, double rightRadius, double width, double x, double y, double xC, double yC){
		  
		  
		  	
		    
    	  
		    // wait 5 seconds
//		    try {
//		      Thread.sleep(2000);
//		    } catch (InterruptedException e) {
//		      // there is nothing to be done here because it is not expected that
//		      // the odometer will be interrupted by another thread
//		    }
//		    
//		    
		    double theta;
//		    if((Math.abs(x*tileLength-odometer.getX()))<10 ){
//		    	if(y*tileLength-odometer.getY()>0){
//		    		theta = 0;
//		    		turnTo(theta,odometer, ZipLineLab.rightMotor, ZipLineLab.leftMotor,leftRadius,rightRadius,width);
//		    	}
//		    	else{
//		    		theta = 180;
//		    		turnTo(theta,odometer, ZipLineLab.rightMotor, ZipLineLab.leftMotor,leftRadius,rightRadius,width);
//		    	}	
//		    }
//		    else if(Math.abs((y*tileLength-odometer.getY()))<10){
//		    	if(x*tileLength-odometer.getX()>0){
//		    		theta =90 ;
//		    		turnTo(theta,odometer, ZipLineLab.rightMotor, ZipLineLab.leftMotor,leftRadius,rightRadius,width);
//		    	}
//		    	else{
//		    		theta =270;
//		    		turnTo(theta,odometer, ZipLineLab.rightMotor, ZipLineLab.leftMotor,leftRadius,rightRadius,width);
//		    	}
//		    	
//		    }
//		    else{
//		    	
//				 // if the x and y error is large it will change the angle of the robot
//					if(y*tileLength > odometer.getY())	{	
//						turnTo(Math.toDegrees(Math.atan((x*tileLength-odometer.getX())/(y*tileLength-odometer.getY())))
//								,odometer, ZipLineLab.rightMotor, ZipLineLab.leftMotor,leftRadius,rightRadius,width);
//					}
//					else if(x*tileLength < odometer.getX()){	
//						turnTo((-1)*Math.toDegrees(Math.atan((y*tileLength-odometer.getY())/(x*tileLength
//								-odometer.getX()))) - 90,
//						odometer, ZipLineLab.rightMotor, ZipLineLab.leftMotor,leftRadius,rightRadius,width);
//					}
//					else	{				
//						turnTo((-1)*Math.toDegrees(Math.atan((y*tileLength-odometer.getY())/(
//								x*tileLength-odometer.getX()))) + 90
//								,odometer, ZipLineLab.rightMotor, ZipLineLab.leftMotor,leftRadius,rightRadius,width);
//					}
//				
//		    }
//
////		    if(wallDistance<minDistance){
////		    	if(odometer.getTheta()>268 && odometer.getTheta()<272){
////		    		ZipLineLab.leftMotor.setSpeed(ROTATE_SPEED);
////		    		ZipLineLab.rightMotor.setSpeed(ROTATE_SPEED);
////		    		ZipLineLab.leftMotor.rotate(convertAngle(leftRadius, width, -90), true);  
////		    		ZipLineLab.rightMotor.rotate(-convertAngle(rightRadius, width, -90), false); 
////		    		ZipLineLab.leftMotor.setSpeed(FORWARD_SPEED); 
////		    		ZipLineLab.rightMotor.setSpeed(FORWARD_SPEED); 
////                	ZipLineLab.leftMotor.rotate(convertDistance(leftRadius, 30), true);  
////                	ZipLineLab.rightMotor.rotate(convertDistance(rightRadius, 30), false);
////                
////                	ZipLineLab.leftMotor.setSpeed(ROTATE_SPEED);
////                	ZipLineLab.rightMotor.setSpeed(ROTATE_SPEED);
////                	ZipLineLab.leftMotor.rotate(convertAngle(leftRadius, width, 90),true);  
////                	ZipLineLab.rightMotor.rotate(-convertAngle(rightRadius, width, 90),false); 
////                	ZipLineLab.leftMotor.setSpeed(FORWARD_SPEED); 
////                	ZipLineLab.rightMotor.setSpeed(FORWARD_SPEED); 
////                	ZipLineLab.leftMotor.rotate(convertDistance(leftRadius, 45), true);  
////                	ZipLineLab.rightMotor.rotate(convertDistance(rightRadius, 45), false);
////                
////                	ZipLineLab.leftMotor.setSpeed(ROTATE_SPEED);
////                	ZipLineLab.rightMotor.setSpeed(ROTATE_SPEED);
////                	ZipLineLab.leftMotor.rotate(convertAngle(leftRadius, width, 90),true);  
////                	ZipLineLab.rightMotor.rotate(-convertAngle(rightRadius, width, 90),false); 
////                	ZipLineLab.leftMotor.setSpeed(FORWARD_SPEED); 
////                	ZipLineLab.rightMotor.setSpeed(FORWARD_SPEED); 
////                	ZipLineLab.leftMotor.rotate(convertDistance(leftRadius, 30), true);  
////                	ZipLineLab.rightMotor.rotate(convertDistance(rightRadius, 30), false);
////                
////                	ZipLineLab.leftMotor.setSpeed(ROTATE_SPEED);
////		    		ZipLineLab.rightMotor.setSpeed(ROTATE_SPEED);
////		    		ZipLineLab.leftMotor.rotate(convertAngle(leftRadius, width, -90),true);  
////		    		ZipLineLab.rightMotor.rotate(-convertAngle(rightRadius, width, -90),false);
////		    	}
////		    	else{
////		    		
////		    	
////		    		ZipLineLab.leftMotor.setSpeed(ROTATE_SPEED);
////		    		ZipLineLab.rightMotor.setSpeed(ROTATE_SPEED);
////		    		ZipLineLab.leftMotor.rotate(convertAngle(leftRadius, width, 90), true);  
////		    		ZipLineLab.rightMotor.rotate(-convertAngle(rightRadius, width, 90), false); 
////		    		ZipLineLab.leftMotor.setSpeed(FORWARD_SPEED); 
////		    		ZipLineLab.rightMotor.setSpeed(FORWARD_SPEED); 
////                	ZipLineLab.leftMotor.rotate(convertDistance(leftRadius, 30), true);  
////                	ZipLineLab.rightMotor.rotate(convertDistance(rightRadius, 30), false);
////                
////                	ZipLineLab.leftMotor.setSpeed(ROTATE_SPEED);
////                	ZipLineLab.rightMotor.setSpeed(ROTATE_SPEED);
////                	ZipLineLab.leftMotor.rotate(convertAngle(leftRadius, width, -90),true);  
////                	ZipLineLab.rightMotor.rotate(-convertAngle(rightRadius, width, -90),false); 
////                	ZipLineLab.leftMotor.setSpeed(FORWARD_SPEED); 
////                	ZipLineLab.rightMotor.setSpeed(FORWARD_SPEED); 
////                	ZipLineLab.leftMotor.rotate(convertDistance(leftRadius, 50), true);  
////                	ZipLineLab.rightMotor.rotate(convertDistance(rightRadius, 50), false);
////                
////                	ZipLineLab.leftMotor.setSpeed(ROTATE_SPEED);
////                	ZipLineLab.rightMotor.setSpeed(ROTATE_SPEED);
////                	ZipLineLab.leftMotor.rotate(convertAngle(leftRadius, width, -90),true);  
////                	ZipLineLab.rightMotor.rotate(-convertAngle(rightRadius, width, -90),false); 
////                	ZipLineLab.leftMotor.setSpeed(FORWARD_SPEED); 
////                	ZipLineLab.rightMotor.setSpeed(FORWARD_SPEED); 
////                	ZipLineLab.leftMotor.rotate(convertDistance(leftRadius, 30), true);  
////                	ZipLineLab.rightMotor.rotate(convertDistance(rightRadius, 30), false);
////                
////                	ZipLineLab.leftMotor.setSpeed(ROTATE_SPEED);
////		    		ZipLineLab.rightMotor.setSpeed(ROTATE_SPEED);
////		    		ZipLineLab.leftMotor.rotate(convertAngle(leftRadius, width, 90),true);  
////		    		ZipLineLab.rightMotor.rotate(-convertAngle(rightRadius, width, 90),false);
////		    	}
////		    }
////		    double distance = Math.sqrt((x*tileLength-odometer.getX())*(x*tileLength-odometer.getX())
////		    		+(y*tileLength-odometer.getY())*(y*tileLength-odometer.getY()));
		    double xDistance = x*tileLength-odometer.getX();
		    ZipLineLab.leftMotor.setSpeed(ROTATE_SPEED);
		    ZipLineLab.rightMotor.setSpeed(ROTATE_SPEED);
		    
		    if(Math.abs(xDistance)<10){ //If the destination is on the same X axis, do not rotate
		    	xDistance = 0;
		    }else if(x*tileLength>odometer.getX()){ //If the destination is to the right of the current position, rotate 90 degrees clockwise
		    	turnTo(90,odometer,leftRadius,rightRadius,width);
		    } else if(x*tileLength<odometer.getX()){ // Otherwise, rotate 90 degrees counter clock wise
		    	turnTo(270,odometer,leftRadius,rightRadius,width);
		    }
		    
		    ZipLineLab.leftMotor.setSpeed(FORWARD_SPEED);
		    ZipLineLab.rightMotor.setSpeed(FORWARD_SPEED);

		    ZipLineLab.leftMotor.rotate(convertDistance(leftRadius, Math.abs(xDistance)), true);
		    ZipLineLab.rightMotor.rotate(convertDistance(rightRadius, Math.abs(xDistance)),false);
		    
		    ZipLineLab.leftMotor.setSpeed(ROTATE_SPEED);
		    ZipLineLab.rightMotor.setSpeed(ROTATE_SPEED);
		    
		    double yDistance = y*tileLength-odometer.getY();
		    
		    //Same rotation logic as on the x axis
		    
		    if(Math.abs(yDistance)<10){
		    	yDistance = 0;
		    }else if(y*tileLength>odometer.getY()){
		    	turnTo(0,odometer,leftRadius,rightRadius,width);
		    }
		    else if(y*tileLength<odometer.getY()){
		    	turnTo(180,odometer,leftRadius,rightRadius,width);
		    }
		    
		    ZipLineLab.leftMotor.setSpeed(FORWARD_SPEED);
		    ZipLineLab.rightMotor.setSpeed(FORWARD_SPEED);

		    ZipLineLab.leftMotor.rotate(convertDistance(leftRadius, Math.abs(yDistance)), true);
		    ZipLineLab.rightMotor.rotate(convertDistance(rightRadius, Math.abs(yDistance)), false);
		    
		    if((Math.abs(xC*tileLength-odometer.getX()))<10 ){
		    	if(yC*tileLength-odometer.getY()>0){
		    		theta = 0;
		    		turnTo(theta,odometer,leftRadius,rightRadius,width);
		    		ZipLineLab.leftMotor.stop(true);
		    		ZipLineLab.rightMotor.stop(true);
		    	}
		    	else{
		    		theta = 180;
		    		turnTo(theta,odometer,leftRadius,rightRadius,width);
		    		ZipLineLab.leftMotor.stop(true);
		    		ZipLineLab.rightMotor.stop(true);
		    	}	
		    }
		    else if(Math.abs((yC*tileLength-odometer.getY()))<10){
		    	if(xC*tileLength-odometer.getX()>0){
		    		theta =90 ;
		    		turnTo(theta,odometer,leftRadius,rightRadius,width);
		    		ZipLineLab.leftMotor.stop(true);
		    		ZipLineLab.rightMotor.stop(true);
		    	}
		    	else{
		    		theta =270;
		    		turnTo(theta,odometer,leftRadius,rightRadius,width);
		    		ZipLineLab.leftMotor.stop(true);
		    		ZipLineLab.rightMotor.stop(true);
		    	}
		    	
		    }
		    else{
		    	
				 // if the x and y error is large it will change the angle of the robot
					if(yC*tileLength > odometer.getY())	{	
						turnTo(Math.toDegrees(Math.atan((xC*tileLength-odometer.getX())/(yC*tileLength-odometer.getY())))
								,odometer,leftRadius,rightRadius,width);
						ZipLineLab.leftMotor.stop(true);
			    		ZipLineLab.rightMotor.stop(true);
					}
					else if(xC*tileLength < odometer.getX()){	
						turnTo((-1)*Math.toDegrees(Math.atan((yC*tileLength-odometer.getY())/(xC*tileLength
								-odometer.getX()))) - 90,
						odometer,leftRadius,rightRadius,width);
						ZipLineLab.leftMotor.stop(true);
			    		ZipLineLab.rightMotor.stop(true);
					}
					else	{				
						turnTo((-1)*Math.toDegrees(Math.atan((yC*tileLength-odometer.getY())/(
								xC*tileLength-odometer.getX()))) + 90
								,odometer,leftRadius,rightRadius,width);
						ZipLineLab.leftMotor.stop(true);
			    		ZipLineLab.rightMotor.stop(true);
					}
				
		    }
	  }
	  

	
	  public void turnTo (double theta,Odometer odometer, 
			  double leftRadius, double rightRadius, double width){
		 
		  ZipLineLab.leftMotor.setSpeed(ROTATE_SPEED);
	      ZipLineLab.rightMotor.setSpeed(ROTATE_SPEED);
	      while(theta-odometer.getTheta()>360){
	    	  theta = theta-360;
	      }
	      while (theta-odometer.getTheta()<0){
	    	  theta = theta+360;
	      }
		  if((theta-odometer.getTheta())>180){
			  
			  ZipLineLab.rightMotor.rotate(convertAngle(rightRadius, width, 360-(theta-odometer.getTheta())), true);
			  ZipLineLab.leftMotor.rotate((-convertAngle(leftRadius, width, 360-(theta-odometer.getTheta()))), false);			  
		  }
		  else{
			  ZipLineLab.leftMotor.rotate((convertAngle(leftRadius, width, theta-odometer.getTheta())), true);
			  ZipLineLab.rightMotor.rotate(-convertAngle(rightRadius, width, theta-odometer.getTheta()), false);
		  }
		  
	  }
	  private static int convertDistance(double radius, double distance) {
		    return (int) ((180.0 * distance) / (Math.PI * radius));
	  }
	  private static int convertAngle(double radius, double width, double angle) {
		    return convertDistance(radius, Math.PI * width * angle / 360.0);
	  }
	  
	  
//	  public static boolean isNavigating(){
//		  
//		
//	  }
	@Override
	public void processUSData(int distance) {
		this.wallDistance = distance;
	}
	@Override
	public int readUSDistance() {
		return 0;
	}
	
}
