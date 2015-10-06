package utils;

import java.lang.Math;

public class Angles {
	
	public static double DegreesToRadians(double degrees){
		return degrees * Math.PI / 180.0;
	}
	
	// min distance from point 1 to point 2 (on the circumference). if positive it is clockwise, if negative counterclockwise
	public static double distanceBetweenTwoPointsOnCircumference(double point1Deg, double point2Deg){
		double diff = point2Deg - point1Deg;
		// normalize between [-180, 180]
		while (diff > 180){
			diff -= 360;
		}
		while (diff < -180){
			diff += 360;
		}
		
		return diff;
	}
	
}
