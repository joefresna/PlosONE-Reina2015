package utils;

import java.util.ArrayList;
import java.util.List;

public class GenericUtils {
	
	public static List<Integer> cloneList(List<Integer> list) {
	    List<Integer> clone = new ArrayList<Integer>(list.size());
	    for(Integer item:list) clone.add(new Integer(item));
	    return clone;
	}
	

}
