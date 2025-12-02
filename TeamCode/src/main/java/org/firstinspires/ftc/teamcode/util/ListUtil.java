package org.firstinspires.ftc.teamcode.util;

import java.util.List;

public class ListUtil {
    public static int findIndexOfItem(List<Integer> list, int item){
        for(int i = 0; i < list.size(); i++){
            int v = list.get(i);
            if(v == item) return i;
        }
        
        return -1;
    }

    public static int findIndexOfItem(List<Double> list, double item){
        for(int i = 0; i < list.size(); i++){
            double v = list.get(i);
            if(v == item) return i;
        }

        return -1;
    }
}
