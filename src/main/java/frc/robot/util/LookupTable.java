package frc.robot.util;

import java.util.Map;
import java.util.TreeMap;

public class LookupTable {

    private TreeMap<Double, Double> values;

    public double getValue(Double lookupKey) {
        Map.Entry<Double, Double> higherEntry = values.higherEntry(lookupKey);
        Map.Entry<Double, Double> lowerEntry = values.lowerEntry(lookupKey);
        double distance = (lookupKey - lowerEntry.getKey()) / (higherEntry.getKey() - lowerEntry.getKey());
        return ((higherEntry.getValue() - lowerEntry.getValue()) * distance) + lowerEntry.getValue();
    }

    
}
