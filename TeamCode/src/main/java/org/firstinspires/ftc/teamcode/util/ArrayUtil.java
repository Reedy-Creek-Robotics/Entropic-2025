package org.firstinspires.ftc.teamcode.util;

public class ArrayUtil {

    /**
     * Find the index of int x in array arr
     * @param arr array to search in
     * @param x int to search for
     * @return index of the int if found in the array, otherwise -1
     */
    public static int findIndexOfItem(int[] arr, int x){
        for(int i = 0; i < arr.length; i++){
            int v = arr[i];
            if(v == x) return i;
        }

        return -1;
    }

    /**
     * Find the index of double x in array arr
     * @param arr array to search in
     * @param x double to search for
     * @return index of the double if found in the array, otherwise -1
     */
    public static int findIndexOfItem(double[] arr, double x){
        for(int i = 0; i < arr.length; i++){
            double v = arr[i];
            if(v == x) return i;
        }

        return -1;
    }

    /**
     * finds the mean of an array
     * @param arr array to average
     * @return the mean of the array, as a double
     */
    public static double mean(int[] arr){
        double total = 0;
        for(int i : arr){
            total += i;
        }
        return total/arr.length;
    }
    
    /**
     * finds the mean of an array
     * @param arr array to average
     * @return the mean of the array, as a double
     */
    public static double mean(double[] arr){
        double total = 0;
        for(double i : arr){
            total += i;
        }
        return total/arr.length;
    }

    /**
     * finds the median of an array
     * @param arr array to find median
     * @return the median of the array, as a double
     */
    public static double median(int[] arr){
        if(arr.length % 2 == 0){
            return (arr[(arr.length/2)-1] + arr[arr.length/2]) / 2.0;
        }else{
            return arr[(int) ((arr.length/2.0) - 1.0 + 0.5)];
        }
    }

    /**
     * finds the median of an array
     * @param arr array to find median
     * @return the median of the array, as a double
     */
    public static double median(double[] arr){
        if(arr.length % 2 == 0){
            return (arr[(arr.length/2)-1] + arr[arr.length/2]) / 2.0;
        }else{
            return arr[(int) ((arr.length/2.0) - 1.0 + 0.5)];
        }
    }

    /**
     * appends an int x to the end of an array arr
     * @param arr array to append to
     * @param x int to append at the end of the array
     * @return int array with x appended at end of array arr
     */
    public static int[] append(int[] arr, int x){
        int[] appended = new int[arr.length + 1];
        System.arraycopy(arr, 0, appended, 0, arr.length);
        appended[arr.length] = x;
        return appended;
    }

    /**
     * appends an double x to the end of an array arr
     * @param arr array to append to
     * @param x double to append at the end of the array
     * @return double array with x appended at end of array arr
     */
    public static double[] append(double[] arr, double x){
        double[] appended = new double[arr.length + 1];
        System.arraycopy(arr, 0, appended, 0, arr.length);
        appended[arr.length] = x;
        return appended;
    }

    public static int[] join(int[] first, int[] second){
        int[] joined = new int[first.length + second.length];
        for(int i = 0; i < first.length; i++){
            joined[i] = first[i];
        }
        for(int i = first.length; i < second.length + first.length; i++){
            joined[i] = second[i-first.length];
        }

        return joined;
    }

    void merge(int[] arr, int l, int m, int r){

        int n1 = m - l + 1;
        int n2 = r - m;

        int[] L = new int[n1];
        int[] R = new int[n2];

        System.arraycopy(arr, l, L, 0, n1);

        for (int j = 0; j < n2; ++j)
            R[j] = arr[m + 1 + j];

        // Merge the temp arrays
        // Initial indexes of first and second subarrays
        int i = 0, j = 0;

        int k = l;
        while (i < n1 && j < n2) {
            if (L[i] <= R[j]) {
                arr[k] = L[i];
                i++;
            }
            else {
                arr[k] = R[j];
                j++;
            }
            k++;
        }

        while (i < n1) {
            arr[k] = L[i];
            i++;
            k++;
        }

        while (j < n2) {
            arr[k] = R[j];
            j++;
            k++;
        }
    }

    // Main function that sorts a[l..r] using
    // merge()
    void sort(int[] arr, int l, int r)
    {
        if (l < r) {

            int m = (l + r) / 2;

            // Sort first and second halves
            sort(arr, l, m);
            sort(arr, m + 1, r);

            // Merge the sorted halves
            merge(arr, l, m, r);
        }
    }
}
