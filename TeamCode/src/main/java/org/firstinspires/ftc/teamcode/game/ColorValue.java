package org.firstinspires.ftc.teamcode.game;

/**
 * Stores red, green, blue, and alpha values as ints in one convenient package
 */
public class ColorValue {

    int red, green, blue, alpha;

    public ColorValue(int red, int green, int blue, int alpha){
        this.red = red;
        this.green = green;
        this.blue = blue;
        this.alpha = alpha;
    }

    public int red(){
        return red;
    }

    public int green(){
        return green;
    }

    public int blue(){
        return blue;
    }

    public int alpha(){
        return alpha;
    }

    public void setValues(int red, int green, int blue, int alpha){
        this.red = red;
        this.green = green;
        this.blue = blue;
        this.alpha = alpha;
    }

    public void setRed(int red){
        setValues(red, this.green, this.blue, this.alpha);
    }

    public void setGreen(int green){
        setValues(this.red, green, this.blue, this.alpha);
    }

    public void setBlue(int blue){
        setValues(this.red, this.green, blue, this.alpha);
    }

    public void setAlpha(int alpha){
        setValues(this.red, this.green, this.blue, alpha);
    }
}
