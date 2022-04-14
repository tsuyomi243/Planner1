package jp.jaxa.iss.kibo.rpc.sampleapk;

public class Waypoint {

    double posX;
    double posY;
    double posZ;
    double quaX;
    double quaY;
    double quaZ;
    double quaW;

    public Waypoint(double posX, double posY, double posZ, double quaX, double quaY, double quaZ, double quaW) {
        this.posX = posX;
        this.posY = posY;
        this.posZ = posZ;
        this.quaX = quaX;
        this.quaY = quaY;
        this.quaZ = quaZ;
        this.quaW = quaW;
    }

}

