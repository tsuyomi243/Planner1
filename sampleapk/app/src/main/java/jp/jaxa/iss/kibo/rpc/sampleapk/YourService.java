        package jp.jaxa.iss.kibo.rpc.sampleapk;
        import android.util.Log;

        import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
        import gov.nasa.arc.astrobee.Result;
        import gov.nasa.arc.astrobee.types.Point;
        import gov.nasa.arc.astrobee.types.Quaternion;

        import org.opencv.aruco.Aruco;
        import org.opencv.aruco.Dictionary;
        import org.opencv.core.Mat;

        import java.lang.reflect.Array;
        import java.util.ArrayList;
        import java.util.Arrays;
        import java.util.List;

        /**
 * Class meant to handle commands from the Ground Data System and execute them in Astrobee
 */
public class YourService extends KiboRpcService {
    private final String TAG = this.getClass().getSimpleName();

    @Override
    protected void runPlan1(){

        // set Waypoint value
        // Write Position and Quaternion here.
        // Waypoint(pos_x,pos_y,pos_z,qua_x,qua_y,qua_z,qua_w)
        Waypoint wp1 = new Waypoint(10.71, -7.77, 4.48, 0, 0.707, 0, 0.707);    // Point1
        Waypoint wp2 = new Waypoint(11.30, -8, 4.55, 0, 0, -0.707, 0.707);      // wp1_From1to2_2
        Waypoint wp3 = new Waypoint(11.30, -9.92, 4.55, 0, 0, -0.707, 0.707);   // wp2_From1to2_2
        Waypoint wp4 = new Waypoint(11.22, -9.92, 5.48, 0, 0, -0.707, 0.707);   // Point2_1
        Waypoint wp5 = new Waypoint(11.30, -9.92, 4.55, 0, 0, -0.707, 0.707);   // wp1_From2toG
        Waypoint wp6 = new Waypoint(11.30, -8.0, 4.55, 0, 0, -0.707, 0.707);   // wp3_From2toG
        Waypoint wp7 = new Waypoint(11.27, -7.89, 4.96, 0, 0, -0.707, 0.707);  // PointGoal_1

        //マーカの設定
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        //ログを取るため
        Log.i(TAG, "start mission");
        /* the mission starts */
        api.startMission();
        // move to a point Point1
        MoveToWaypoint(wp1);
        // report point1 arrival
        api.reportPoint1Arrival();
        // irradiate the laser
        api.laserControl(true);
        // get a camera image
        Mat image1 = api.getMatNavCam();
        //読み取った画像からマーカを認識
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();
        Aruco.detectMarkers(image1, dictionary, corners, markerIds);
        Aruco.drawDetectedMarkers(image1, corners, markerIds);
        api.saveMatImage(image1, "image 1");
        //試しに出力
        Print_AR(corners, markerIds);
        // take target1 snapshots
        api.takeTarget1Snapshot();
        // turn the laser off
        api.laserControl(false);

        // move to a point wp1_From1to2
        MoveToWaypoint(wp2);

        // move to a point wp2_From1to2
        MoveToWaypoint(wp3);

        // move to a point Point2
        MoveToWaypoint(wp4);

        // get a camera image
        Mat image2 = api.getMatNavCam();
        // irradiate the laser
        api.laserControl(true);
        // take target1 snapshots
        api.takeTarget2Snapshot();
        // turn the laser off
        api.laserControl(false);

        // move to a point wp1_From2toG
        MoveToWaypoint(wp5);

        // move to a point wp3_From2toG
        MoveToWaypoint(wp6);

        // move to a point Goal
        MoveToWaypoint(wp7);

        /* ******************************************** */
        /* write your own code and repair the air leak! */
        /* ******************************************** */
        // send mission completion
        api.reportMissionCompletion();
    }
    @Override
    protected void runPlan2(){
        // write here your plan 2
    }
    @Override
    protected void runPlan3(){
        // write here your plan 3
    }
    // You can add your method
    private void moveToWrapper(double pos_x, double pos_y, double pos_z,
                               double qua_x, double qua_y, double qua_z,
                               double qua_w){
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float)qua_x, (float)qua_y,
                (float)qua_z, (float)qua_w);
        api.moveTo(point, quaternion, true);
    }
    private void relativeMoveToWrapper(double pos_x, double pos_y, double pos_z,
                                       double qua_x, double qua_y, double qua_z,
                                       double qua_w) {
        final Point point = new Point(pos_x, pos_y, pos_z);
        final Quaternion quaternion = new Quaternion((float) qua_x, (float) qua_y,
                (float) qua_z, (float) qua_w);
        api.relativeMoveTo(point, quaternion, true);
    }

    private void MoveToWaypoint(Waypoint name){
        moveToWrapper(name.posX, name.posY, name.posZ, name.quaX, name.quaY, name.quaZ, name.quaW);
    }

    private void Print_AR(List<Mat> corners, Mat markerIds){
        for(int id=0; id<4; id++){
                    Log.i(TAG,"markerIds:"+ Arrays.toString(markerIds.get(id,0)));
                    Log.i(TAG,"左上:"+ Arrays.toString(corners.get(id).get(0, 0)));
                    Log.i(TAG,"右上:"+ Arrays.toString(corners.get(id).get(0, 1)));
                    Log.i(TAG,"右下:"+ Arrays.toString(corners.get(id).get(0, 2)));
                    Log.i(TAG,"左下:"+ Arrays.toString(corners.get(id).get(0, 3)));
                }
            }
            /*
            Log.i(TAG,"markerIds:"+ markerIds.get(0,j)[0]);
            Log.i(TAG,"左上:"+ Arrays.toString(corners.get(i).get(0, 0)));
            Log.i(TAG,"右上:"+ Arrays.toString(corners.get(i).get(0, 1)));
            Log.i(TAG,"右下:"+ Arrays.toString(corners.get(i).get(0, 2)));
            Log.i(TAG,"左下:"+ Arrays.toString(corners.get(i).get(0, 3)));
            */


}

