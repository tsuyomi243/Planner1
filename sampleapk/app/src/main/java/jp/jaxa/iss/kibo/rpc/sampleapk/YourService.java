        package jp.jaxa.iss.kibo.rpc.sampleapk;

        import android.util.Log;

        import jp.jaxa.iss.kibo.rpc.api.KiboRpcService;
        import gov.nasa.arc.astrobee.Result;
        import gov.nasa.arc.astrobee.types.Point;
        import gov.nasa.arc.astrobee.types.Quaternion;

        import org.opencv.aruco.Aruco;
        import org.opencv.aruco.Dictionary;
        import org.opencv.core.Mat;

        import org.opencv.core.MatOfDouble;

        import org.opencv.core.Scalar;
        import org.opencv.imgproc.Imgproc;

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
        // Waypoint(pos_x, pos_y, pos_z, qua_x, qua_y, qua_z, qua_w, avoidX, avoidY, avoidZ)
        //X : right,left Y : back, front Z : down,up
        Waypoint wp1 = new Waypoint(10.71, -7.77, 4.48,
                                    0, 0.707, 0, 0.707,
                                    0, 0, 0.05);    // Point1
        Waypoint wp2 = new Waypoint(11.30, -8, 4.55,
                                    0, 0, -0.707, 0.707,
                                    0, 0, 0);      // wp1_From1to2
        Waypoint wp3 = new Waypoint(11.30, -9.92, 4.55,
                                    0, 0, -0.707, 0.707,
                                    0, 0, -0.05);   // wp2_From1to2_2
        Waypoint wp4 = new Waypoint(11.204, -9.92, 5.47,
                                    0, 0, -0.707, 0.707,
                                    0, 0, -0.01);   // Point2_1
        Waypoint wp5 = new Waypoint(11.30, -9.92, 4.55,
                                    0, 0, -0.707, 0.707,
                                    0, 0, 0);   // wp1_From2toG
        Waypoint wp6 = new Waypoint(11.30, -8.0, 4.55,
                                    0, 0, -0.707, 0.707,
                                    0, 0, -0.05);   // wp3_From2toG
        Waypoint wp7 = new Waypoint(11.27, -7.89, 4.96,
                                    0, 0, -0.707, 0.707,
                                    0, 0, 0);  // PointGoal_1


        //マーカの設定
        Dictionary dictionary = Aruco.getPredefinedDictionary(Aruco.DICT_5X5_250);
        //ログを取るため
        Log.i(TAG, "start mission");
        // the mission starts
        api.startMission();
        // move to a point Point1
        MoveToWaypoint(wp1);
        // report point1 arrival
        api.reportPoint1Arrival();
        // get a camera image
        Mat image1 = api.getMatNavCam();
        // irradiate the laser
        api.laserControl(true);


        //マーカ関連の宣言
        List<Mat> corners = new ArrayList<>();
        Mat markerIds = new Mat();

        //読み取った画像からマーカを認識
        Aruco.detectMarkers(image1, dictionary, corners, markerIds);
        Aruco.drawDetectedMarkers(image1, corners, markerIds);
        api.saveMatImage(image1, "image1.png");


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
        // image2 = gray image
        // image2_color = RGB image
        Mat image2 = api.getMatNavCam();
        Mat image2_color = new Mat();
        Imgproc.cvtColor(image2, image2_color, Imgproc.COLOR_GRAY2RGB);


        //image2のマーカー検出
        Aruco.detectMarkers(image2, dictionary, corners, markerIds);

        /*
            #動作が不安定なため一時的に削除

                // 右下のマーカーを探す
                // in -> corner
                // out -> topLeftとなるマーカが配列の何番目かを表す数字n
                int br_num = findBottomRight(corners);
                String str = "" + br_num;
                Log.i(TAG, str);
        */

        // 一時的な解決策
        br_num = 1

        //4隅の座標を取得
        //右回りcorners.get(n)のリスト [右下、左下、左上、右上]x2
        int[] num_clockwise = {1,0,2,3,1,0,2,3};
        double[] xy_bottomRight = new double[2];
        double[] xy_bottomLeft = new double[2];
        double[] xy_topLeft = new double[2];
        double[] xy_topRight = new double[2];
        for(int n=0; n<4; n++){
            if(br_num == num_clockwise[n]){
                xy_bottomRight = corners.get(num_clockwise[n]).get(0,2);    // 右下
                xy_bottomLeft = corners.get(num_clockwise[n+1]).get(0,3);    // 左下
                xy_topLeft = corners.get(num_clockwise[n+2]).get(0,0);      // 左上
                xy_topRight = corners.get(num_clockwise[n+3]).get(0,1);     //右上
                // for Debug
                Log.i(TAG, "xy_bottomRight:" + (int)xy_bottomRight[0] + "," + (int)xy_bottomRight[1]);
                Log.i(TAG, "xy_bottomLeft:" + (int)xy_bottomLeft[0] + "," + (int)xy_bottomLeft[1]);
                Log.i(TAG, "xy_topLeft:" + (int)xy_topLeft[0] + "," + (int)xy_topLeft[1]);
                Log.i(TAG, "xy_topRight:" + (int)xy_topRight[0] + "," + (int)xy_topRight[1]);
                Imgproc.circle(image2_color, new org.opencv.core.Point((int)xy_bottomRight[0], (int)xy_bottomRight[1]), 1, new Scalar(0,255,0), 3, 8, 0 );
                Imgproc.circle(image2_color, new org.opencv.core.Point((int)xy_bottomLeft[0], (int)xy_bottomLeft[1]), 1, new Scalar(0,255,0), 3, 8, 0 );
                Imgproc.circle(image2_color, new org.opencv.core.Point((int)xy_topRight[0], (int)xy_topRight[1]), 1, new Scalar(0,255,0), 3, 8, 0 );
                Imgproc.circle(image2_color, new org.opencv.core.Point((int)xy_topLeft[0], (int)xy_topLeft[1]), 1, new Scalar(0,255,0), 3, 8, 0 );
            }
        }
        //座標のLogを出力、
        Print_AR(corners, markerIds);

        //ハフ変換を使えば行けそう
        Mat circles = new Mat();
        /*
            HoughCircles(gray, circles, HOUGH_GRADIENT, dp, min_dist, param_1, param_2, min_radius, max_radius)
                gray: Input image (grayscale).
                circles: A vector that stores sets of 3 values: xc,yc,r for each detected circle.
                HOUGH_GRADIENT: Define the detection method. Currently this is the only one available in OpenCV.
                dp = 1: The inverse ratio of resolution.
                min_dist = gray.rows/16: Minimum distance between detected centers.
                param_1 = 200: Upper threshold for the internal Canny edge detector.
                param_2 = 100*: Threshold for center detection.
                min_radius = 0: Minimum radius to be detected. If unknown, put zero as default.
                max_radius = 0: Maximum radius to be detected. If unknown, put zero as default.
        */
        Imgproc.HoughCircles(image2, circles, Imgproc.HOUGH_GRADIENT, 1.0, image2.size().height/16, 100.0, 30.0, 10, 100);

        int[] changed_circle_pos = new int[2];
        //画像に検出した円を描画
        for(int x = 0; x < circles.cols(); x++){
            double[] c = circles.get(0, x);

            Log.i(TAG, "中心座標候補 x : "+ (int)Math.round(c[0]) +", y : "+ (int)Math.round(c[1]));
            //AR marker外の円を除去
            //x
            if((int)Math.round(c[0]) > (int)xy_topLeft[0] && (int)Math.round(c[0]) < (int)xy_bottomRight[0]){
                //y
                if((int)Math.round(c[1]) > (int)xy_topLeft[1] && (int)Math.round(c[1]) < (int)xy_bottomRight[1]){
                    changed_circle_pos[0] = (int)Math.round(c[0]);
                    changed_circle_pos[1] = (int)Math.round(c[1]);
                    org.opencv.core.Point center = new org.opencv.core.Point(Math.round(c[0]), Math.round(c[1]));
                    // circle center
                    // center color = Red
                    Imgproc.circle(image2_color, center, 1, new Scalar(255,0,0), 3, 8, 0 );
                    // circle outline
                    int radius = (int) Math.round(c[2]);
                    // circle colors = Green
                    Imgproc.circle(image2_color, center, radius, new Scalar(0,255,0), 3, 8, 0 );
                }
            }
        }

        final int[] circle_deviation = {3, -11};
        final int[] fixed_circle_pos = {((int)xy_topLeft[0]+(int)xy_bottomRight[0])/2+circle_deviation[0],
                                        ((int)xy_topLeft[1]+(int)xy_bottomRight[1])/2+circle_deviation[1]};

        // Fixed の Target2 を画像上に表示
        org.opencv.core.Point center_fixed = new org.opencv.core.Point(
                fixed_circle_pos[0],
                fixed_circle_pos[1]);
        // fixed center's color =  blue
        Imgproc.circle(image2_color, center_fixed, 1, new Scalar(0,255,255), 1, 8, 0 );
        Imgproc.circle(image2_color, center_fixed, 30, new Scalar(0,255,255), 3, 8, 0 );




        //レーザ位置修正(相対移動)
        Log.i(TAG, "レーザ位置修正移動");
        double[] fix_laser_pos = new double[2];
        double[] AR12_bottomright = new double[2];
        double[] AR12_center = new double[2];
        AR12_bottomright = corners.get(2).get(0,2);
        AR12_center[0] = ((int)xy_topLeft[0]+(int)AR12_bottomright[0])/2;
        AR12_center[1] = ((int)xy_topLeft[1]+(int)AR12_bottomright[1])/2;
        //debug
        Log.i(TAG, "AR12 中心座標 x : "+ (int)AR12_center[0] +", y : "+ (int)AR12_center[1]);
        Imgproc.circle(image2_color, new org.opencv.core.Point((int)AR12_center[0], (int)AR12_center[1]), 2, new Scalar(255,0,255), 3, 8, 0 );
        final double distance_per_pixel = 0.1125/(fixed_circle_pos[0]-AR12_center[0]); //[m/pix]

        fix_laser_pos[0] = (fixed_circle_pos[0] - changed_circle_pos[0])*distance_per_pixel;
        fix_laser_pos[1] = (fixed_circle_pos[1] - changed_circle_pos[1])*distance_per_pixel;
        Log.i(TAG, "修正量 : "+ fix_laser_pos[0] +", y : "+ fix_laser_pos[1]);

        relativeMoveToWrapper(fix_laser_pos[0], 0, fix_laser_pos[1], 0, 0, -0.707, 0.707);

        //画像の保存

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

        // send mission completion
        api.reportMissionCompletion();
        api.saveMatImage(image2,"image2.png");
        Aruco.drawDetectedMarkers(image2_color, corners, markerIds);
        api.saveMatImage(image2_color, "image2_color.png");
        api.saveMatImage(image2, "image2_gray.png");
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

        final int LOOP_MAX = 10;

        int count = 0;
        while(count < LOOP_MAX){
            final Point point = new Point(name.posX + name.avoidX*count, name.posY + name.avoidY*count, name.posZ + name.avoidZ*count);
            final Quaternion quaternion = new Quaternion((float)name.quaX, (float)name.quaY, (float)name.quaZ, (float)name.quaW);
            
            Result result = api.moveTo(point, quaternion, true);
            ++count;

            if(result.hasSucceeded()){
                break;
            }
            Log.i(TAG, "move Failure, retry");
        }
    }

    private void Print_AR(List<Mat> corners, Mat markerIds) {
        for (int n = 0; n < 4; n++) {
            Log.i(TAG, "markerIds:" + Arrays.toString(markerIds.get(n,0)));
            Log.i(TAG, "左上:" + Arrays.toString(corners.get(n).get(0, 0)));
            Log.i(TAG, "右上:" + Arrays.toString(corners.get(n).get(0, 1)));
            Log.i(TAG, "右下:" + Arrays.toString(corners.get(n).get(0, 2)));
            Log.i(TAG, "左下:" + Arrays.toString(corners.get(n).get(0, 3)));
        }
    }


    //右下のマーカを見つける
    private int findBottomRight(List<Mat> corners){
        Log.i(TAG,"start findBottomRight");
        // out = 関数のreturn
        int out = 0;
        int temp = 0;

        //corners.get(n).get(0, 0) -> n番目のマーカの右下のxy座標を取得
        for(int n=0; n<4; n++){
            Log.i(TAG,"Loop" + n );
            // 三平方の定理で一番数字が大きいものは遠いことを用いる
            // a^2 + b^2 = c^2
            double[] ab = corners.get(n).get(0,2);
            int c = (int)ab[0] * (int)ab[0] + (int)ab[1] * (int)ab[1];
            if(temp < c ){
                temp = c;
                out = n;
                Log.i(TAG,"change");
            }
        }
        // 右下（一番遠い）のは配列の何番目かをreturn
        Log.i(TAG,"finish findBottomRight");
        return out;
    }

}