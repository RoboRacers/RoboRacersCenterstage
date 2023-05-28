/*
 * Copyright (c) 2020 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.modules.opencv;

import static java.lang.Thread.sleep;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

// ToDo: Get localisation information for coneDetection to work in desired location
//  and avoid detecting cone on the poll.
/*
 * This is an advanced sample showcasing detecting and determining the orientation
 * of multiple Cones, switching the viewport output, and communicating the results
 * of the vision processing to user-code.
 */
public class ConeDetection
{
    ConeDetectionPipeline ConeDetectionpipeline;

    public final int RED_CONE  = 1;
    public final int BLUE_CONE = 2;

    public ConeDetection(OpenCvCamera camera) {


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()  {
            @Override
            public void onOpened()  {
                camera.startStreaming(1184,656, OpenCvCameraRotation.UPRIGHT);
                ConeDetectionpipeline = new ConeDetectionPipeline(BLUE_CONE);
                camera.setPipeline(ConeDetectionpipeline);
            }

            @Override
            public void onError(int errorCode)  {    }
        });

        while (ConeDetectionpipeline == null)
        {}
    }



    public double getCenter()
    {
        return ConeDetectionpipeline.getCenter();
    }

    static class ConeDetectionPipeline extends OpenCvPipeline
    {
        int cone_type=-1;
        double centerX = 0.0;
        Mat imgHSV = new Mat();

        Mat imgRGB = new Mat();
        Mat imgGray = new Mat();
        Mat edges = new Mat();
        Mat hierarchy = new Mat();

        Mat red_mask1 = new Mat();
        Mat red_mask2 = new Mat();

        Scalar red_low1 = new Scalar(0, 70, 0);
        Scalar red_high1 = new Scalar(10, 255, 255);

        Scalar red_low2 = new Scalar(170, 70, 50);
        Scalar red_high2 = new Scalar(180, 255, 255);

        Scalar blue_lowHSV = new Scalar(110, 50, 50); // lower bound HSV for blue
        Scalar blue_highHSV = new Scalar(130, 255, 255); // higher bound HSV for blue

        public ConeDetectionPipeline(int cone_type)
        {
            cone_type=cone_type;
        }

        @Override
        public Mat processFrame(Mat frame)
        {

            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();

            cone_type = 1;

            Imgproc.blur(frame, frame, new Size(39,39));

            Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5,5));

            Imgproc.erode(frame, frame, element);

            Imgproc.cvtColor(frame, imgHSV, Imgproc.COLOR_RGB2HSV);

            //ToDo: Create different april Tag for RED and BLUE cones
            if(cone_type==1)       {
                Core.inRange(imgHSV, red_low1, red_high1, red_mask1);
                Core.inRange(imgHSV, red_low2, red_high2, red_mask2);
                //ToDO: Do we need this function for just ORing two maskes
                Core.addWeighted(red_mask1, 1.0, red_mask2, 1.0, 0.0, imgGray);
            }
            else if (cone_type==2) {
                Core.inRange(imgHSV, blue_lowHSV, blue_highHSV, imgGray);
            }

            Imgproc.Canny(imgGray, edges, 100, 250);

            Imgproc.findContours(edges, contours, hierarchy, Imgproc.RETR_EXTERNAL,
                                 Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint2f[] contoursPoly  = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];

            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                MatOfPoint2f cPoly = new MatOfPoint2f(contours.get(i).toArray());
                Imgproc.approxPolyDP(cPoly, contoursPoly[i], 3, true);

                if (Imgproc.contourArea(cPoly) > 50 ) {
                    boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
                    Imgproc.rectangle(frame, boundRect[i], new Scalar(0.5, 76.9, 89.8));

                    Integer x2 = boundRect[i].x + (int) boundRect[i].height / 2;
                    Integer y2 = boundRect[i].y + (int) boundRect[i].width / 2;

                    centerX = x2;

                    String text = "x: " + x2.toString() + ", y: " + y2.toString();

                    Imgproc.putText(frame, text, new Point(x2 - 10, y2 - 10),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5,
                            new Scalar(0, 255, 0), 2);
                }
            }

            return frame;
        }

        public double getCenter()  { return centerX; }

    }
}