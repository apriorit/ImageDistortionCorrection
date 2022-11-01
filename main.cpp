#include <iostream>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

Ptr<aruco::CharucoBoard> createBoard()
{
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_100);
    Ptr<aruco::CharucoBoard> board = aruco::CharucoBoard::create(10, 10, 0.04f, 0.02f, dictionary);
    return board;
}

int main()
{
    Ptr<aruco::CharucoBoard> charucoboard = createBoard();
    Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();

    vector<vector<vector<Point2f>>> allCorners;
    vector<vector<int>> allIds;
    vector<Mat> allImages;
    Size imageSize;

    // collect data from each frame
    for (int i = 1; i <= 4; i++)
    {
        Mat image, imageCopy;
        image = imread("calibration_image_" + to_string(i) + ".jpg");

        vector<int> ids;
        vector<vector<Point2f>> corners, rejected;

        // detect markers
        aruco::detectMarkers(image, board->dictionary, corners, ids);

        // refind strategy to detect more markers
        if (true)
        {
            aruco::refineDetectedMarkers(image, board, corners, ids, rejected);
        }

        // interpolate charuco corners
        // interpolated corners will not be used, this code is example
        Mat currentCharucoCorners, currentCharucoIds;
        if (ids.size() > 0)
        {
            aruco::interpolateCornersCharuco(corners, ids, image, charucoboard, currentCharucoCorners, currentCharucoIds);
        }

        // draw results
        image.copyTo(imageCopy);
        if (ids.size() > 0)
        {
            aruco::drawDetectedMarkers(imageCopy, corners);
        }
        if (currentCharucoCorners.total() > 0)
        {
            aruco::drawDetectedCornersCharuco(imageCopy, currentCharucoCorners, currentCharucoIds);
        }
        imshow("Detected markers" + std::to_string(i), imageCopy);

        allCorners.push_back(corners);
        allIds.push_back(ids);
        allImages.push_back(image);

        imageSize = image.size();
    }

    if (allIds.size() < 1)
    {
        cerr << "Not enough captures for calibration" << endl;
        return 0;
    }

    Mat cameraMatrix, distCoeffs;
    vector<Mat> rvecs, tvecs;
    double repError;
    int calibrationFlags = 0;

    // prepare data for calibration
    vector<vector<Point2f>> allCornersConcatenated;
    vector<int> allIdsConcatenated;
    vector<int> markerCounterPerFrame;
    markerCounterPerFrame.reserve(allCorners.size());
    for (unsigned int i = 0; i < allCorners.size(); i++)
    {
        markerCounterPerFrame.push_back((int)allCorners[i].size());
        for (unsigned int j = 0; j < allCorners[i].size(); j++)
        {
            allCornersConcatenated.push_back(allCorners[i][j]);
            allIdsConcatenated.push_back(allIds[i][j]);
        }
    }

    // calibrate camera using aruco markers
    double arucoRepErr;
    arucoRepErr = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
                                              markerCounterPerFrame, board, imageSize, cameraMatrix,
                                              distCoeffs, noArray(), noArray(), calibrationFlags);

    // Interpolate charuco markers with aruco calibration results usage
    vector<vector<Point2f>> allCharucoCorners;
    vector<vector<int>> allCharucoIds;
    int nFrames = (int)allCorners.size();
    for (int i = 0; i < nFrames; i++)
    {
        // interpolate using camera parameters
        Mat currentCharucoCorners, currentCharucoIds;
        aruco::interpolateCornersCharuco(allCorners[i], allIds[i], allImages[i], charucoboard,
                                         currentCharucoCorners, currentCharucoIds, cameraMatrix,
                                         distCoeffs);

        allCharucoCorners.push_back(currentCharucoCorners);
        allCharucoIds.push_back(currentCharucoIds);
    }

    if (allCharucoCorners.size() < 4)
    {
        cerr << "Not enough corners for calibration " << allCharucoCorners.size() << endl;
        return 0;
    }

    // calibrate camera using charuco
    repError = aruco::calibrateCameraCharuco(allCharucoCorners, allCharucoIds, charucoboard, imageSize,
                                             cameraMatrix, distCoeffs, rvecs, tvecs, calibrationFlags);

    cout << "Rep Error Aruco: " << arucoRepErr << endl;
    cout << "Rep Error Charuco: " << repError << endl;

    Mat image = imread("image_for_correction.jpg");
    Mat finalImage;
    imshow("Before correction", image);
    undistort(image, finalImage, cameraMatrix, distCoeffs);
    imshow("After correction", finalImage);

    waitKey(0);
    return 0;
}
