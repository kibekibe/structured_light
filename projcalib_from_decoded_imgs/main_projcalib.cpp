/* This is sample from the OpenCV book. The copyright notice is below */

/* *************** License:**************************
Oct. 3, 2008
Right to use this code in any way you want without warranty, support or any guarantee of it working.

BOOK: It would be nice if you cited it:
Learning OpenCV: Computer Vision with the OpenCV Library
by Gary Bradski and Adrian Kaehler
Published by O'Reilly Media, October 3, 2008

AVAILABLE AT:
http://www.amazon.com/Learning-OpenCV-Computer-Vision-Library/dp/0596516134
Or: http://oreilly.com/catalog/9780596516130/
ISBN-10: 0596516134 or: ISBN-13: 978-0596516130

OPENCV WEBSITES:
Homepage:      http://opencv.org
Online docs:   http://docs.opencv.org
Q&A forum:     http://answers.opencv.org
Issue tracker: http://code.opencv.org
GitHub:        https://github.com/Itseez/opencv/
************************************************** */

#include "opencv2/opencv.hpp"
#include <fstream>
#include <time.h>

using namespace cv;
using namespace std;

const char* keys =
{
	"{calib_mode  |       | Mode of calibration; 'proj' means calibration only projector, and 'procam' means stereo calibration between a camera and a projector}"
	"{w           |       | Board width          }"
	"{h           |       | Board height          }"
	"{s           | 1.0   | Square size          }"
	"{imglist     |       | Image list file name  }"
	"{xlist       | xlist.txt   | x          }"
	"{ylist       | ylist.txt   | y          }"
	"{proj_sw     | 1.0   | Image width of projector         }"
	"{proj_sh     | 1.0   | Image height of projector        }"
};


enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };



static std::vector<std::string> getStringList(const std::string& filename)
{
	std::vector<std::string> strArray;

	ifstream ifs(filename.c_str());
	string tmp;
	while (ifs && getline(ifs, tmp)) {
		strArray.push_back(tmp);
	}

	return strArray;
}


static double computeReprojectionErrors(
	const vector<vector<Point3f> >& objectPoints,
	const vector<vector<Point2f> >& imagePoints,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const Mat& cameraMatrix, const Mat& distCoeffs,
	vector<float>& perViewErrors)
{
	vector<Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); i++)
	{
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i],
			cameraMatrix, distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)std::sqrt(err*err / n);
		totalErr += err*err;
		totalPoints += n;
	}

	return std::sqrt(totalErr / totalPoints);
}

static void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType = CHESSBOARD)
{
	corners.resize(0);

	switch (patternType)
	{
	case CHESSBOARD:
	case CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; i++)
			for (int j = 0; j < boardSize.width; j++)
				corners.push_back(Point3f(float(j*squareSize),
				float(i*squareSize), 0));
		break;

	case ASYMMETRIC_CIRCLES_GRID:
		for (int i = 0; i < boardSize.height; i++)
			for (int j = 0; j < boardSize.width; j++)
				corners.push_back(Point3f(float((2 * j + i % 2)*squareSize),
				float(i*squareSize), 0));
		break;

	default:
		CV_Error(Error::StsBadArg, "Unknown pattern type\n");
	}
}

static bool runCalibration(vector<vector<Point2f> > imagePoints,
	Size imageSize, Size boardSize, Pattern patternType,
	float squareSize, float aspectRatio,
	int flags, Mat& cameraMatrix, Mat& distCoeffs,
	vector<Mat>& rvecs, vector<Mat>& tvecs,
	vector<float>& reprojErrs,
	double& totalAvgErr)
{
	cameraMatrix = Mat::eye(3, 3, CV_64F);
	if (flags & CALIB_FIX_ASPECT_RATIO)
		cameraMatrix.at<double>(0, 0) = aspectRatio;

	distCoeffs = Mat::zeros(8, 1, CV_64F);

	vector<vector<Point3f> > objectPoints(1);
	calcChessboardCorners(boardSize, squareSize, objectPoints[0], patternType);

	objectPoints.resize(imagePoints.size(), objectPoints[0]);

	double rms = calibrateCamera(objectPoints, imagePoints, imageSize, cameraMatrix,
		distCoeffs, rvecs, tvecs, flags | CALIB_FIX_K4 | CALIB_FIX_K5);
	///*|CALIB_FIX_K3*/|CALIB_FIX_K4|CALIB_FIX_K5);
	printf("RMS error reported by calibrateCamera: %g\n", rms);

	bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);

	totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
		rvecs, tvecs, cameraMatrix, distCoeffs, reprojErrs);

	return ok;
}


static void
StereoCalib(Size boardSize, Pattern patternType, Size imageSize,
float squareSize, const vector<vector<vector<Point2f>>>& imagePoints )
{	const int maxScale = 2;
	//const float squareSize = 1.f;  // Set this to your actual square size
	// ARRAY AND VECTOR STORAGE:

	//vector<vector<Point2f> > imagePoints[2];
	vector<vector<Point3f> > objectPoints;
	

	size_t i, j, k, nimages = imagePoints[0].size();

	objectPoints.resize(nimages);

	for (i = 0; i < nimages; i++)
	{
		calcChessboardCorners(boardSize, squareSize, objectPoints[i], patternType);
		//for (j = 0; j < boardSize.height; j++)
		//	for (k = 0; k < boardSize.width; k++)
		//		objectPoints[i].push_back(Point3f(k*squareSize, j*squareSize, 0));
	}

	cout << "Running stereo calibration ...\n";

	Mat cameraMatrix[2], distCoeffs[2];
	cameraMatrix[0] = Mat::eye(3, 3, CV_64F);
	cameraMatrix[1] = Mat::eye(3, 3, CV_64F);
	Mat R, T, E, F;

	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, E, F,
		CALIB_FIX_ASPECT_RATIO +
		CALIB_ZERO_TANGENT_DIST +
		//CALIB_SAME_FOCAL_LENGTH +
		CALIB_RATIONAL_MODEL +
		CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
	cout << "done with RMS error=" << rms << endl;

	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];
	for (i = 0; i < nimages; i++)
	{
		int npt = (int)imagePoints[0][i].size();
		Mat imgpt[2];
		for (k = 0; k < 2; k++)
		{
			imgpt[k] = Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], int(k + 1), F, lines[k]);
		}
		for (j = 0; j < npt; j++)
		{
			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
				imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x*lines[0][j][0] +
				imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	cout << "average reprojection err = " << err / npoints << endl;

	// save intrinsic parameters
	FileStorage fs("intrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";

	Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];

	stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

	fs.open("extrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
		cout << "Error: can not save the extrinsic parameters\n";


}




static void
StereoCalib(//Mat1d& Ac, Mat1d& Dc, Mat1d& Ap, Mat1d& Dp, 
			Mat& Ac, Mat& Dc, Mat& Ap, Mat& Dp,
			Size boardSize, Pattern patternType, Size imageSize,
			float squareSize, const vector<vector<vector<Point2f>>>& imagePoints)
{
	//vector<vector<Point2f> > imagePoints[2];
	vector<vector<Point3f> > objectPoints;

	size_t i, j, k, nimages = imagePoints[0].size();

	objectPoints.resize(nimages);

	for (i = 0; i < nimages; i++)
	{
		calcChessboardCorners(boardSize, squareSize, objectPoints[i], patternType);
	}

	cout << "Running stereo calibration ...\n";

	Mat cameraMatrix[2], distCoeffs[2];
	cameraMatrix[0] = Ac;
	cameraMatrix[1] = Ap;
	distCoeffs[0] = Dc;
	distCoeffs[1] = Dp;

	Mat R, T, E, F;

	double rms = stereoCalibrate(objectPoints, imagePoints[0], imagePoints[1],
		cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, E, F,
		//CALIB_FIX_ASPECT_RATIO +
		CALIB_ZERO_TANGENT_DIST +
		CALIB_FIX_INTRINSIC,
		//CALIB_SAME_FOCAL_LENGTH +
		//CALIB_RATIONAL_MODEL +
		//CALIB_FIX_K3 + CALIB_FIX_K4 + CALIB_FIX_K5,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
	cout << "done with RMS error=" << rms << endl;

	// CALIBRATION QUALITY CHECK
	// because the output fundamental matrix implicitly
	// includes all the output information,
	// we can check the quality of calibration using the
	// epipolar geometry constraint: m2^t*F*m1=0
	double err = 0;
	int npoints = 0;
	vector<Vec3f> lines[2];
	for (i = 0; i < nimages; i++)
	{
		int npt = (int)imagePoints[0][i].size();
		Mat imgpt[2];
		for (k = 0; k < 2; k++)
		{
			imgpt[k] = Mat(imagePoints[k][i]);
			undistortPoints(imgpt[k], imgpt[k], cameraMatrix[k], distCoeffs[k], Mat(), cameraMatrix[k]);
			computeCorrespondEpilines(imgpt[k], int(k + 1), F, lines[k]);
		}
		for (j = 0; j < npt; j++)
		{
			double errij = fabs(imagePoints[0][i][j].x*lines[1][j][0] +
				imagePoints[0][i][j].y*lines[1][j][1] + lines[1][j][2]) +
				fabs(imagePoints[1][i][j].x*lines[0][j][0] +
				imagePoints[1][i][j].y*lines[0][j][1] + lines[0][j][2]);
			err += errij;
		}
		npoints += npt;
	}
	cout << "average reprojection err = " << err / npoints << endl;

	// save intrinsic parameters
	FileStorage fs("intrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << cameraMatrix[0] << "D1" << distCoeffs[0] <<
			"M2" << cameraMatrix[1] << "D2" << distCoeffs[1];
		fs.release();
	}
	else
		cout << "Error: can not save the intrinsic parameters\n";

	Mat R1, R2, P1, P2, Q;
	Rect validRoi[2];

	stereoRectify(cameraMatrix[0], distCoeffs[0],
		cameraMatrix[1], distCoeffs[1],
		imageSize, R, T, R1, R2, P1, P2, Q,
		CALIB_ZERO_DISPARITY, 1, imageSize, &validRoi[0], &validRoi[1]);

	fs.open("extrinsics.yml", FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "R" << R << "T" << T << "R1" << R1 << "R2" << R2 << "P1" << P1 << "P2" << P2 << "Q" << Q;
		fs.release();
	}
	else
		cout << "Error: can not save the extrinsic parameters\n";


}


static void saveCameraParams(const string& filename,
	Size imageSize, Size boardSize,
	float squareSize, float aspectRatio, int flags,
	const Mat& cameraMatrix, const Mat& distCoeffs,
	const vector<Mat>& rvecs, const vector<Mat>& tvecs,
	const vector<float>& reprojErrs,
	const vector<vector<Point2f> >& imagePoints,
	double totalAvgErr)
{
	FileStorage fs(filename, FileStorage::WRITE);

	time_t tt;
	time(&tt);
	struct tm *t2 = localtime(&tt);
	char buf[1024];
	strftime(buf, sizeof(buf) - 1, "%c", t2);

	fs << "calibration_time" << buf;

	if (!rvecs.empty() || !reprojErrs.empty())
		fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
	fs << "image_width" << imageSize.width;
	fs << "image_height" << imageSize.height;
	fs << "board_width" << boardSize.width;
	fs << "board_height" << boardSize.height;
	fs << "square_size" << squareSize;

	if (flags & CALIB_FIX_ASPECT_RATIO)
		fs << "aspectRatio" << aspectRatio;

	if (flags != 0)
	{
		sprintf(buf, "flags: %s%s%s%s",
			flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
			flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
			flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
			flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
		//cvWriteComment( *fs, buf, 0 );
	}

	fs << "flags" << flags;

	fs << "camera_matrix" << cameraMatrix;
	fs << "distortion_coefficients" << distCoeffs;

	fs << "avg_reprojection_error" << totalAvgErr;
	if (!reprojErrs.empty())
		fs << "per_view_reprojection_errors" << Mat(reprojErrs);

	if (!rvecs.empty() && !tvecs.empty())
	{
		CV_Assert(rvecs[0].type() == tvecs[0].type());
		Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
		for (int i = 0; i < (int)rvecs.size(); i++)
		{
			Mat r = bigmat(Range(i, i + 1), Range(0, 3));
			Mat t = bigmat(Range(i, i + 1), Range(3, 6));

			CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
			CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
			//*.t() is MatExpr (not Mat) so we can use assignment operator
			r = rvecs[i].t();
			t = tvecs[i].t();
		}
		//cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
		fs << "extrinsic_parameters" << bigmat;
	}

	if (!imagePoints.empty())
	{
		Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
		for (int i = 0; i < (int)imagePoints.size(); i++)
		{
			Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
			Mat imgpti(imagePoints[i]);
			imgpti.copyTo(r);
		}
		fs << "image_points" << imagePtMat;
	}
}

static bool readStringList(const string& filename, vector<string>& l)
{
	l.resize(0);
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	FileNode n = fs.getFirstTopLevelNode();
	if (n.type() != FileNode::SEQ)
		return false;
	FileNodeIterator it = n.begin(), it_end = n.end();
	for (; it != it_end; ++it)
		l.push_back((string)*it);
	return true;
}


static bool runAndSave(const string& outputFilename,
	const vector<vector<Point2f> >& imagePoints,
	Size imageSize, Size boardSize, Pattern patternType, float squareSize,
	float aspectRatio, int flags, Mat& cameraMatrix,
	Mat& distCoeffs, bool writeExtrinsics, bool writePoints)
{
	vector<Mat> rvecs, tvecs;
	vector<float> reprojErrs;
	double totalAvgErr = 0;

	bool ok = runCalibration(imagePoints, imageSize, boardSize, patternType, squareSize,
		aspectRatio, flags, cameraMatrix, distCoeffs,
		rvecs, tvecs, reprojErrs, totalAvgErr);
	printf("%s. avg reprojection error = %.2f\n",
		ok ? "Calibration succeeded" : "Calibration failed",
		totalAvgErr);

	if (ok)
		saveCameraParams(outputFilename, imageSize,
		boardSize, squareSize, aspectRatio,
		flags, cameraMatrix, distCoeffs,
		writeExtrinsics ? rvecs : vector<Mat>(),
		writeExtrinsics ? tvecs : vector<Mat>(),
		writeExtrinsics ? reprojErrs : vector<float>(),
		writePoints ? imagePoints : vector<vector<Point2f> >(),
		totalAvgErr);
	return ok;
}


static vector<Point2f> getProjPoints(const vector<Point2f>& camImagePoints,
									 const Mat1f& xMap,
									 const Mat1f& yMap,
									 float        xScale,
									 float        yScale)
{
	vector<Point2f> projPoints(camImagePoints.size());

	//	
	for (size_t i = 0; i < camImagePoints.size(); ++i) {
		// バイリニア
		//
		int px = int(camImagePoints[i].x);
		int py = int(camImagePoints[i].y);			

		Point2f left_top(xMap(py, px), yMap(py, px));
		Point2f left_bottom(xMap(py+1, px), yMap(py+1, px));
		Point2f right_top(xMap(py, px+1), yMap(py, px+1));
		Point2f right_bottom(xMap(py+1, px+1), yMap(py+1, px+1));

		float tx = camImagePoints[i].x - float(px);			
		Point2f top   = (1.0f - tx) * left_top + tx * right_top;
		Point2f bottom = (1.0f - tx) * left_bottom + tx * right_bottom;

		float ty = camImagePoints[i].y - float(py);
		projPoints[i] = (1.0f - ty) * top + ty * bottom;
		projPoints[i].x *= xScale;
		projPoints[i].y *= yScale;
	}	

	return projPoints;
}

int main(int argc, char* argv[])
{
	CommandLineParser parser(argc, argv, keys);


	Size boardSize;
	boardSize.width  = parser.get<int>("w");
	boardSize.height = parser.get<int>("h");

	Size camSize, projPatternSize(120,120), projRealSize(512,384);
	Size subPixelBlockSize = Size(11, 11);
	float squareSize = parser.get<float>("s");
	float aspectRatio = 1.f;
	
	const char* outputCamFileName  = "camera_data.yml";
	const char* outputProjFileName = "proj_data.yml";
	const char* outputRtFileName   = "rt_data.yml";

	
	bool writeExtrinsics = false, writePoints = false;
	
	int flags = 0;
	
	
	vector<string> imageList = getStringList(parser.get<string>("imglist"));
	vector<string> xList = getStringList(parser.get<string>("xlist"));
	vector<string> yList = getStringList(parser.get<string>("ylist"));
	Pattern pattern = CHESSBOARD;

	float xScale = (float)projRealSize.width / (float)projPatternSize.width;//parser.get<float>("proj_sh");
	float yScale = (float)projRealSize.height / (float)projPatternSize.height;//parser.get<float>("proj_sw");
	

	vector<vector<Point2f>> projPoints, camPoints;

	for (size_t i = 0; i < imageList.size(); ++i) {
		Mat3b view = imread(imageList[i], 1);

		if (i == 0) {
			camSize = view.size();
			subPixelBlockSize.width = camSize.width / 100;
			subPixelBlockSize.height = camSize.height / 100;
		}

		vector<Point2f> camPointbuf;

		Mat1b viewGray;
		cvtColor(view, viewGray, COLOR_BGR2GRAY);

		bool found;
		switch (pattern)
		{
		case CHESSBOARD:
			//found = findChessboardCorners(view, boardSize, camPointbuf,
				//CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
			found = findChessboardCorners(viewGray, boardSize, camPointbuf);
				///*CALIB_CB_ADAPTIVE_THRESH |*/ CALIB_CB_FAST_CHECK | CALIB_CB_NORMALIZE_IMAGE);
			break;
		case CIRCLES_GRID:
			found = findCirclesGrid(view, boardSize, camPointbuf);
			break;
		case ASYMMETRIC_CIRCLES_GRID:
			found = findCirclesGrid(view, boardSize, camPointbuf, CALIB_CB_ASYMMETRIC_GRID);
			break;
		default:
			return fprintf(stderr, "Unknown pattern type\n"), -1;
		}


		// improve the found corners' coordinate accuracy
		if (pattern == CHESSBOARD && found) cornerSubPix(viewGray, camPointbuf, subPixelBlockSize,
			Size(-1, -1), TermCriteria(TermCriteria::EPS + TermCriteria::COUNT, 30, 0.1));

		if (found) {
			Mat1f xMap = imread(xList[i], -1);
			Mat1f yMap = imread(yList[i], -1);
			vector<Point2f> projPointbuf = getProjPoints(camPointbuf, xMap, yMap, xScale, yScale);
			projPoints.push_back(projPointbuf);
			camPoints.push_back(camPointbuf);
		}

	}

	Mat cameraMatrix, camDistCoeffs;
	Mat projMatrix, projDistCoeffs;

	const string calib_mode = parser.get<string>("calib_mode");
	if (calib_mode == "proj") {
		runAndSave(outputCamFileName, projPoints, projRealSize,
			boardSize, pattern, squareSize, aspectRatio,
			flags, projMatrix, camDistCoeffs,
			writeExtrinsics, writePoints);
	}
	else if (calib_mode == "procam") {
		vector<vector<vector<Point2f>>> imagePoints(2);		
		imagePoints[0] = camPoints;
		imagePoints[1] = projPoints;
#if 1
		runAndSave(outputCamFileName, camPoints, camSize, boardSize, pattern, squareSize, aspectRatio, flags, cameraMatrix, camDistCoeffs, writeExtrinsics, writePoints);
		runAndSave(outputProjFileName, projPoints, projRealSize, boardSize, pattern, squareSize, aspectRatio, flags, projMatrix, projDistCoeffs, writeExtrinsics, writePoints);
		StereoCalib(cameraMatrix, camDistCoeffs, projMatrix, projDistCoeffs, boardSize, pattern, camSize, squareSize, imagePoints);
#else
		StereoCalib(boardSize, pattern, camSize, squareSize, imagePoints);
#endif
	}
	return 0;
}