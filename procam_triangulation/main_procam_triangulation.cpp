#include "opencv2/opencv.hpp"
#include <fstream>

using namespace cv;
using namespace std;

const char* keys =
{
	"{xMap   |x.exr             |x decoded image filename         }"
	"{yMap   |y.exr             |y decoded image filename         }"
	"{mask   |mask.png          |         }"
	"{cam    |camera_data.yml   |camera parameter         }"
	"{proj   |proj_data.yml     |projector parameter         }"
	"{proj_h |384               |image width of the projector         }"
	"{proj_w |512               |image height of the projector         }"
	"{rt     |extrinsics.yml    |extrinsic parameter between the camera and the projector }"
	"{texture|                  |rgb image for reconstructed point cloud         }"
	"{output |reconstructed.ply |output point cloud file name         }"
};




void savePly(const string& fileName, const Mat1f& points4D, const Mat2i& locations, const Mat3b& rgb)
{
	ofstream ofs(fileName.c_str());
	ofs << "ply" << endl << "format ascii 1.0" << endl;

	ofs << "element vertex " << points4D.cols << endl;

	ofs << "property float x" << endl 
		<< "property float y" << endl 
		<< "property float z" << endl;

	ofs << "property uchar red" << endl
		<< "property uchar green" << endl 
		<< "property uchar blue" << endl;
	
	ofs << "end_header" << endl;

	for (int i = 0; i < points4D.cols; ++i)
	{
		Point p = locations(i, 0);
		ofs << points4D(0, i) / points4D(3, i) << " "
			<< points4D(1, i) / points4D(3, i) << " "
			<< points4D(2, i) / points4D(3, i) << " "
			<< (int)rgb(p.y, p.x)[2] << " "
			<< (int)rgb(p.y, p.x)[1] << " "
			<< (int)rgb(p.y, p.x)[0] << endl;
	}
}




Mat2f getProjPts(const Mat2i& locations, const Mat1f& xMap, const Mat1f& yMap)
{
	Mat2f dst(locations.size());
	for (int p = 0; p < locations.rows; ++p)
	{
		dst(p, 0)[0] = xMap(locations(p, 0)[1], locations(p, 0)[0]);
		dst(p, 0)[1] = yMap(locations(p, 0)[1], locations(p, 0)[0]);
	}
	return dst;
}

int main(int argc, char* argv[])
{
	CommandLineParser parser(argc, argv, keys);

	const string xMapFileName = parser.get<string>("xMap");
	const string yMapFileName = parser.get<string>("yMap");
	const string maskFileName = parser.get<string>("mask");

	const string camFileName  = parser.get<string>("cam");
	const string projFileName = parser.get<string>("proj");
	const string rtFileName   = parser.get<string>("rt");
	
	
	const Mat1f xMap = imread(xMapFileName, -1);
	const Mat1f yMap = imread(yMapFileName, -1);

	Mat1d Ac, Ap;
	Mat1d Dc, Dp;
	Mat1d R, T;

	// reading intrinsic parameters
	FileStorage fs_cam(camFileName, FileStorage::READ);
	if (!fs_cam.isOpened())
	{
		printf("Failed to open file %s\n", camFileName);
		return -1;
	}	
	fs_cam["camera_matrix"] >> Ac;
	fs_cam["distortion_coeffs"] >> Dc;

	FileStorage fs_proj(projFileName, FileStorage::READ);
	if (!fs_proj.isOpened())
	{
		printf("Failed to open file %s\n", projFileName);
		return -1;
	}
	fs_proj["camera_matrix"] >> Ap;
	fs_proj["distortion_coeffs"] >> Dp;

	FileStorage fs_rt(rtFileName, FileStorage::READ);
	if (!fs_rt.isOpened())
	{
		printf("Failed to open file %s\n", rtFileName);
		return -1;
	}

	
	fs_rt["R"] >> R;
	fs_rt["T"] >> T;

	Mat1d projMatr1 = Mat1d::zeros(3, 4);
	for (int j = 0; j < 3; ++j) {
		for (int i = 0; i < 3; ++i) {
			projMatr1(j, i) = Ac(j, i);
		}
	}

	cout << "projMatr1" << endl << projMatr1 << endl;

	Mat1d RT = Mat1d::zeros(3, 4);
	for (int j = 0; j < 3; ++j) {
		for (int i = 0; i < 3; ++i) {
			RT(j, i) = R(j, i);
		}
		RT(j, 3) = T(j, 0);
	}
	cout << "RT" << endl << RT << endl;
	Mat1d projMatr2 = Ap * RT;

	Mat2i projPoints1;

	Mat1b mask = imread(maskFileName, 0);
	cv::findNonZero(mask, projPoints1);
	Mat2f projPoints2 = getProjPts(projPoints1, xMap, yMap);

	Mat1f points4D;
	cv::triangulatePoints(projMatr1, projMatr2, (Mat2f)projPoints1, projPoints2, points4D);
		
	Mat3b rgb = imread(parser.get<string>("texture"));
	savePly(parser.get<string>("output"), points4D, projPoints1, rgb);
	return 0;
}