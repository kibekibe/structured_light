/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2015, OpenCV Foundation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include "opencv2/opencv.hpp"
#include <opencv2/structured_light.hpp>
#include "opencv2/structured_light/graycodepattern.hpp"
#include <fstream>

using namespace cv;
using namespace std;

 const char* keys =
{
"{@images_list     |         | Image list where the captured pattern images are saved}"
"{@proj_width      |         | The projector width used to acquire the pattern          }"
"{@proj_height     |         | The projector height used to acquire the pattern}"
"{calib_param_path |         | Calibration_parameters          }"
"{mask             |mask.png | Output mask image filename      }"
"{x_png            |x.png    | X decoded image filename (8bit) }"
"{y_png            |y.png    | Y decoded image filename (8bit) }"
"{x_exr            |x.exr    | X decoded image filename (float)}"
"{y_exr            |y.exr    | Y decoded image filename (float)}"
"{white_thresh     |5        | The white threshold (optional)}"
"{black_thresh     |40       | The black threshold (optional)}" 
};



Mat1b computeShadowMask(const Mat1b& blackImage, const Mat1b& whiteImage, size_t threshold)
{
	Mat1b shadowMask = Mat1b::zeros(blackImage.size());
	for (int j = 0; j < shadowMask.rows; ++j) {
		for (int i = 0; i < shadowMask.cols; ++i) {			
			if (whiteImage(j, i) > blackImage(j, i) + threshold){
				shadowMask(j, i) = 255;
			}
		}
	}
	return shadowMask;
}

Mat2f computeDecodeImage(Ptr<structured_light::GrayCodePattern>& graycode, const vector<Mat1b>& captured_pattern, const Mat1b& mask)
{
	Mat2f decodedImage = Mat2f::zeros(mask.size());
	for (int j = 0; j < decodedImage.rows; ++j) {
		for (int i = 0; i < decodedImage.cols; ++i) {
			if (mask(j, i) == 0){
				continue;
			}
			Point projPixel;
			bool error = graycode->getProjPixel(captured_pattern, i, j, projPixel);
			if (error){
				continue;
			}
			decodedImage(j, i)[0] = (float)projPixel.x;
			decodedImage(j, i)[1] = (float)projPixel.y;
		}
	}
	return decodedImage;
}

vector<string> getStringList(const string& filename)
{
	vector<string> strList;
	ifstream ifs(filename.c_str());
	string tmp;
	while(ifs && getline(ifs, tmp)){
		strList.push_back(tmp);
	}
	return strList;
}

vector<Mat1b> getImags(const vector<string>& filenames)
{
	vector<Mat1b> imgs(filenames.size());
	for (size_t i = 0; i < imgs.size(); ++i){
		imgs[i] = imread(filenames[i], IMREAD_GRAYSCALE);
	}
	return imgs;
}

void vizDecodedImage(const Mat2f& decoded, int projWidth, int projHeight, const string& strX = "x.png", const string& strY = "y.png")
{
	Mat1b xMap = Mat1b::zeros(decoded.size());
	Mat1b yMap = Mat1b::zeros(decoded.size());
	for (int j = 0; j < decoded.rows; ++j) {
		for (int i = 0; i < decoded.cols; ++i) {
			if (decoded(j, i)[0] == 0.0f){
				continue;
			}
			int corresX = int(decoded(j, i)[0] * 255.0f / (float)projWidth);
			int corresY = int(decoded(j, i)[1] * 255.0f / (float)projHeight);
			xMap(j, i) = corresX;
			yMap(j, i) = corresY;
		}
	}
	imwrite(strX, xMap);
	imwrite(strY, yMap);
}

Mat1b getDecoedMask(const Mat2f& decoded)
{
	Mat1b mask = Mat1b::zeros(decoded.size());
	for (int j = 0; j < decoded.rows; ++j) {
		for (int i = 0; i < decoded.cols; ++i) {
			if (decoded(j, i)[0] == 0.0f){
				continue;
			}
			mask(j, i) = 255;
		}
	}
	return mask;
}

void saveDecodedImage(const Mat2f& decoded, const string& strX = "x.exr", const string& strY = "y.exr")
{
	vector<Mat1f> tmp(2);
	split(decoded, tmp);
	imwrite(strX, tmp[0]);
	imwrite(strY, tmp[1]);
}

int main(int argc, char* argv[])
{
	
	CommandLineParser parser(argc, argv, keys);


	structured_light::GrayCodePattern::Params params;
	string images_file = parser.get<string>(0);
	params.width       = parser.get<int>(1);
	params.height      = parser.get<int>(2);

	
	// Set up GraycodePattern with params
	Ptr<structured_light::GrayCodePattern> graycode = structured_light::GrayCodePattern::create(params);
	size_t white_thresh = parser.get<int>("white_thresh");
	size_t black_thresh = parser.get<int>("black_thresh");

	graycode->setWhiteThreshold(white_thresh);

	size_t num_pattern = graycode->getNumberOfPatternImages();

	
	const vector<string> image_list    = getStringList(images_file);

	const Mat1b white_image      = imread(image_list[num_pattern],     IMREAD_GRAYSCALE);
	const Mat1b black_image      = imread(image_list[num_pattern + 1], IMREAD_GRAYSCALE);
	const Mat1b shadow_mask      = computeShadowMask(black_image, white_image, black_thresh);


	const vector<Mat1b> captured_pattern = getImags(image_list);

	cout << endl << "Decoding pattern ..." << endl;
	Mat2f decodedImage = computeDecodeImage(graycode, captured_pattern, shadow_mask);

	const string str_x_png = parser.get<string>("x_png");
	const string str_y_png = parser.get<string>("y_png");
	const string str_x_exr = parser.get<string>("x_exr");
	const string str_y_exr = parser.get<string>("y_exr");
	vizDecodedImage(decodedImage, params.width, params.height, str_x_png, str_y_png);
	saveDecodedImage(decodedImage, str_x_exr, str_y_exr);

	const string str_mask = parser.get<string>("mask");
	imwrite(str_mask, getDecoedMask(decodedImage));

	return 0;
}