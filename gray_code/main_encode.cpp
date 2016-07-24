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

using namespace cv;
using namespace std;

 const char* keys =
{ 
"{@path         | .  | Path of the folder where the captured pattern images will be saved }"
"{@proj_width   |512 | Projector width            }"
"{@proj_height  |384 | Projector height           }" 
};

int main(int argc, char* argv[])
{

	structured_light::GrayCodePattern::Params params;
	CommandLineParser parser(argc, argv, keys);
	String path = parser.get<String>(0);
	params.width = parser.get<int>(1);
	params.height = parser.get<int>(2);

	if (path.empty() || params.width < 1 || params.height < 1)
	{
		cerr << "Some of parameters are invalid." << endl;
		return -1;
	}

	// Set up GraycodePattern with params
	Ptr<structured_light::GrayCodePattern> graycode = structured_light::GrayCodePattern::create(params);
	// Storage for pattern
	vector<Mat> pattern;
	graycode->generate(pattern);

	cout << pattern.size() << " pattern images + 2 images for shadows mask computation to acquire with both cameras"
		<< endl;
	// Generate the all-white and all-black images needed for shadows mask computation
	Mat white;
	Mat black;
	graycode->getImagesForShadowMasks(black, white);
	pattern.push_back(white);
	pattern.push_back(black);

	for (size_t i = 0; i < pattern.size(); ++i) 
	{
		imwrite(format("%s/pattern_%02d.png", path.c_str(), i + 1), pattern[i]);
	}
	return 0;
}