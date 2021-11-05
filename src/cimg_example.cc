#include <stdio.h>
#include <iostream>
#include <cmath>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "visualization/CImg.h"

using std::cout;
using std::endl;
using std::sqrt;
using std::sin;
using std::cos;

using Eigen::Vector2f;
using Eigen::Matrix2f;
using Eigen::Rotation2Df;

using cimg_library::CImg;
using cimg_library::CImgDisplay;

int main() {
  cout << "CImg Demo.\n";
  // Create a new image of size 500x400, filled with black pixels.
  CImg<float> image(500,400,1,1,0);

  // Draw filled circles of radius 5 pixels.
  float circle_color = 0.5;
  image.draw_circle(150, 150, 5, &circle_color);
  image.draw_circle(50, 50, 5, &circle_color);
  image.draw_circle(50, 150, 5, &circle_color);
  image.draw_circle(150, 50, 5, &circle_color);

  // Gaussian blur with stddev 2.5 pixels.
  image.blur(2.5);

  CImgDisplay main_disp(image,"Click a point");
  while (!main_disp.is_closed()) {
    main_disp.wait();
    if (main_disp.button()) {
      const int x = main_disp.mouse_x();
      const int y = main_disp.mouse_y();
      if (x >=0 && y >=0 && x < image.width() && y < image.height()) {
        cout << "Value at " << x << "," << y << " : " << image(x, y) << endl;
      }
    }
  }
  return 0;
}
