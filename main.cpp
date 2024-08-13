#include <iostream>
#include <glm/glm.hpp>
#include <vector>
#include <sstream>
#include <string>

#include <fstream>
#include "pystring.h"
#include "include/canvas.h"
#include "glm-aabb/aabb.hpp"


template<class T>
struct Contour
{
    std::vector<T> points; 

    void clear()
    {
        points.clear();
    }

    int size()
    {
        return points.size();
    }
};

template<class T>
struct SliceContours
{
    std::vector<Contour<T>> contours;

    void clear()
    {
        contours.clear();
    }

    int size()
    {
        return contours.size();
    }
};

template<class T>
concept check = requires(T a) {
	T::length();
};

template<check T>
std::vector<SliceContours<T>> loadContourFromFile(const std::string& filename)
{
    std::vector<SliceContours<T>> contours;

    std::ifstream infile(filename);

    int slice_index_current = -1;
    int contour_index_current = -1;
    std::string line;
    SliceContours<T> slicecontour;
    Contour<T> contourpath;

    int num_vec_part = T::length();
    while (std::getline(infile, line))
    {
        line = pystring::strip(line);
        if(!line.size())
        continue;
        std::vector<std::string> segments;
        pystring::split(line,segments," ");

        int offset = 2;
        int num_part = std::min<int>(segments.size()-offset, num_vec_part);
        int slice_idx = std::stoi(segments[0]);
        int contour_idx = std::stoi(segments[1]);

        //create new
        if(slice_idx!=slice_index_current)
        {
            if (contourpath.size())
            {
                slicecontour.contours.push_back(contourpath);
            }
            if(slicecontour.size())
			{
                contours.emplace_back(slicecontour);
			}
			slice_index_current = slice_idx;
			contour_index_current = contour_idx;
			slicecontour.clear();
			contourpath.clear();
        }
        else if(contour_idx !=contour_index_current)
        {
            if (contourpath.size())
			{
                slicecontour.contours.emplace_back(contourpath);
			}
			contour_index_current = contour_idx;

            contourpath.clear();
        }

        T point;
        for(auto i=0;i< num_vec_part;i++)
        {
            float v = std::stof(segments[offset+i]);
            point[i] = v;
        }
        contourpath.points.emplace_back(point);

    }

    return contours;

}

int main(int argc,char** argv)
{
    //initialize variables

	int window_size = 800;

    //1 load file into points
    auto contours = loadContourFromFile<glm::vec3>("slice_contours.txt");

    //2 calculate the bounding box
    glm::AABB box;
    for (auto slice : contours)
    {
        for (auto contour : slice.contours)
        {
            for (auto pt : contour.points)
            {
                box.extend(pt);
            }
        }
        //break;
    }

    float offset_window_size = window_size - 30;
    glm::vec3 extent = box.getDiagonal();
    int idx = extent.x > extent.y ? 0 : 1;
    float scale = idx == 0 ? offset_window_size / extent.x : offset_window_size / extent.y;
    glm::vec3 offset = box.getMin();

    //apply transform
	for (auto& slice : contours)
	{
		for (auto& contour : slice.contours)
		{
			for (auto& pt : contour.points)
			{
                pt = (pt - offset) * scale;
			}
		}

		//break;
	}

    //2 create heigh getting function from xyz

	canvas::set_title(L"polyline Example");
	canvas::set_size(window_size, window_size);

    
    canvas::on_render([&]() {
        using namespace canvas::color;

        canvas::buffer buf;
        buf.set_size(window_size, window_size, gray);
#pragma region drawing
        //buf.draw_line(0, 0, 100, 100, canvas::rgb(255, 0, 0));
        for (auto slicecontour : contours)
        {
            unsigned int rand_color = canvas::rgb(rand() % 255, rand() % 255, rand() % 255);//canvas::rgb(255, 0, 0);// 
            for (auto contour : slicecontour.contours)
            {
                int sizeOfPoints = contour.points.size();
                for (int i = 0; i < sizeOfPoints; i++)
                {
                    buf.draw_line(contour.points[i].x, contour.points[i].y,
                        contour.points[(i + 1)% sizeOfPoints].x, contour.points[(i + 1)% sizeOfPoints].y,rand_color);
                };
            }
            //break;
        }


#pragma endregion

        buf.flip();
        canvas::get_frame().stretch(buf);

        /*  Draw the canvas.
         */
        canvas::finalize();

        /*  Reset the render function.
         */
        canvas::done();

        });


	return canvas::run();
}