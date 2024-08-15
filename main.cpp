#include <iostream>
#include <glm/glm.hpp>
#include <vector>
#include <sstream>
#include <string>

#include <fstream>
#include "pystring.h"
#include "include/canvas.h"
#include "glm-aabb/aabb.hpp"
#include "simplifyPath.h"


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

template<class T>
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

std::vector<glm::vec4> heightmap;

//vec4: x y height index(of contour)
struct LeftIdx
{
    int left_idx;
    float height;
    bool righthere;
};
LeftIdx getHeightFromHeightMap(float x)
{
    if (heightmap.empty())
        return { 0,std::numeric_limits<float>::min() };
    if(x < heightmap.front().x)
        return { -1,std::numeric_limits<float>::min() };
    if(x > heightmap.back().x)
		return { (int)heightmap.size()-1,std::numeric_limits<float>::min()};



    //search index
    glm::vec4 height_start;
    glm::vec4 height_end;
    int idx = 0;
    for (auto item = heightmap.begin(); item != heightmap.end(); item++)
    {
        if (x == item->x)
        {
            return { idx,item->z ,true};
        }
        else if (x <= item->x)
        {
            height_end = *item;

            item--;
            height_start = *item;
            idx--;
            break;
        }
        idx++;
    }

    //slerp value
    float ratio = (x - height_start.x) / (height_end.x - height_start.x);
    float slerp_height = height_start.z + (height_end.z - height_start.z) * ratio;

    return { idx,slerp_height,false };
}

//void insertAt(int idx,glm::vec4 val)
//{
//
//	auto left_pos = heightmap.begin();
//	for (int i = 0; i < idx; i++)left_pos++;
//
//    heightmap.insert(left_pos, val);
//}

void insertAt(float x, glm::vec4 val)
{
    if (heightmap.empty())
    {
        heightmap.emplace_back(val);
        return;
    }
        
    auto leftpart = getHeightFromHeightMap(x);
    
    if (leftpart.righthere)
    {
        //replace it?
        if (val.z > leftpart.height)
		{
			auto left_pos = heightmap.begin();
			for (int i = 0; i < leftpart.left_idx; i++)left_pos++;
            left_pos->z = val.z;
        }
        return;
    }
    int idx = leftpart.left_idx + 1;

	auto left_pos = heightmap.begin();
	for (int i = 0; i < idx; i++)left_pos++;

	heightmap.insert(left_pos, val);
}

void removeFrom(float xmin, float xmax)
{
    for (auto item = heightmap.begin(); item != heightmap.end(); /*item++*/)
    {
        if (item->x > xmin && item->x < xmax)
        {
            item = heightmap.erase(item);

        }
        else
            item++;
    }
}


bool lineSegmentsIntersect(const glm::vec2& p0, const glm::vec2& p1, const glm::vec2& q0, const glm::vec2& q1, glm::vec2& intersection) {
	glm::vec2 s1 = p1 - p0;
	glm::vec2 s2 = q1 - q0;

	float s = (-s1.y * (p0.x - q0.x) + s1.x * (p0.y - q0.y)) / (-s2.x * s1.y + s1.x * s2.y);
	float t = (s2.x * (p0.y - q0.y) - s2.y * (p0.x - q0.x)) / (-s2.x * s1.y + s1.x * s2.y);

	if (s >= 0 && s <= 1 && t >= 0 && t <= 1) {
		// Intersection point
		intersection = p0 + (t * s1);
		std::cout << "Line segments intersect at: (" << intersection.x << ", " << intersection.y << ")" << std::endl;
		return true;
	}

	return false;
}


/// <summary>
/// x,y,z,index
/// </summary>
/// <param name="val"></param>
void updateHeightMap(glm::vec4 line_start,glm::vec4 line_end)
{
	auto val_start = getHeightFromHeightMap(line_start.x);
	auto val_end = getHeightFromHeightMap(line_end.x);
    //not exist?
    bool not_exist = false;
    if (val_start.height == std::numeric_limits<float>::min())
    {
        //insert this point
        insertAt(line_start.x,glm::vec4(line_start.x, 0, line_start.y, line_start.w));
        not_exist = true;
    }
    if (val_end.height == std::numeric_limits<float>::min())
    {
        //insert this point
        insertAt(line_end.x,glm::vec4(line_end.x, 0, line_end.y, line_end.w));
        not_exist = true;
    }
    
    if (not_exist) return;


    if ((line_start.y - val_start.height) * (line_end.y - val_end.height) == 0)
    {
        if (line_start.y == val_start.height)
        {
            //lie on start
            //update end
            if (line_end.y > val_end.height)
            {
                insertAt(line_end.x, glm::vec4(line_end.x, 0, line_end.y, line_end.w));
            }

        }
        else if (line_end.y == val_end.height)
		{
			//lie on end
			//update start
			if (line_start.y > val_start.height)
			{
				insertAt(line_start.x, glm::vec4(line_start.x, 0, line_start.y, line_start.w));
			}
        }

        return;

    }
    bool has_intersection = (line_start.y - val_start.height) * (line_end.y - val_end.height) < 0;

    

    if (!has_intersection)
    {
        if (line_start.y - val_start.height < 0)
            //ignore this segment
            return;
        else //update this segment
        {
            removeFrom(line_start.x, line_end.x);

			insertAt(line_start.x, glm::vec4(line_start.x, 0, line_start.y, line_start.w));

			insertAt(line_end.x, glm::vec4(line_end.x, 0, line_end.y, line_end.w));
        }
    }
    else //
    {
		glm::vec2 map_start(line_start.x, val_start.height);
		glm::vec2 map_end(line_end.x, val_end.height);
        glm::vec2 intersection_pt;

        bool re = lineSegmentsIntersect(map_start, map_end, glm::vec2(line_start), glm::vec2(line_end),intersection_pt);
        assert(re);


        if (line_start.y < val_start.height)
		{

			removeFrom(intersection_pt.x, line_end.x);
            //insert intersection point
            glm::vec4 intersect_v4(intersection_pt.x, 0, intersection_pt.y, line_start.w);
            insertAt(intersection_pt.x, intersect_v4);

			//insert endpoint
			insertAt(line_end.x, glm::vec4(line_end.x, 0, line_end.y, line_end.w));

        }
        else
		{
			removeFrom(line_start.x,intersection_pt.x);
			//insert start point
			insertAt(val_start.left_idx + 1, glm::vec4(line_start.x,0, line_start.y, line_start.w));

			//insert intersection point
			glm::vec4 intersect_v4(intersection_pt.x, 0, intersection_pt.y, line_start.w);
			insertAt(intersection_pt.x, intersect_v4);
        }
    }
}


void createHeightMap(const SliceContours<glm::vec3>& contours)
{
    if (!contours.contours.size())
        return;

    //initialize with first contour
    heightmap.clear();
    int contour_idx = 0;
    for (auto contour : contours.contours)
    {
        //update each segment
        //for (int i = 0; i < contour.size() - 1; i++)
        for (int i = 0; i < contour.size()/2+5; i++)
        {
			auto line_start = contour.points[i];//assign height 
			auto line_end = contour.points[i+1];

            if (contour.size() / 2 == i)
                int a = 0;
            updateHeightMap(glm::vec4(line_start, contour_idx), glm::vec4(line_end,contour_idx));
        }
        contour_idx++;
        break;
    }

    int a = 0;
    
    //auto& firstcontour = contours.contours[0];

    //1 get min and max id in direction x
    //int min_x_id = -1;
    //float min_x = 1000000.0f;
    //int max_x_id = -1;
    //float max_x = -min_x;
    //for (int i = 0; i < firstcontour.points.size(); i++)
    //{
    //    if (min_x > firstcontour.points[i].x)
    //    {
    //        min_x = firstcontour.points[i].x;
    //        min_x_id = i;
    //    }
    //    if (max_x < firstcontour.points[i].x)
    //    {
    //        max_x = firstcontour.points[i].x;
    //        max_x_id = i;
    //    }
    //}
    //assert(max_x_id != min_x_id);

    //2 



}

bool getIntersectionHeight(const glm::vec2& pt0, const glm::vec2& pt1,float x,float& height)
{
    glm::vec2 pt_s, pt_e;
    if (pt0.x == pt1.x)
    {
        if (pt0.x == x)
        {
            height = std::max<float>(pt0.y, pt1.y);
            return true;
        }
        else return false;
    }
    else if (pt0.x < pt1.x)
    {
        pt_s = pt0;
        pt_e = pt1;
    }
    else
    {
        pt_s = pt1;
        pt_e = pt0;
    }

    if (pt_s.x > x)
        return false;
    else if (pt_e.x < x)
        return false;

    float ratio = (x - pt_s.x) / (pt_e.x - pt_s.x);
    height = pt_s.y + (pt_e.y - pt_s.y) * ratio;
    return true;
}
std::vector<Contour<glm::vec2>> getHeightContour(const SliceContours<glm::vec3>& contours)
{
    double step = 0.01;//mm


	//1 get min and max
	float min_x = std::numeric_limits<float>::max();
	float max_x = -min_x;
    
    for (auto contour : contours.contours)
    {
        for (int i = 0; i < contour.points.size(); i++)
        {
            if (min_x > contour.points[i].x)
            {
                min_x = contour.points[i].x;
            }
            if (max_x < contour.points[i].x)
            {
                max_x = contour.points[i].x;
            }
        }
    }

    float len_contour_on_dir_x = max_x - min_x;
    if (len_contour_on_dir_x == 0)
    {
        std::vector<Contour<glm::vec2>> empty;
        return empty;
    }

    std::vector<Point> line_paths;


    std::vector<float> heightContour;
    for (float x = min_x; x <= max_x; x += step)
    {
        float height_max = std::numeric_limits<float>::min();
        //for each segment
        for (auto contour : contours.contours)
        {
            for (int i = 0; i < contour.points.size(); i++)
            {
				auto pt0 = contour.points[i];
				auto pt1 = contour.points[(i+1)% contour.points.size()];

                float height;
                if (getIntersectionHeight(pt0, pt1,x, height))
                {
                    if (height > height_max)
                    {
                        height_max = height;
                    }
                };
                
            }
        }
        heightContour.push_back(height_max);
        
        line_paths.emplace_back(x, height_max);
        
		//heightmap.emplace_back(x, 0, height_max, 0);
    }

    std::unique_ptr<simplifyPath> simplify(new simplifyPath);

    auto pts = simplify->simplifyWithRDP(line_paths,0.01);

    std::vector< Contour<glm::vec2>> heightcontours;
    Contour<glm::vec2> contour;
    for (auto pt : pts)
    {
        if (pt.y == std::numeric_limits<float>::min())
        {
            if (contour.size())
            {
                heightcontours.push_back(contour);
                contour.clear();
            }
        }
        else
            contour.points.emplace_back(pt.x, pt.y);
    }
    if(contour.size())
        heightcontours.push_back(contour);

    return heightcontours;
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

	}

    int display_contour = 17;

    //processing
    auto height_contour= getHeightContour(contours[display_contour]);

    for (auto contour : contours)
    {
        getHeightContour(contour);
    }

    //2 create heigh getting function from xyz

	canvas::set_title(L"test");
	canvas::set_size(window_size, window_size);

    
    canvas::on_render([&]() {
        using namespace canvas::color;

        canvas::buffer buf;
        buf.set_size(window_size, window_size, black);
#pragma region drawing
        //buf.draw_line(0, 0, 100, 100, canvas::rgb(255, 0, 0));
        int display_idx = display_contour;
        for (int k = 0; k < contours.size(); k++)
        {
            if (display_idx != k)
                continue;

            auto slicecontour = contours[k];

            for (auto contour : slicecontour.contours)
            {
                unsigned int rand_color = canvas::rgb(rand() % 255, rand() % 255, rand() % 255);//canvas::rgb(255, 0, 0);// 
                int sizeOfPoints = contour.points.size();
                for (int i = 0; i < sizeOfPoints; i++)
                {
                    buf.draw_line(contour.points[i].x, contour.points[i].y,
                        contour.points[(i + 1) % sizeOfPoints].x, contour.points[(i + 1) % sizeOfPoints].y, rand_color);
                };
            }
        }


        //draw heightmap
        for (auto contour : height_contour)
        {
            unsigned int red_color = canvas::rgb(255, 255, 0);// rand() % 255);
            for (int i = 0; i < contour.size() - 1; i++)
            {
                glm::vec2 pt_s = contour.points[i];// (height_contour[i].x, height_contour[i].y);

                glm::vec2 pt_e = contour.points[i + 1];//(height_contour[i+1].x, height_contour[i+1].y);

                buf.draw_line(pt_s.x, pt_s.y,
                    pt_e.x, pt_e.y, red_color);
            }
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