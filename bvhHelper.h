#ifndef BVH_HELPER_H_
#define BVH_HELPER_H_

#include <glm/glm.hpp>
#include <memory>
#include "glm-aabb/aabb.hpp"
#include <iostream>
#include <string>
#include <time.h>

inline void elapse_time(std::string tags = "") {

	static clock_t start_time = clock();
	if (tags.size() > 0)
	{
		auto cur_time = clock();

		auto elapsed = (cur_time - start_time) / (float)CLOCKS_PER_SEC;

		start_time = cur_time;

		std::cout << tags << " time:" << elapsed << " s" << std::endl;
	}
	else //reset;
		start_time = clock();
};

namespace BVH
{
	typedef glm::vec3 float3;
	typedef unsigned int uint;

	class Tri
	{
	public:
		glm::vec3 vertex0, vertex1, vertex2;
		glm::vec3 centroid;
	};

	class Ray
	{
	public:
		glm::vec3 O, D;
		float t = 1e30f;
	};

	//version 1
	//struct BVHNode
	//{
	//	glm::AABB nodeBounds;
	//	BVHNode* left, * right;
	//	bool isLeaf;
	//	std::vector<Tri*> primitives;
	//};

	//version 2
	struct BVHNode
	{
		float3 aabbMin, aabbMax;     // 24 bytes 外包围盒的两个角点
		uint leftChild;// , rightChild;//rightChild is always leftChild + 1  // 8 bytes  当前节点左右两个child所在的索引（N个叶子元素最后会生成2N-1个节点）
		uint firstPrim, primCount;   // 8 bytes; total: 44 bytes 节点包含三角形区间第一个图元的索引，以及有多少个图元
		bool isLeaf()
		{
			return primCount != 0;
		}
	};

	class BVHHelper
	{

		BVHNode* bvhNode = nullptr;// [N * 2 - 1] ; N个图元最终会有2N-1个节点
		uint rootNodeIdx = 0, node_used = 1;//初始只有一个根节点
	public:

		int N = 0;
		Tri* tri;
	public:

		BVHHelper() {};

		~BVHHelper()
		{

			N = 0;
			if (tri)
				delete[]tri;
			if (bvhNode)
				delete[]bvhNode;

		}


		/// <summary>
		/// create random triangles for test
		/// </summary>
		/// <param name="n"></param>
		void init(int n)
		{
			N = n;
			tri = new Tri[N];

			for (int i = 0; i < N; i++)
			{
				float3 r0(RandomFloat(), RandomFloat(), RandomFloat());
				float3 r1(RandomFloat(), RandomFloat(), RandomFloat());
				float3 r2(RandomFloat(), RandomFloat(), RandomFloat());
				tri[i].vertex0 = r0 * 9.0f - float3(5.0f);
				tri[i].vertex1 = tri[i].vertex0 + r1;
				tri[i].vertex2 = tri[i].vertex0 + r2;
			}
		};

		float RandomFloat()
		{
			return rand() / (float)RAND_MAX;
		}

		void BuildBVH()
		{
			if (N == 0)return;

			//free
			if (bvhNode) delete[]bvhNode;
			//initial
			bvhNode = new BVHNode[2 * N - 1];
			
			//construction
			for (int i = 0; i < N; i++)
				tri[i].centroid = (tri[i].vertex0 + tri[i].vertex1 + tri[i].vertex2) * 0.3333f;
			// assign all triangles to root node
			BVHNode& root = bvhNode[rootNodeIdx];
			root.leftChild = 0;// root.rightChild = 0;
			root.firstPrim = 0, root.primCount = N;
			UpdateNodeBounds(rootNodeIdx);
			// subdivide recursively
			Subdivide(rootNodeIdx);
		}

		void UpdateNodeBounds(uint node_idx)
		{
			auto& node = bvhNode[node_idx];
			glm::AABB box;
			for (int first = node.firstPrim,i=0; i < node.primCount; i++)
			{
				box.extend(tri[first+i].vertex0);
				box.extend(tri[first+i].vertex1);
				box.extend(tri[first+i].vertex2);
			}
			node.aabbMin = box.getMin();
			node.aabbMax = box.getMax();
		}

		void Subdivide(uint node_idx)
		{
			//1 select axis
			auto& node = bvhNode[node_idx];
			if (node.primCount <= 2)return;//2个三角形有可能永远分不出来
			auto extent = node.aabbMax - node.aabbMin;
			int axis = 0;
			if (extent.y > extent.x) axis = 1;
			if (extent.z > extent[axis])axis = 2;

			float middle_value = ((node.aabbMin + node.aabbMax) * 0.5f)[axis];
			//2 split triangles 并不需要按照从小到大的顺序，只需要小的在左边，大的在右边
			int i = node.firstPrim, j = i + node.primCount-1;
			while (i < j)
			{
				if (tri[i].centroid[axis] < middle_value)
					i++;
				else
					std::swap(tri[i], tri[j--]);
			}
			//--------------------------------------------------------------------------
			//3 create two child node
			int leftcount = i - node.firstPrim;
			//分不出来两个节点，那就没有必要细分了
			if (leftcount == 0 || leftcount == node.primCount) return;

			int left_child_idx = node_used++;
			int right_child_idx = node_used++;
			
			//assign to parent
			node.leftChild = left_child_idx;
			//node.rightChild = right_child_idx;
			auto& leftchild = bvhNode[left_child_idx];
			leftchild.firstPrim = node.firstPrim;
			leftchild.primCount = leftcount;
			
			auto& rightchild = bvhNode[right_child_idx];
			rightchild.firstPrim = i;
			rightchild.primCount = node.primCount - leftcount;
			//remove node content
			node.primCount = 0;
			UpdateNodeBounds(left_child_idx);
			UpdateNodeBounds(right_child_idx);

			//recursive
			Subdivide(left_child_idx);
			Subdivide(right_child_idx);
		}

		bool IntersectBVH(Ray& ray,const uint& nodeidx)
		{
			auto& node = bvhNode[nodeidx];
			if (!IntersectAABB(ray, node.aabbMin, node.aabbMax)) return false;

			bool intersect = false;
			if (node.isLeaf())
			{
				for (auto i = 0; i < node.primCount; i++)
					intersect |= IntersectTri(ray, tri[node.firstPrim + i]);

			}
			else
			{
				intersect |= IntersectBVH(ray, node.leftChild);
				intersect |= IntersectBVH(ray, node.leftChild+1);
			}
			return intersect;
		}

		bool IntersectAABB(Ray& ray, const float3& bmin, const float3& bmax)
		{
			float tx1 = (bmin.x - ray.O.x) / ray.D.x, tx2 = (bmax.x - ray.O.x) / ray.D.x;
			float tmin = std::min<float>(tx1, tx2), tmax = std::max<float>(tx1, tx2);
			float ty1 = (bmin.y - ray.O.y) / ray.D.y, ty2 = (bmax.y - ray.O.y) / ray.D.y;
			tmin = std::max<float>(tmin, std::min<float>(ty1, ty2)), tmax = std::min<float>(tmax, std::max<float>(ty1, ty2));
			float tz1 = (bmin.z - ray.O.z) / ray.D.z, tz2 = (bmax.z - ray.O.z) / ray.D.z;
			tmin = std::max<float>(tmin, std::min<float>(tz1, tz2)), tmax = std::min<float>(tmax, std::max<float>(tz1, tz2));
			return tmax >= tmin && tmin < ray.t && tmax > 0;
		}

		bool IntersectTri(Ray& ray, const Tri& tri)
		{
			const float3 edge1 = tri.vertex1 - tri.vertex0;
			const float3 edge2 = tri.vertex2 - tri.vertex0;
			const float3 h = cross(ray.D, edge2);
			const float a = dot(edge1, h);
			if (a > -0.0001f && a < 0.0001f) return false; // ray parallel to triangle
			const float f = 1 / a;
			const float3 s = ray.O - tri.vertex0;
			const float u = f * dot(s, h);
			if (u < 0 || u > 1) return false;
			const float3 q = cross(s, edge1);
			const float v = f * dot(ray.D, q);
			if (v < 0 || u + v > 1) return false;
			const float t = f * dot(edge2, q);
			if (t > 0.0001f) {
				ray.t = std::min<float>(ray.t, t);
				return true;
			}
			return false;
		};
	};

	inline void internal_test()
	{
		std::unique_ptr<BVHHelper> bvhhelper(new BVHHelper);
		bvhhelper->init(640);
		elapse_time();
		bvhhelper->BuildBVH();
		float3 camPos(0, 0, -18);
		float3 p0(-1, 1, -15), p1(1, 1, -15), p2(-1, -1, -15);
		Ray ray;

		int w = 640;
		int h = 640;
		FILE* f = fopen("image.ppm", "w");         // Write image to PPM file.
		fprintf(f, "P3\n%d %d\n%d\n", w, h, 255);
		
		elapse_time("bvh construction");
		for (int y = 0; y < h; y++) for (int x = 0; x < w; x++)
		{
			float3 pixelPos = p0 + (p1 - p0) * (x / float(w)) + (p2 - p0) * (y / float(h));
			ray.O = camPos;
			ray.D = glm::normalize(pixelPos - ray.O);
			ray.t = 1e30f;

			bool intersect = false;
#if 0
			//naive test:check all triangles
			for (int i = 0; i < bvhhelper->N; i++)
			{
				auto& tri = bvhhelper->tri[i];
				intersect = bvhhelper->IntersectTri(ray, tri);
				if (intersect)
					break;
			}
#else
			uint rootNodeIdx = 0;
			intersect = bvhhelper->IntersectBVH(ray, rootNodeIdx);

#endif

			if (intersect)
			{
				fprintf(f, "%d %d %d ", 255,255,255);
			}
			else
				fprintf(f, "%d %d %d ", 0,0,0);
		}

		elapse_time("elaseped");

	};
}
#endif