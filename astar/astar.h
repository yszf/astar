#ifndef __A_STAR_H_2020_11_27__
#define __A_STAR_H_2020_11_27__

#include <vector>
#include <iostream>
#include <algorithm>
#include <cassert>
#ifdef _WIN32
#include <vld.h>
#endif

using namespace std;

#define STEP 10
#define OBLIQUE 14

#define MAX_X 8
#define MAX_Y 12

typedef char DataType;

enum APointType {
    kCanReach,
    kClosed,
    kOpened,
    kBarrier
};

class APoint {
public:
    APoint();
    int x;
    int y;
    int f;
    int g;
    int h;
    APointType type;
    APoint* parent;

    bool operator == (const APoint& point) {
        if (x == point.x && y == point.y) {
            return true;
        }
        return false;
    }

    void CalcF() {
        f = g + h;
    }
};

class AStar {
public:
    AStar();
    ~AStar();
    bool InitMap(DataType array[MAX_X][MAX_Y], int width, int height);
    bool FindPath(APoint* begin_point, APoint* end_point, vector<APoint*>& path_result);
private:
    bool _CanReach(int x, int y);
    bool _CanReach(APoint* point, int x, int y, bool is_ignore_corner);
	APoint* _GetPoint(int x, int y);
    bool _GetNeighboringPoint(APoint* point, bool is_ignore_corner = false);
    bool _IsExist(const vector<APoint*>& vec, APoint* point);
    bool _IsExist(const vector<APoint*>& vec, int x, int y);
    void _FoundPoint(APoint* cur_point, APoint* point);
    void _NotFoundPoint(APoint* cur_point, APoint* point);
    int _CalcG(APoint* start, APoint* point);
    int _CalcH(APoint* end, APoint* point);
private:
    vector<APoint*> open_list_;
    vector<APoint*> close_list_;
    vector<APoint*> neighbour_list_;
    vector<vector<APoint*>> all_points_;
    APoint* cur_point_;
    APoint* end_point_;
	bool is_init_;
};

#endif // AStar