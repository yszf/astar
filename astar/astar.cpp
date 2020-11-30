#include "astar.h"

APoint::APoint() :x(0), y(0), f(0), g(0), h(0), type(kCanReach), parent(nullptr) {

}

AStar::AStar() : cur_point_(nullptr), end_point_(nullptr), is_init_(false) {

}

AStar::~AStar() {
    open_list_.clear();
    close_list_.clear();
    neighbour_list_.clear();

    for (size_t i = 0; i < all_points_.size(); ++i) {
        for (size_t j = 0; j < all_points_[i].size(); ++j) {
            delete all_points_[i][j];
            all_points_[i][j] = nullptr;
        }
    }

    cur_point_ = nullptr;
    end_point_ = nullptr;
}

bool AStar::InitMap(DataType array[MAX_X][MAX_Y], int width, int height) {
    for (int i = 0; i < width; ++i) {
        vector<APoint*> line_points;
        for (int j = 0; j < height; ++j) {
            APoint* point = new (nothrow) APoint;
            if (nullptr == point) {
                cout << "allocate memory failed" << endl;
                return false;
            }
            point->x = i;
            point->y = j;
            if ('0' != array[i][j] && '1' != array[i][j]) {
                cout << "init point failed" << endl;
                return false;
            }
            else if ('0' == array[i][j]) {
                point->type = kBarrier;
            }
            line_points.emplace_back(point);
        }
        all_points_.emplace_back(line_points);
    }
	is_init_ = true;
    return true;
}

bool AStar::_CanReach(int x, int y) {
    if (0 >= x || MAX_X - 1 <= x || 0 >= y || MAX_Y - 1 <= y) { // 地图外围都是障碍
        return false;
    }

	APoint* point = _GetPoint(x, y);
	if (nullptr == point) {
		cout << "map is not init" << endl;
		return false;
	}

    if (kBarrier != point->type) {
        return true;
    }
    else {
        return false;
    }
}

bool AStar::_CanReach(APoint* point, int x, int y, bool is_ignore_corner) {
    if (point->x == x && point->y == y) {
        return false;
    }

    if (!_CanReach(x, y) || _IsExist(close_list_, x, y)) {
        return false;
    }
    else {
        if (1 == abs(x - point->x) + abs(y - point->y)) {
            return true;
        }
        else {
            if (x > point->x && y > point->y) { // 右下
                if (_CanReach(x - 1, y) && _CanReach(x, y - 1)) {
                    return true;
                }
            }
            else if (x > point->x && y < point->y) { // 左下
                if (_CanReach(x - 1, y) && _CanReach(x, y + 1)) {
                    return true;
                }
            }
            else if (x < point->x && y < point->y) { // 左上
                if (_CanReach(x + 1, y) && _CanReach(x, y + 1)) {
                    return true;
                }
            }
            else if (x < point->x && y > point->y) { // 右上
                if (_CanReach(x + 1, y) && _CanReach(x, y - 1)) {
                    return true;
                }
            }

            return is_ignore_corner;
        }
    }
}

APoint* AStar::_GetPoint(int x, int y) {
	if (0 >= x || MAX_X - 1 <= x || 0 >= y || MAX_Y - 1 <= y) {
		return nullptr;
	}
	return all_points_[x][y];
}

bool AStar::_GetNeighboringPoint(APoint* point, bool is_ignore_corner) {
    neighbour_list_.clear();
    for (int x = point->x - 1; x <= point->x + 1; ++x) {
        for (int y = point->y - 1; y <= point->y + 1; ++y) {
            if (_CanReach(point, x, y, is_ignore_corner)) {
				APoint* p = _GetPoint(x, y);
				if (nullptr == point) {
					cout << "map is not init" << endl;
					return false;
				}
                neighbour_list_.emplace_back(p);
            }
        }
    }
	return true;
}

bool AStar::_IsExist(const vector<APoint*>& vec, APoint* point) {
    assert(nullptr != point);
    for (const auto po : vec) {
        if (po->x == point->x && po->y == point->y) {
            return true;
        }
    }
    return false;
}

bool AStar::_IsExist(const vector<APoint*>& vec, int x, int y) {
    for (const auto po : vec) {
        if (po->x == x && po->y == y) {
            return true;
        }
    }
    return false;
}

void AStar::_FoundPoint(APoint* cur_point, APoint* point) {
    assert(nullptr != cur_point && nullptr != point);
    int g = _CalcG(cur_point, point);
    if (g < point->g) {
        point->parent = cur_point;
        point->g = g;
        point->CalcF();
    }
}

void AStar::_NotFoundPoint(APoint* cur_point, APoint* point) {
    assert(nullptr != cur_point && nullptr != point);
    point->parent = cur_point;
    point->g = _CalcG(cur_point, point);
    point->h = _CalcH(end_point_, point);
	point->CalcF();
    point->type = kOpened;
    open_list_.emplace_back(point);
}

int AStar::_CalcG(APoint* start, APoint* point) {
    assert(nullptr != start && nullptr != point);
    int g = (2 == abs(point->x - start->x) + abs(point->y - start->y)) ? OBLIQUE : STEP;
    int parent_g = nullptr != point->parent ? point->parent->g : 0;
    return g + parent_g;
}

// A*算法的要求是H值必须恒小于实际路径长
//用曼哈顿算法不一定能得到最优路径 只能叫A搜索 不能叫A*搜索
//A*本身不限制H使用的估计算法 如max(dx, dy)、sqrt(dx*dx + dy * dy)
//只要你能保证H值恒小于实际路径长，A* 就是成立的。你甚至可以取一个常数0，这样A*就退化为广搜了
int AStar::_CalcH(APoint* end, APoint* point) {
    assert(nullptr != end && nullptr != point);
	// 曼哈顿城市街区估算法
	int step = abs(point->x - end->x) + abs(point->y - end->y);
	// 欧几里得
//	int step = static_cast<int>(sqrt(pow(abs(point->x - end->x), 2) + pow(abs(point->y - end->y), 2)));
    return step * STEP;
}

//自定义排序函数
bool MySort(const APoint* p1, const APoint* p2) {
	assert(nullptr != p1 && nullptr != p2);
    return p1->f < p2->f;
}


bool AStar::FindPath(APoint* begin_point, APoint* end_point, vector<APoint*>& path_result) {
	if (!is_init_) {
		cout << "map is not init" << endl;
		return false;
	}

    bool res = false;
    if (nullptr == begin_point || nullptr == end_point) {
        cout << "param error" << endl;
        return res;
    }

    if (kBarrier == begin_point->type) {
        cout << "begin point is barrier" << endl;
        return res;
    }

    if (kBarrier == end_point->type) {
        cout << "end point is barrier" << endl;
        return res;
    }

    if (*begin_point == *end_point) {
        cout << "begin == end" << endl;
        return res;
    }

    end_point_ = end_point;
    open_list_.emplace_back(begin_point);
    begin_point->type = kOpened;

    do {
        cur_point_ = open_list_[0];
        open_list_.erase(open_list_.begin());
        cur_point_->type = kClosed;

        close_list_.emplace_back(cur_point_);

        if (*cur_point_ == *end_point_) {
			res = true;
            // 无头结点链表逆序
            APoint* p = cur_point_;
            APoint* q = p->parent;
            p->parent = nullptr;
            while (nullptr != q) {
				APoint* tmp = q->parent;
                q->parent = p;
                p = q;
                q = tmp;
            }

            while (nullptr != p) {
                path_result.emplace_back(p);
                p = p->parent;
            }
            break;
        }

        // 获取可以到达的相邻节点
		if (!_GetNeighboringPoint(cur_point_)) {
			break;
		}
        for (size_t i = 0; i < neighbour_list_.size(); ++i) {
            APoint* point = neighbour_list_[i];
            if (_IsExist(open_list_, point)) { // kOpened == point->type 在开放列表
                // 计算g值，如果比原来的大，就什么都不做，否则设置它的父节点为当前节点，并更新g和f
                _FoundPoint(cur_point_, point);
            }
            else {
                _NotFoundPoint(cur_point_, point);
            }
        }
        //排序 F值最小的排在前面
        sort(open_list_.begin(), open_list_.end(), MySort);
    } while (0 < open_list_.size());

	//for (size_t i = 0; i < close_list_.size(); ++i) {
	//	cout << close_list_[i]->x << "," << close_list_[i]->y << " ";
	//}
	//cout << endl;
    return res;
}