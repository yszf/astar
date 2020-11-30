#include "astar.h"

template<typename T>
void PrintMap(T map[MAX_X][MAX_Y], int width, int height) {
	for (int i = 0; i < width; ++i) {
		for (int j = 0; j < height; ++j) {
			cout << map[i][j] << " ";
		}
		cout << endl;
	}
}

int CalcPathCostF(vector<APoint*>& path_result) {
	int cost = 0;
	for (auto po : path_result) {
		cost += po->f;
	}
	return cost;
}

int main(int argc, const char* argv[]) {
	//char map_data[MAX_X][MAX_Y] = {
	//	{'0','0','0','0','0','0','0','0','0','0','0','0'},
	//	{'0','1','0','0','1','0','1','1','1','1','1','0'},
	//	{'0','1','1','1','1','0','1','1','1','1','1','0'},
	//	{'0','0','0','0','1','0','1','1','1','1','1','0'},
	//	{'0','1','0','0','1','0','1','1','1','1','0','0'},
	//	{'0','1','1','1','1','0','1','1','1','1','1','0'},
	//	{'0','1','1','0','0','1','1','1','1','1','1','0'},
	//	{'0','1','1','1','1','1','1','1','1','1','1','0'},
	//	{'0','1','0','0','1','1','1','1','1','1','1','0'},
	//	{'0','1','1','0','0','1','1','1','1','1','1','0'},
	//	{'0','1','0','1','1','1','1','1','1','1','1','0'},
	//	{'0','0','0','0','0','0','0','0','0','0','0','0'},
	//};

	char map_data[MAX_X][MAX_Y] = {
		{ '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'},
		{ '0', '1', '1', '0', '0', '1', '0', '1', '1', '1', '1', '0'},
		{ '0', '1', '1', '0', '0', '1', '1', '1', '1', '1', '1', '0'},
		{ '0', '1', '1', '1', '1', '1', '0', '1', '1', '0', '0', '0'},
		{ '0', '0', '0', '1', '1', '1', '1', '1', '0', '0', '1', '0'},
		{ '0', '0', '1', '0', '1', '1', '1', '1', '1', '1', '1', '0'},
		{ '0', '1', '0', '1', '1', '1', '1', '0', '1', '1', '1', '0'},
		{ '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'},
	};

	PrintMap(map_data, MAX_X, MAX_Y);
	AStar star;
	if (!star.InitMap(map_data, MAX_X, MAX_Y)) {
		exit(1);
	}

	APoint* start_point = new (nothrow) APoint;
	assert(nullptr != start_point);
	APoint* end_point = new (nothrow) APoint;
	assert(nullptr != end_point);
	start_point->x = 1;
	start_point->y = 1;
	end_point->x = 6;
	end_point->y = 10;
	vector<APoint*> path_result;
	bool res = star.FindPath(start_point, end_point, path_result);
	if (res) {
		cout << "find path success!" << endl; 
		cout << "path point count: " << path_result.size()
			 << ", path cost f: " << CalcPathCostF(path_result)
			 << ", path res: " << endl;
		for (size_t i = 0; i < path_result.size(); ++i) {
			auto point = path_result[i];
			map_data[point->x][point->y] = '*';
			if (i != path_result.size() - 1) {
				cout << "(" << point->x << "," << point->y << "," 
					<< point->f << "," << point->g << "," << point->h << ")->";
			}
			else {
				cout << "(" << point->x << "," << point->y << "," 
					<< point->f << "," << point->g << "," << point->h << ")";
			}
		}
		cout << endl;
		cout << "--------print path---------" << endl;
		PrintMap(map_data, MAX_X, MAX_Y);
	}
	else {
		cout << "find path failed!" << endl;
	}

	delete start_point;
	delete end_point;

	return 0;
}