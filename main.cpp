//
//  main.cpp
//  A*算法
//
//  Created by mac on 2024/10/18.
//
#include <iostream>
#include <vector>
#include <queue>
#include <algorithm>
#include <cmath>

using namespace std;

// 构建探索地图
const int l = 40; // 地图宽度
const int h = 15; // 地图高度
const int tx = 4; // 目标 x 坐标
const int ty = 35; // 目标 y 坐标

// 0 可行驶的路   1 不可行驶区域  2 起点  3 走过的路  9 是目标
int map[h][l] = {
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
    {4,2,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
    {4,0,0,0,0,4,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
    {4,0,0,0,0,4,0,0,0,0,0,4,4,0,0,0,4,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
    {4,0,0,0,0,4,4,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
    {4,0,0,0,0,0,0,4,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,4},
    {4,0,0,0,0,0,0,4,0,0,0,4,4,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,4},
    {4,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,4,4,0,0,0,0,0,0,0,0,0,0,4},
    {4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,4},
    {4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
    {4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
    {4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4},
    {4,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,4},
    {4,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0,4},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1}
};

// 定义节点结构体 ，用于存储每一个路径点点所有信息 共5个
struct Node {
    int x, y; // 节点的坐标
    float g, f; // g 表示从起点到该节点的实际成本，f 表示 g + h（启发式函数值）
    Node *parent; // 指向父节点的指针

    // 构造函数
    Node(int x, int y, float g, Node *parent)
        : x(x), y(y), g(g), f(0), parent(parent) {}
};

// 定义一个比较器类，用于优先队列
struct CompareNode {
    bool operator()(const Node* lhs, const Node* rhs) const {
        return lhs->f > rhs->f; // 返回 f 值较大的节点，谁的值更大，优先级更小，会被排在后面
    }
};

// 计算欧几里得距离启发函数
float euclidean_heuristic(const Node& node) {
    return sqrt(pow(node.x - tx, 2) + pow(node.y - ty, 2)); // 计算节点与目标节点之间的直线距离
}


// 计算路径平滑启发函数
float smoothness_heuristic(const Node& node) {
    if (node.parent == nullptr) { // 如果没有父节点，则只使用欧几里得距离
        return euclidean_heuristic(node);
    }

    // 计算局部路径长度
    float local_distance = sqrt(pow(node.x - node.parent->x, 2) + pow(node.y - node.parent->y, 2));

    // 计算目标方向的角度
    float angle_to_goal = atan2(ty - node.y, tx - node.x);

    // 计算来自父节点的方向的角度
    float angle_from_parent = atan2(node.y - node.parent->y, node.x - node.parent->x);

    // 计算两个方向之间的角度差异
    float angle_difference = fabs(angle_to_goal - angle_from_parent);

    // 归一化角度差到 [0, PI]
    if (angle_difference > M_PI) {
        angle_difference = 2 * M_PI - angle_difference;
    }

    // 计算局部平滑度惩罚
    float local_smoothness_penalty = local_distance * angle_difference;

    // 初始化全局平滑度惩罚
    float global_smoothness_penalty = 0.0f;

    // 递归计算全局路径平滑度
    Node* current = const_cast<Node*>(&node);
    while (current->parent != nullptr) {
        Node* grandparent = current->parent->parent;
        if (grandparent == nullptr) {
            break;
        }

        // 计算父节点到祖父节点的方向角度
        float grandparent_angle_from_parent = atan2(current->parent->y - grandparent->y, current->parent->x - grandparent->x);

        // 计算父节点方向与祖父节点方向之间的角度差异
        float parent_angle_difference = fabs(grandparent_angle_from_parent - atan2(current->y - current->parent->y, current->x - current->parent->x));

        // 归一化角度差到 [0, PI]
        if (parent_angle_difference > M_PI) {
            parent_angle_difference = 2 * M_PI - parent_angle_difference;
        }

        // 计算父节点的局部平滑度惩罚
        float parent_local_distance = sqrt(pow(current->parent->x - grandparent->x, 2) + pow(current->parent->y - grandparent->y, 2));
        float parent_local_smoothness_penalty = parent_local_distance * parent_angle_difference;

        global_smoothness_penalty += parent_local_smoothness_penalty;

        current = current->parent;
    }

    // 调整平滑度因子
    const float smoothness_factor_local = 0.3f; // 局部平滑度因子
    const float smoothness_factor_global = 0.4f; // 全局平滑度因子

    // 计算总的启发式值
    return euclidean_heuristic(node) + smoothness_factor_local * local_smoothness_penalty + smoothness_factor_global * global_smoothness_penalty;
}

// 显示地图
void show(int a[h][l]) {
    map[tx][ty] = 9; // 设置目标点
    for (int x = 0; x < h; ++x) { // 遍历每行
        for (int y = 0; y < l; ++y) { // 遍历每列
            switch (a[x][y]) { // 根据值打印不同的字符
                case 0: cout << " "; break; // 可行驶的路
                case 1: cout << "H"; break; // 不可行驶区域
                case 2: cout << "O"; break; // 起点
                case 3: cout << "*"; break; // 走过的路
                case 4: cout << "H"; break; // 边界
                case 9: cout << "A"; break; // 目标
                default: break; // 默认不做处理
            }
        }
        cout << endl; // 换行
    }
}

// 在开放列表中查找节点
Node* find_in_open_list(vector<Node*>& open_list, int x, int y) {
    for (auto &node : open_list) { // 遍历开放列表
        if (node->x == x && node->y == y) { // 如果找到节点
            return node; // 返回节点指针
        }
    }
    return nullptr; // 未找到则返回空指针
}

// A* 算法主函数
void a_star(Node *start, Node *goal) {
    priority_queue<Node*, vector<Node*>, CompareNode> open_list_queue; // 创建优先队列当创建priority_queue时，将CompareNode类型的对象传递给它，priority_queue会在内部使用这个对象来调用operator()函数进行元素的比较
    vector<Node*> open_list; // 创建开放列表
    vector<Node*> closed_list; // 创建关闭列表

    start->g = 0; // 设置起始节点的实际成本为 0
    start->f = smoothness_heuristic(*start); // 设置起始节点的总成本
    open_list_queue.push(start); // 将起始节点加入优先队列
    open_list.push_back(start); // 将起始节点加入开放列表

    int step = 0; // 步数计数器
    while (!open_list_queue.empty()) { // 当开放列表不为空
        Node *current = open_list_queue.top(); // 获取当前节点（最小 f 值）
        open_list_queue.pop(); // 移除当前节点

        if (current->x == goal->x && current->y == goal->y) { // 如果到达目标
            vector<Node*> path; // 创建路径向量
            while (current->parent != nullptr) { // 回溯路径
                path.push_back(current); // 推入指针
                current = current->parent; // 移动到父节点
            }
            path.push_back(current); // 添加起点

            // 反转路径
            reverse(path.begin(), path.end());

            // 更新路径上的节点状态
            for (auto &node : path) {
                map[node->x][node->y] = 3; // 标记路径上的节点
            }

            // 显示最终结果
            cout << "找到目标，步数：" << step << endl;
            show(map); // 显示地图
            return; // 退出函数
        }

        closed_list.push_back(current); // 将当前节点加入关闭列表

        static const int directions[8][2] = {{0, 1}, {0, -1}, {1, 0}, {-1, 0}, {1, 1}, {-1, 1}, {-1, -1}, {1, -1}}; // 方向数组
        static const float costs[8] = {1.0f, 1.0f, 1.0f, 1.0f, sqrtf(2.0f), sqrtf(2.0f), sqrtf(2.0f), sqrtf(2.0f)}; // 成本数组

        for (int d = 0; d < 8; ++d) { // 遍历所有方向
            int newX = current->x + directions[d][0]; // 新的 x 坐标
            int newY = current->y + directions[d][1]; // 新的 y 坐标

            if (newX >= 0 && newX < h && newY >= 0 && newY < l && (map[newX][newY] == 0 || map[newX][newY] == 9)) { // 如果新坐标有效
                Node *neighbor = new Node(newX, newY, current->g + costs[d], current); // 创建邻居节点
                neighbor->f = neighbor->g + smoothness_heuristic(*neighbor); // 计算邻居节点的总成本

                auto it = std::find_if(closed_list.begin(), closed_list.end(),[neighbor](const Node* n) { return n->x == neighbor->x && n->y == neighbor->y; }); // 查找关闭列表
                if (it != closed_list.end()) { // 如果已在关闭列表
                    delete neighbor; // 删除邻居节点
                    continue; // 继续下一个方向
                }

                Node* existing_node = find_in_open_list(open_list, neighbor->x, neighbor->y); // 查找开放列表
                if (existing_node != nullptr) { // 如果已存在
                    if (neighbor->g < existing_node->g) { // 如果新节点成本更低
                        existing_node->g = neighbor->g; // 更新成本
                        existing_node->parent = neighbor->parent; // 更新父节点
                        existing_node->f = existing_node->g + smoothness_heuristic(*existing_node); // 更新总成本
                    }
                    delete neighbor; // 删除邻居节点
                } else { // 如果不存在
                    open_list_queue.push(neighbor); // 加入优先队列
                    open_list.push_back(neighbor); // 加入开放列表
                }
            }
        }

        // 显示当前状态
        step++; // 增加步数
        cout << "第 " << step << " 步" << endl;
        show(map); // 显示地图
    }
}

int main() {
    // 初始化地图和节点
    Node *start = new Node(1, 1, 0.0f, nullptr); // 创建起始节点
    Node *goal_node = new Node(tx, ty, 0.0f, nullptr); // 创建目标节点

    a_star(start, goal_node); // 执行 A* 算法

    // 清理内存
    delete start; // 释放起始节点内存
    delete goal_node; // 释放目标节点内存

    return 0; // 返回 0 结束程序
}




