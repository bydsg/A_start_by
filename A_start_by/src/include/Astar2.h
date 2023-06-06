#ifndef _A_STAR2_H_
#define _A_STAR2_H_

#pragma once
#include<vector>
#include<list>
#include<memory>
#include<algorithm>
#include<iostream>
using namespace std;
 pair<int, int> start_by = { 0,0 };//起点
pair<int, int> over_by = { 0,0 };//起点


 struct Point {
    double x;
    double y;
    double l;
    double r;
    double s;
    double theta;
    int id;
};
struct Road
{
    int id;
    std::vector<int> pre;
    std::vector<int> beh;
    double distance;
    std::vector<Point> road_points;
};
vector<Point> opnlist_point;

int num_by=0;
vector<pair<int, int>>  final_path;
pair<int, int> final_pair;
struct ANode
{
    ANode(int X, int Y, std::shared_ptr<ANode> p = nullptr) : x(X), y(Y), prev(p) {}
    int x; //点的x坐标
    int y; //点的y坐标
    int G=0; //起点到该点的欧拉距离
    int H=0;//该点到终点的曼哈顿距离
    int F=0;//G+H
    std::weak_ptr<ANode> prev;//指向的前一个节点!!!改成了weak_ptr不然数据多的时候析构报错
 
};
class AStar
{
public:
    AStar(std::vector<std::vector<int>> m ): maps(m) {}
    std::shared_ptr<ANode> findPath(std::shared_ptr<ANode> beg, std::shared_ptr<ANode> end);
    void PrintAStarPath(const std::pair<int, int> &, const std::pair<int, int> &);
    ~AStar()
    {
        openlist.clear();
        closeist.clear();
    }
private:
    void refreshOpenList(std::shared_ptr<ANode>, std::shared_ptr<ANode> end);
    int calculateH(std::shared_ptr<ANode>, std::shared_ptr<ANode>) const;
    int calculateF(std::shared_ptr<ANode>,std::shared_ptr<ANode>) const;
 
private:
    std::vector<std::vector<int>> maps;//地图
    std::list<std::shared_ptr<ANode>> openlist;//保存还未遍历过的节点
    std::list<std::shared_ptr<ANode>> closeist;//保存已经找到最短路径的节点
    const static int costLow;//上下位移的距离
    const static int costHigh;//斜向位移的距离
};
const int AStar::costLow = 10;
const int AStar::costHigh = 14;
int AStar::calculateH(std::shared_ptr<ANode> point, std::shared_ptr<ANode> end) const
{
    //return costLow * (std::abs(point->x - end->x) + std::abs(point->y - end->y));
        return costLow *sqrt( (pow(point->x - end->x,2)+pow(point->y - end->y,2)));
}
int AStar::calculateF(std::shared_ptr<ANode> point,std::shared_ptr<ANode> end) const
{
    return point->G + calculateH(point, end);
}
std::shared_ptr<ANode> AStar::findPath(std::shared_ptr<ANode> beg, std::shared_ptr<ANode> end)
{
    refreshOpenList(beg,end);//遍历起点周围的八个点，将其置入opensilt中，若已在则比较其G值是否更小，若更小则替换
    while (!openlist.empty())
    {
        auto iter = std::min_element(openlist.cbegin(), openlist.cend(), [](std::shared_ptr<ANode> p1, std::shared_ptr<ANode> p2)
                                                                                                                                             { return p1->F <= p2->F;});//找到openlis中F最小的
        closeist.push_back(*iter);
        std::shared_ptr<ANode> iter_temp = *iter;
        openlist.erase(iter);
        refreshOpenList(iter_temp, end);
        auto iter2 = std::find_if(openlist.cbegin(), openlist.cend(), [end](std::shared_ptr<ANode> sp)
                                  { return (sp->x == end->x) && (sp->y == end->y); });//判断终点是否进入openlist
        if (iter2 != openlist.end())
            return *iter2;
    }
    return nullptr;
}
void AStar::refreshOpenList(std::shared_ptr<ANode> point, std::shared_ptr<ANode> end)
{
    bool upIsWall = false;//表示当前点上有障碍物，即对应的斜向没法走
    bool downIsWall = false;
    bool leftIsWall = false;
    bool rightIsWall = false;
    if (point->x - 1 >= 0 && maps[point->x - 1][point->y] == 1)
        upIsWall = true;
    if (point->x + 1 < int(maps.size()) && maps[point->x + 1][point->y] == 1)
        downIsWall = true;
    if (point->y - 1 >= 0 && maps[point->x][point->y - 1] == 1)
        leftIsWall = true;
    if (point->y + 1 < int(maps.front().size()) && maps[point->x][point->y + 1] == 1)
        rightIsWall = true;
    //第一步初始化起点及其周围的节点
    if (openlist.empty() && closeist.empty())//全场只进一次？
    {
        closeist.push_back(point);
        for (int i = point->x - 1; i <= point->x + 1; ++i)
        {
            for (int j = point->y - 1; j <= point->y + 1; ++j)
            {
                if (i >= 0 && j >= 0 && i < int(maps.size()) && j < int(maps.front().size()) && (i != point->x || j != point->y) && !maps[i][j])
                {
                    if (i != point->x && j != point->y)
                    {
                        if (leftIsWall && j < point->y)
                            continue;
                        if (rightIsWall && j > point->y)
                            continue;
                        if (upIsWall && i < point->x)
                            continue;
                        if (downIsWall && i > point->x)
                            continue;                       
                    }
                    auto cur = std::make_shared<ANode>(i, j, point);
                    cur->G = (i != point->x && j != point->y) ? costHigh : costLow;
                    cur->H = calculateH(cur, end);
                    cur->F = calculateF(cur, end);
                    openlist.push_back(cur);
                    Point point1;
                    point1.x=(cur)->x;
                    point1.y=(cur)->y;
                    opnlist_point.push_back(point1);
                }
            }
        }
    }
    //刷新每一次找到的起点到当前点的最短路径中的当前点的周围节点情况
    else
    {
        
        for (int i = point->x - 1; i <= point->x + 1; ++i)
        {
            for (int j = point->y - 1; j <= point->y + 1; ++j)
            {
                if (i >= 0 && j >= 0 && i < int(maps.size()) && j < int(maps.front().size()) && (i != point->x || j != point->y) && !maps[i][j])
                {
                    if (i != point->x && j != point->y)
                    {
                        if (leftIsWall && j < point->y)
                            continue;
                        if (rightIsWall && j > point->y)
                            continue;
                        if (upIsWall && i < point->x)
                            continue;
                        if (downIsWall && i > point->x)
                            continue;                       
                    }
                    auto cur = std::make_shared<ANode>(i, j, point);
                    cur->G = ((i != point->x && j != point->y) ? costHigh : costLow) + point->G;
                    cur->H = calculateH(cur, end);
                    cur->F = calculateF(cur, end);

                    auto iter_close=std::find_if(closeist.cbegin(), closeist.cend(), [i,j](std::shared_ptr<ANode> sp)
                                  { return (sp->x == i) && (sp->y == j); });//暂时可以理解成判断ij是否在closeist中
                    if (iter_close == closeist.end())//若没在
                    {
                        auto iter_open=std::find_if(openlist.cbegin(), openlist.cend(), [i,j](std::shared_ptr<ANode> sp)
                                  { return (sp->x == i) && (sp->y == j); });
                        if (iter_open != openlist.end())//若在openlist中
                        {
                            if((*iter_open)->G > cur->G)
                            {
                                (*iter_open)->G = cur->G;
                                (*iter_open)->F = (*iter_open)->G + (*iter_open)->H;
                                (*iter_open)->prev = point;
                            }
                        }
                        else
                           openlist.push_back(cur); 
                            Point point1;
                            point1.x=(cur)->x;
                            point1.y=(cur)->y;
                            opnlist_point.push_back(point1);
                    }
                }
            }
        }
    }
}
void AStar::PrintAStarPath(const std::pair<int, int>& start, const std::pair<int, int>& end)
{
    auto start_sp = std::make_shared<ANode>(start.first, start.second), 
               end_sp = std::make_shared<ANode>(end.first, end.second);
    std::shared_ptr<ANode> final = findPath(start_sp, end_sp);
    if (!final)
        std::cout << "没有找到起点到终点路径" << std::endl;
    else
    {
        int i=0; int j=0;
        final_path.clear();
        std::list<std::shared_ptr<ANode>>::iterator iter;
        while (final)
        {
            i++;
            final_pair.first=final->x;
            final_pair.second=final->y;
            final_path.push_back(final_pair);
            final = final->prev.lock();
        }
    }
}

#endif