#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>



class LineNode
{
    public:
        LineNode(std::vector<Eigen::Vector2d> const& trackPoints, int left, int right);
        ~LineNode();

        bool pointOnLine(Eigen::Vector2d& point, double threshold);

    private:
        LineNode* _l_child = nullptr;
        LineNode* _r_child = nullptr;

        double _l_boundary = 0;
        double _r_boundary = 0;
        Eigen::RowVector2d _lineNormal;
        double _lineCoefficient;
};