#include "LineNode.h"
#include <iostream>

LineNode::LineNode(std::vector<Eigen::Vector2d> const& trackPoints, int left, int right)
{
    int mid = left + (right - left) / 2;

    _r_boundary = trackPoints[mid][0];
    _l_boundary = trackPoints[mid - 1][0];

    _lineNormal = (trackPoints[mid] - trackPoints[mid - 1]).normalized();
    _lineNormal = Eigen::RowVector2d(-_lineNormal[1], _lineNormal[0]);
    
    _lineCoefficient = _lineNormal * trackPoints[mid - 1];

    if(left <= mid - 1)
        _l_child = new LineNode(trackPoints, left, mid - 1);
    if(mid + 1 <= right)
        _r_child = new LineNode(trackPoints, mid + 1, right);
}

LineNode::~LineNode()
{
    if(_l_child != nullptr)
        delete _l_child;
    if(_r_child != nullptr)
        delete _r_child;
}

bool LineNode::pointOnLine(Eigen::Vector2d& point, double threshold)
{
    bool pointOnLine = false;

    if(point[0] < _l_boundary)
    {
        if(_l_child != nullptr)
            pointOnLine = _l_child->pointOnLine(point, threshold);
    }
    else if(point[0] > _r_boundary)
    {
        if(_r_child != nullptr)
            pointOnLine = _r_child->pointOnLine(point, threshold);
    }
    else
    {
        double distance = abs(_lineNormal * point - _lineCoefficient);

        if(distance < threshold)
            pointOnLine = true;
    }

    return pointOnLine;
}