#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <vector>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ClearanceProfileSetter : public rclcpp::Node
{
public:
    ClearanceProfileSetter()
    : Node("clearance_profile_setter")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing ClearanceProfileSetter node");

        // Initialize subscribers
        highlighted_marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
            "highlighted_driving_path", 10, std::bind(&ClearanceProfileSetter::highlightedMarkerCallback, this, std::placeholders::_1));

        // Initialize publishers
        polygon_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ClearanceProfile_Polygon", 10);
        tube_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ClearanceProfile_Tube", 10);

        // Define the clearance profile points
        clearance_profile_points_ = {
            createPoint(0.0, -1.274, 0.0),
            createPoint(0.0, -1.583, 0.380),
            createPoint(0.0, -1.683, 0.380),
            createPoint(0.0, -1.711, 0.760),
            createPoint(0.0, -1.862, 3.590),
            createPoint(0.0, -1.706, 3.895),
            createPoint(0.0, -1.070, 4.740),
            createPoint(0.0, 1.070, 4.740),
            createPoint(0.0, 1.706, 3.895),
            createPoint(0.0, 1.862, 3.590),
            createPoint(0.0, 1.711, 0.760),
            createPoint(0.0, 1.683, 0.380),
            createPoint(0.0, 1.583, 0.380),
            createPoint(0.0, 1.274, 0.0)
        };
    }

private:
    void highlightedMarkerCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received highlighted driving path marker array with %zu markers", msg->markers.size());
        highlighted_marker_array_ = *msg;
        generatePolygonsAndTubes();
    }

    void generatePolygonsAndTubes()
{
    if (highlighted_marker_array_.markers.empty())
    {
        RCLCPP_WARN(this->get_logger(), "No markers in the highlighted driving path");
        return;
    }

    RCLCPP_INFO(this->get_logger(), "Generating polygons and tubes");

    visualization_msgs::msg::MarkerArray polygon_markers;
    visualization_msgs::msg::MarkerArray tube_markers;

    size_t polygon_id = 0;
    size_t tube_id = 0;

    for (const auto& marker : highlighted_marker_array_.markers)
    {
        if (marker.type != visualization_msgs::msg::Marker::LINE_STRIP)
        {
            RCLCPP_WARN(this->get_logger(), "Marker type is not LINE_STRIP, skipping marker with id %d", marker.id);
            continue;
        }

        if (marker.points.size() < 2)
        {
            RCLCPP_WARN(this->get_logger(), "Marker with id %d has insufficient points", marker.id);
            continue;
        }

        // Store polygon markers for later tube generation
        std::vector<visualization_msgs::msg::Marker> polygons;

        // Generate polygons at intervals
        for (size_t i = 0; i < marker.points.size() - 1; ++i)
        {
            const auto& start_point = marker.points[i];
            const auto& end_point = marker.points[i + 1];

            double dx = end_point.x - start_point.x;
            double dy = end_point.y - start_point.y;
            double segment_length = std::sqrt(dx * dx + dy * dy);
            int num_samples = static_cast<int>(std::ceil(segment_length / 100.0)); // Interval of 20 meters

            // Compute the orthogonal vector
            double orthogonal_x = -dy / segment_length;
            double orthogonal_y = dx / segment_length;

            // Sample points along the segment
            for (int j = 0; j <= num_samples; ++j)
            {
                double alpha = j / static_cast<double>(num_samples);
                geometry_msgs::msg::Point sampled_point;
                sampled_point.x = start_point.x + alpha * dx;
                sampled_point.y = start_point.y + alpha * dy;
                sampled_point.z = start_point.z;

                // Create polygon marker
                visualization_msgs::msg::Marker polygon_marker;
                polygon_marker.header.frame_id = marker.header.frame_id;
                polygon_marker.header.stamp = this->now();
                polygon_marker.ns = "2dClearanceProfile_Polygon";
                polygon_marker.id = polygon_id++;
                polygon_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
                polygon_marker.action = visualization_msgs::msg::Marker::ADD;
                polygon_marker.color.a = 0.5; // 50% transparent
                polygon_marker.color.r = 0.0;
                polygon_marker.color.g = 0.588; // KIT green (0, 150, 130)
                polygon_marker.color.b = 0.51;
                polygon_marker.scale.x = 0.1; // Line width

                // Generate polygon points
                for (const auto& profile_point : clearance_profile_points_)
                {
                    geometry_msgs::msg::Point new_point;
                    new_point.x = sampled_point.x + orthogonal_x * profile_point.y;
                    new_point.y = sampled_point.y + orthogonal_y * profile_point.y;
                    new_point.z = sampled_point.z + profile_point.z;

                    polygon_marker.points.push_back(new_point);
                }
                polygon_marker.points.push_back(polygon_marker.points.front()); // Close the polygon

                polygons.push_back(polygon_marker);

                RCLCPP_INFO(this->get_logger(), "Generated polygon marker %d with %zu points", polygon_marker.id, polygon_marker.points.size());
            }
        }

        // Add polygons to marker array
        polygon_markers.markers.insert(polygon_markers.markers.end(), polygons.begin(), polygons.end());

        // Generate tube markers between consecutive polygons
        for (size_t k = 1; k < polygons.size(); ++k)
        {
            const auto& prev_polygon = polygons[k - 1];
            const auto& curr_polygon = polygons[k];

            // Create tube marker
            visualization_msgs::msg::Marker tube_marker;
            tube_marker.header.frame_id = marker.header.frame_id;
            tube_marker.header.stamp = this->now();
            tube_marker.ns = "3dClearanceProfile_Tube";
            tube_marker.id = tube_id++;
            tube_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST; // Use TRIANGLE_LIST for filled tube
            tube_marker.action = visualization_msgs::msg::Marker::ADD;
            tube_marker.color.a = 0.1; // 10% visibility
            tube_marker.color.r = 0.0;
            tube_marker.color.g = 0.588; // KIT green (0, 150, 130)
            tube_marker.color.b = 0.51;
            tube_marker.scale.x = 0.1; // Line width (unused for TRIANGLE_LIST)

            // Add points to tube_marker (create triangles to connect edges of polygons)
            for (size_t p = 0; p < clearance_profile_points_.size(); ++p)
            {
                size_t next_p = (p + 1) % clearance_profile_points_.size();

                // Triangle 1
                tube_marker.points.push_back(prev_polygon.points[p]);
                tube_marker.points.push_back(curr_polygon.points[p]);
                tube_marker.points.push_back(curr_polygon.points[next_p]);

                // Triangle 2
                tube_marker.points.push_back(prev_polygon.points[p]);
                tube_marker.points.push_back(curr_polygon.points[next_p]);
                tube_marker.points.push_back(prev_polygon.points[next_p]);
            }

            RCLCPP_INFO(this->get_logger(), "Generated tube marker %d with %zu points", tube_marker.id, tube_marker.points.size());
            tube_markers.markers.push_back(tube_marker);
        }
    }

    // Publish the markers
    polygon_pub_->publish(polygon_markers);
    tube_pub_->publish(tube_markers);
}


    geometry_msgs::msg::Point32 createPoint(float x, float y, float z)
    {
        geometry_msgs::msg::Point32 point;
        point.x = x;
        point.y = y;
        point.z = z;
        return point;
    }

    rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr highlighted_marker_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr polygon_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tube_pub_;
    visualization_msgs::msg::MarkerArray highlighted_marker_array_;
    std::vector<geometry_msgs::msg::Point32> clearance_profile_points_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ClearanceProfileSetter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// #include "rclcpp/rclcpp.hpp"
// #include "geometry_msgs/msg/point32.hpp"
// #include "visualization_msgs/msg/marker_array.hpp"
// #include "visualization_msgs/msg/marker.hpp"
// #include <vector>
// #include <cmath>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// class ClearanceProfileSetter : public rclcpp::Node
// {
// public:
//     ClearanceProfileSetter()
//     : Node("clearance_profile_setter")
//     {
//         RCLCPP_INFO(this->get_logger(), "Initializing ClearanceProfileSetter node");

//         // Initialize subscribers
//         highlighted_marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
//             "highlighted_driving_path", 10, std::bind(&ClearanceProfileSetter::highlightedMarkerCallback, this, std::placeholders::_1));

//         // Initialize publishers
//         polygon_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ClearanceProfile_Polygon", 10);
//         tube_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ClearanceProfile_Tube", 10);

//         // Define the clearance profile points
//         clearance_profile_points_ = {
//             createPoint(0.0, -1.274, 0.0),
//             createPoint(0.0, -1.583, 0.380),
//             createPoint(0.0, -1.683, 0.380),
//             createPoint(0.0, -1.711, 0.760),
//             createPoint(0.0, -1.862, 3.590),
//             createPoint(0.0, -1.706, 3.895),
//             createPoint(0.0, -1.070, 4.740),
//             createPoint(0.0, 1.070, 4.740),
//             createPoint(0.0, 1.706, 3.895),
//             createPoint(0.0, 1.862, 3.590),
//             createPoint(0.0, 1.711, 0.760),
//             createPoint(0.0, 1.683, 0.380),
//             createPoint(0.0, 1.583, 0.380),
//             createPoint(0.0, 1.274, 0.0)
//         };
//     }

// private:
//     void highlightedMarkerCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
//     {
//         RCLCPP_INFO(this->get_logger(), "Received highlighted driving path marker array");
//         highlighted_marker_array_ = *msg;
//         generatePolygonsAndTubes();
//     }

//    void generatePolygonsAndTubes()
// {
//     if (highlighted_marker_array_.markers.empty())
//     {
//         RCLCPP_WARN(this->get_logger(), "No markers in the highlighted driving path");
//         return;
//     }

//     RCLCPP_INFO(this->get_logger(), "Generating polygons and tubes");

//     visualization_msgs::msg::MarkerArray polygon_markers;
//     visualization_msgs::msg::MarkerArray tube_markers;

//     size_t polygon_id = 0;
//     size_t tube_id = 0;

//     for (const auto& marker : highlighted_marker_array_.markers)
//     {
//         if (marker.type != visualization_msgs::msg::Marker::LINE_STRIP)
//         {
//             RCLCPP_WARN(this->get_logger(), "Marker type is not LINE_STRIP, skipping marker with id %d", marker.id);
//             continue;
//         }

//         if (marker.points.size() < 2)
//         {
//             RCLCPP_WARN(this->get_logger(), "Marker with id %d has insufficient points", marker.id);
//             continue;
//         }

//         // Store polygon markers for later tube generation
//         std::vector<visualization_msgs::msg::Marker> polygons;

//         // Generate polygons at intervals
//         for (size_t i = 0; i < marker.points.size() - 1; ++i)
//         {
//             const auto& start_point = marker.points[i];
//             const auto& end_point = marker.points[i + 1];

//             double dx = end_point.x - start_point.x;
//             double dy = end_point.y - start_point.y;
//             double segment_length = std::sqrt(dx * dx + dy * dy);
//             int num_samples = static_cast<int>(std::ceil(segment_length / 5.0)); // Interval of 5 meters

//             // Compute the orthogonal vector
//             double orthogonal_x = -dy / segment_length;
//             double orthogonal_y = dx / segment_length;

//             // Sample points along the segment
//             for (int j = 0; j <= num_samples; ++j)
//             {
//                 double alpha = j / static_cast<double>(num_samples);
//                 geometry_msgs::msg::Point sampled_point;
//                 sampled_point.x = start_point.x + alpha * dx;
//                 sampled_point.y = start_point.y + alpha * dy;
//                 sampled_point.z = start_point.z;

//                 // Create polygon marker
//                 visualization_msgs::msg::Marker polygon_marker;
//                 polygon_marker.header.frame_id = marker.header.frame_id;
//                 polygon_marker.header.stamp = this->now();
//                 polygon_marker.ns = "2dClearanceProfile_Polygon";
//                 polygon_marker.id = polygon_id++;
//                 polygon_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
//                 polygon_marker.action = visualization_msgs::msg::Marker::ADD;
//                 polygon_marker.color.a = 0.5; // 50% transparent
//                 polygon_marker.color.r = 0.0;
//                 polygon_marker.color.g = 0.588; // KIT green (0, 150, 130)
//                 polygon_marker.color.b = 0.51;
//                 polygon_marker.scale.x = 0.1; // Line width

//                 // Generate polygon points
//                 for (const auto& profile_point : clearance_profile_points_)
//                 {
//                     geometry_msgs::msg::Point new_point;
//                     new_point.x = sampled_point.x + orthogonal_x * profile_point.y;
//                     new_point.y = sampled_point.y + orthogonal_y * profile_point.y;
//                     new_point.z = sampled_point.z + profile_point.z;

//                     polygon_marker.points.push_back(new_point);
//                 }
//                 polygon_marker.points.push_back(polygon_marker.points.front()); // Close the polygon

//                 polygons.push_back(polygon_marker);

//                 RCLCPP_INFO(this->get_logger(), "Generated polygon marker %d with %zu points", polygon_marker.id, polygon_marker.points.size());
//             }
//         }

//         // Add polygons to marker array
//         polygon_markers.markers.insert(polygon_markers.markers.end(), polygons.begin(), polygons.end());

//         // Generate tube markers between consecutive polygons
//         for (size_t k = 1; k < polygons.size(); ++k)
//         {
//             const auto& prev_polygon = polygons[k - 1];
//             const auto& curr_polygon = polygons[k];

//             // Create tube marker
//             visualization_msgs::msg::Marker tube_marker;
//             tube_marker.header.frame_id = marker.header.frame_id;
//             tube_marker.header.stamp = this->now();
//             tube_marker.ns = "3dClearanceProfile_Tube";
//             tube_marker.id = tube_id++;
//             tube_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST; // Use TRIANGLE_LIST for filled tube
//             tube_marker.action = visualization_msgs::msg::Marker::ADD;
//             tube_marker.color.a = 0.1; // 10% visibility
//             tube_marker.color.r = 0.0;
//             tube_marker.color.g = 0.588; // KIT green (0, 150, 130)
//             tube_marker.color.b = 0.51;
//             tube_marker.scale.x = 0.1; // Line width (unused for TRIANGLE_LIST)

//             // Add points to tube_marker (create triangles to connect edges of polygons)
//             for (size_t p = 0; p < clearance_profile_points_.size(); ++p)
//             {
//                 size_t next_p = (p + 1) % clearance_profile_points_.size();

//                 // Triangle 1
//                 tube_marker.points.push_back(prev_polygon.points[p]);
//                 tube_marker.points.push_back(curr_polygon.points[p]);
//                 tube_marker.points.push_back(curr_polygon.points[next_p]);

//                 // Triangle 2
//                 tube_marker.points.push_back(prev_polygon.points[p]);
//                 tube_marker.points.push_back(curr_polygon.points[next_p]);
//                 tube_marker.points.push_back(prev_polygon.points[next_p]);
//             }

//             tube_markers.markers.push_back(tube_marker);

//             RCLCPP_INFO(this->get_logger(), "Generated tube marker %d with %zu points", tube_marker.id, tube_marker.points.size());
//         }
//     }

//     RCLCPP_INFO(this->get_logger(), "Number of polygon markers before publishing: %zu", polygon_markers.markers.size());
//     RCLCPP_INFO(this->get_logger(), "Number of tube markers before publishing: %zu", tube_markers.markers.size());

//     // Publish the markers
//     polygon_pub_->publish(polygon_markers);
//     tube_pub_->publish(tube_markers);

//     RCLCPP_INFO(this->get_logger(), "Published %zu polygon markers and %zu tube markers", polygon_markers.markers.size(), tube_markers.markers.size());
// }



//     geometry_msgs::msg::Point32 createPoint(float x, float y, float z)
//     {
//         geometry_msgs::msg::Point32 point;
//         point.x = x;
//         point.y = y;
//         point.z = z;
//         return point;
//     }

//     rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr highlighted_marker_sub_;
//     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr polygon_pub_;
//     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tube_pub_;
//     visualization_msgs::msg::MarkerArray highlighted_marker_array_;
//     std::vector<geometry_msgs::msg::Point32> clearance_profile_points_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ClearanceProfileSetter>());
//     rclcpp::shutdown();
//     return 0;
// }

// // #include "rclcpp/rclcpp.hpp"
// // #include "geometry_msgs/msg/point32.hpp"
// // #include "visualization_msgs/msg/marker_array.hpp"
// // #include "visualization_msgs/msg/marker.hpp"
// // #include <vector>
// // #include <cmath>
// // #include <tf2/LinearMath/Quaternion.h>
// // #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// // class ClearanceProfileSetter : public rclcpp::Node
// // {
// // public:
// //     ClearanceProfileSetter()
// //     : Node("clearance_profile_setter")
// //     {
// //         RCLCPP_INFO(this->get_logger(), "Initializing ClearanceProfileSetter node");

// //         // Initialize subscribers
// //         highlighted_marker_sub_ = this->create_subscription<visualization_msgs::msg::MarkerArray>(
// //             "highlighted_driving_path", 10, std::bind(&ClearanceProfileSetter::highlightedMarkerCallback, this, std::placeholders::_1));

// //         // Initialize publishers
// //         polygon_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("ClearanceProfile_Polygon", 10);

// //         // Define the clearance profile points
// //         clearance_profile_points_ = {
// //             createPoint(0.0, -1.274, 0.0),
// //             createPoint(0.0, -1.583, 0.380),
// //             createPoint(0.0, -1.683, 0.380),
// //             createPoint(0.0, -1.711, 0.760),
// //             createPoint(0.0, -1.862, 3.590),
// //             createPoint(0.0, -1.706, 3.895),
// //             createPoint(0.0, -1.070, 4.740),
// //             createPoint(0.0, 1.070, 4.740),
// //             createPoint(0.0, 1.706, 3.895),
// //             createPoint(0.0, 1.862, 3.590),
// //             createPoint(0.0, 1.711, 0.760),
// //             createPoint(0.0, 1.683, 0.380),
// //             createPoint(0.0, 1.583, 0.380),
// //             createPoint(0.0, 1.274, 0.0)
// //         };
// //     }

// // private:
// //     void highlightedMarkerCallback(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
// //     {
// //         RCLCPP_INFO(this->get_logger(), "Received highlighted driving path marker array");
// //         highlighted_marker_array_ = *msg;
// //         generatePolygons();
// //     }

// //     void generatePolygons()
// // {
// //     if (highlighted_marker_array_.markers.empty())
// //     {
// //         RCLCPP_WARN(this->get_logger(), "No markers in the highlighted driving path");
// //         return;
// //     }

// //     RCLCPP_INFO(this->get_logger(), "Generating polygons");

// //     visualization_msgs::msg::MarkerArray polygon_markers;

// //     size_t polygon_id = 0;

// //     for (const auto& marker : highlighted_marker_array_.markers)
// //     {
// //         if (marker.type != visualization_msgs::msg::Marker::LINE_STRIP)
// //         {
// //             RCLCPP_WARN(this->get_logger(), "Marker type is not LINE_STRIP, skipping marker with id %d", marker.id);
// //             continue;
// //         }

// //         if (marker.points.size() < 2)
// //         {
// //             RCLCPP_WARN(this->get_logger(), "Marker with id %d has insufficient points", marker.id);
// //             continue;
// //         }

// //         // Generate polygons at intervals
// //         for (size_t i = 0; i < marker.points.size() - 1; ++i)
// //         {
// //             const auto& start_point = marker.points[i];
// //             const auto& end_point = marker.points[i + 1];

// //             double dx = end_point.x - start_point.x;
// //             double dy = end_point.y - start_point.y;
// //             double segment_length = std::sqrt(dx * dx + dy * dy);
// //             int num_samples = static_cast<int>(std::ceil(segment_length / 5.0)); // Interval of 5 meters

// //             for (int j = 0; j <= num_samples; ++j)
// //             {
// //                 double alpha = j / static_cast<double>(num_samples);
// //                 geometry_msgs::msg::Point sampled_point;
// //                 sampled_point.x = start_point.x + alpha * dx;
// //                 sampled_point.y = start_point.y + alpha * dy;
// //                 sampled_point.z = start_point.z;

// //                 // Create polygon marker
// //                 visualization_msgs::msg::Marker polygon_marker;
// //                 polygon_marker.header.frame_id = marker.header.frame_id;
// //                 polygon_marker.header.stamp = this->now();
// //                 polygon_marker.ns = "2dClearanceProfile_Polygon";
// //                 polygon_marker.id = polygon_id++;
// //                 polygon_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
// //                 polygon_marker.action = visualization_msgs::msg::Marker::ADD;
// //                 polygon_marker.color.a = 1.0;
// //                 polygon_marker.color.r = 0.0;
// //                 polygon_marker.color.g = 0.588; // KIT green (0, 150, 130)
// //                 polygon_marker.color.b = 0.51;
// //                 polygon_marker.scale.x = 0.1; // Line width

// //                 // Compute orthogonal vector for the polygon
// //                 double orthogonal_x = -dy / segment_length;
// //                 double orthogonal_y = dx / segment_length;

// //                 // Generate polygon points
// //                 for (const auto& profile_point : clearance_profile_points_)
// //                 {
// //                     geometry_msgs::msg::Point new_point;
// //                     new_point.x = sampled_point.x + orthogonal_x * profile_point.y;
// //                     new_point.y = sampled_point.y + orthogonal_y * profile_point.y;
// //                     new_point.z = sampled_point.z + profile_point.z;

// //                     polygon_marker.points.push_back(new_point);
// //                 }
// //                 polygon_marker.points.push_back(polygon_marker.points.front()); // Close the polygon

// //                 polygon_markers.markers.push_back(polygon_marker);

// //                 RCLCPP_INFO(this->get_logger(), "Generated polygon marker %d with %zu points", polygon_marker.id, polygon_marker.points.size());
// //             }
// //         }
// //     }

// //     RCLCPP_INFO(this->get_logger(), "Number of polygon markers before publishing: %zu", polygon_markers.markers.size());

// //     // Publish the markers
// //     polygon_pub_->publish(polygon_markers);

// //     RCLCPP_INFO(this->get_logger(), "Published %zu polygon markers", polygon_markers.markers.size());
// // }



// //     geometry_msgs::msg::Point32 createPoint(float x, float y, float z)
// //     {
// //         geometry_msgs::msg::Point32 point;
// //         point.x = x;
// //         point.y = y;
// //         point.z = z;
// //         return point;
// //     }

// //     rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr highlighted_marker_sub_;
// //     rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr polygon_pub_;
// //     visualization_msgs::msg::MarkerArray highlighted_marker_array_;
// //     std::vector<geometry_msgs::msg::Point32> clearance_profile_points_;
// // };

// // int main(int argc, char **argv)
// // {
// //     rclcpp::init(argc, argv);
// //     rclcpp::spin(std::make_shared<ClearanceProfileSetter>());
// //     rclcpp::shutdown();
// //     return 0;
// // }
