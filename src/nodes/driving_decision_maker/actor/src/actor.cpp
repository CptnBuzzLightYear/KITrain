#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <observer_msgs/msg/observer_info.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <deque>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <string>
#include <cstring>

class ActorNode : public rclcpp::Node {
public:
    ActorNode() : Node("actor_node"), 
                  current_state_(State::UNINITIALIZED), // Start in an uninitialized state
                  smoothing_window_size_(10), detection_threshold_(40.0), 
                  last_velocity_(0.0), v_max_(5.0), 
                  has_published_idle_(false) { // Track whether IDLE has been published

        current_task_id_ = -1;  // Initialize TaskID to -1 (no task received)

        // Subscriptions
        observer_info_subscription_ = this->create_subscription<observer_msgs::msg::ObserverInfo>(
            "observer_info", 10, std::bind(&ActorNode::observerInfoCallback, this, std::placeholders::_1));
        
        velocity_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "vehicle_velocity", 10, std::bind(&ActorNode::velocityCallback, this, std::placeholders::_1));
        
        order_info_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "order_info", 10, std::bind(&ActorNode::orderInfoCallback, this, std::placeholders::_1));

        // Publishers
        target_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>("target_velocity", 10);
        target_distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>("target_distance", 10);
        current_state_publisher_ = this->create_publisher<std_msgs::msg::String>("current_state", 10);
        smoothed_distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>("smoothed_distance", 10);
        task_completed_publisher_ = this->create_publisher<std_msgs::msg::String>("task_completed", 10);

        // Initialization
        target_velocity_ = 0.0;
        target_distance_ = 30.0; // Default value, to be updated based on state
        last_detection_time_ = this->now();
        last_velocity_ = 0.0;

        // Initialize UDP socket
        udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (udp_socket_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
            rclcpp::shutdown();
        }

        // Setup the UDP server address structure
        server_address_.sin_family = AF_INET;
        server_address_.sin_port = htons(50131);  // Port number as provided
        server_address_.sin_addr.s_addr = inet_addr("172.23.60.118");  // IP address as provided

        RCLCPP_INFO(this->get_logger(), "Actor node initialized and UDP communication setup complete.");

        // Timer to periodically send UDP messages
        udp_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),  // Send UDP message every 100 ms
            [this]() {
                sendUDPMessage();  // Continuously send UDP messages
            }
        );

          // Timer for checking if IDLE state has lasted 2 seconds
        idle_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            [this]() {
                if (current_state_ == State::IDLE) {
                    std_msgs::msg::String task_msg;
                    task_msg.data = "Task Completed";
                    task_completed_publisher_->publish(task_msg);
                    RCLCPP_INFO(this->get_logger(), "Task Completed after staying in IDLE for 2 seconds.");
                }
            }
        );
    }

    ~ActorNode() {
        close(udp_socket_);
    }

private:
    enum class State {
        UNINITIALIZED,  // Initial state before receiving any velocity messages
        IDLE,
        ACCELERATING,
        CLOSINGGAP,
        DECELERATING,
        STOPPED,
        ATTACHING,
        EMERGENCY_STOP
    };

    State current_state_;
    int current_task_id_;  
    double target_velocity_;
    double target_distance_;
    rclcpp::Time last_detection_time_;
    double last_velocity_;
    double v_max_;
    bool has_published_idle_;

    size_t smoothing_window_size_;
    double detection_threshold_;
    std::deque<double> distance_history_;

    int udp_socket_;
    struct sockaddr_in server_address_;

    rclcpp::Subscription<observer_msgs::msg::ObserverInfo>::SharedPtr observer_info_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr order_info_subscription_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_velocity_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_distance_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_state_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr smoothed_distance_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_completed_publisher_;

    rclcpp::TimerBase::SharedPtr udp_timer_;
     rclcpp::TimerBase::SharedPtr idle_timer_;  // Timer for IDLE state

    uint8_t encodeVelocity(double velocity) {
        return static_cast<uint8_t>(std::min(std::max(velocity / 40.0 * 255.0, 0.0), 255.0));
    }

    uint8_t encodeDistance(double distance) {
        double max_distance = 255.0; // Example: 255 meters max
        return static_cast<uint8_t>(std::min(std::max(distance, 0.0), max_distance));
    }

    void sendUDPMessage() {
        RCLCPP_INFO(this->get_logger(), "Sending UDP - Target Velocity: %f, Target Distance: %f", target_velocity_, target_distance_);
        
        uint8_t encoded_velocity = encodeVelocity(target_velocity_);
        uint8_t encoded_distance = encodeDistance(target_distance_);

        uint8_t data[2] = {encoded_velocity, encoded_distance};
        
        ssize_t sent_bytes = sendto(udp_socket_, data, sizeof(data), 0,
                                    (struct sockaddr *)&server_address_, sizeof(server_address_));
        if (sent_bytes < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send data via UDP. Retrying...");
            for (int i = 0; i < 3; ++i) {
                sent_bytes = sendto(udp_socket_, data, sizeof(data), 0,
                                    (struct sockaddr *)&server_address_, sizeof(server_address_));
                if (sent_bytes >= 0) {
                    RCLCPP_INFO(this->get_logger(), "Successfully sent UDP message after retry.");
                    break;
                }
            }
            if (sent_bytes < 0) {
                RCLCPP_FATAL(this->get_logger(), "Failed to send UDP message after multiple attempts.");
            }
        } else {
            RCLCPP_INFO(this->get_logger(), "Sent UDP message: Velocity: %d, Distance: %d", data[0], data[1]);
        }
    }

    double smoothDistance(double new_distance) {
        if (std::isinf(new_distance)) {
            return std::numeric_limits<double>::infinity();  
        }

        if (!std::isnan(new_distance) && !std::isinf(new_distance)) {
            if (distance_history_.size() >= smoothing_window_size_) {
                distance_history_.pop_front();
            }
            distance_history_.push_back(new_distance);
        }

        if (distance_history_.empty()) {
            return 0.0;  
        }

        double sum = 0.0;
        for (double distance : distance_history_) {
            sum += distance;
        }

        return sum / distance_history_.size();
    }

    void publishCurrentState() {
        std_msgs::msg::String state_msg;
        switch (current_state_) {
            case State::UNINITIALIZED:
                // Do not publish anything in UNINITIALIZED state
                return;
            case State::IDLE:
                state_msg.data = "Idle";
                break;
            case State::ACCELERATING:
                state_msg.data = "Accelerating";
                break;
            case State::CLOSINGGAP:
                state_msg.data = "CLOSINGGAP";
                break;
            case State::DECELERATING:
                state_msg.data = "Decelerating";
                break;
            // case State::STOPPED:
            //     state_msg.data = "Stopped";
            //     break;
            case State::ATTACHING:
                state_msg.data = "Attaching";
                break;
            case State::EMERGENCY_STOP:
                state_msg.data = "Emergency Stop";
                break;
        }
        current_state_publisher_->publish(state_msg);
    }

    void publishTargetVelocityAndDistance() {
        std_msgs::msg::Float64 velocity_msg;
        velocity_msg.data = target_velocity_;
        target_velocity_publisher_->publish(velocity_msg);

        std_msgs::msg::Float64 distance_msg;
        distance_msg.data = target_distance_;
        target_distance_publisher_->publish(distance_msg);
    }

    void clearTaskCompletedMessage() {
        std_msgs::msg::String empty_msg;
        empty_msg.data = "";  // Publish an empty string to indicate the reset
        task_completed_publisher_->publish(empty_msg);
    }

void orderInfoCallback(const std_msgs::msg::String::SharedPtr msg) {
    try {
        // Extract TaskID
        std::string data = msg->data;
        std::size_t pos = data.find("TaskID: ");
        if (pos == std::string::npos) {
            throw std::invalid_argument("TaskID not found in the message");
        }

        std::string task_id_str = data.substr(pos + 8);
        task_id_str = task_id_str.substr(0, task_id_str.find_first_of(", \n\r"));

        int task_id = std::stoi(task_id_str);
        current_task_id_ = task_id;

        // Clear the "Task Completed" message if a new task is set
        if (task_id > 0) {
            clearTaskCompletedMessage();
            RCLCPP_INFO(this->get_logger(), "New task received, clearing 'Task Completed' state.");
        }

        // Emergency Stop
        if (task_id == 0) {
            current_state_ = State::EMERGENCY_STOP;
            target_velocity_ = 0.0;
            target_distance_ = 1.0;
            RCLCPP_INFO(this->get_logger(), "Switching to EMERGENCY_STOP state.");
        } else {
            // Regular Task Handling
            if (current_state_ == State::IDLE) {
                if (smoothed_distance_ > 40.0) {
                    current_state_ = State::ACCELERATING;
                    target_velocity_ = v_max_;
                    target_distance_ = 0.0;
                    RCLCPP_INFO(this->get_logger(), "Track free - switching to ACCELERATING state.");
                } else if (smoothed_distance_ > 10.0 && smoothed_distance_ < 40.0) {
                    current_state_ = State::CLOSINGGAP;
                    target_velocity_ = v_max_ / 2.0;
                    target_distance_ = 0.0;
                    RCLCPP_INFO(this->get_logger(), "Switching to CLOSINGGAP state.");
                } else if (task_id == 1 && smoothed_distance_ > 3.5 && smoothed_distance_ <= 10.0) {
                    current_state_ = State::ATTACHING;
                    target_velocity_ = 1.0;
                    target_distance_ = 0.0;
                    RCLCPP_INFO(this->get_logger(), "Switching to ATTACHING state.");
                }
                else {
                    current_state_ == State::IDLE;

                }
            }
        }

        publishTargetVelocityAndDistance();
        publishCurrentState();

    } catch (const std::invalid_argument& e) {
        RCLCPP_ERROR(this->get_logger(), "Invalid TaskID received: '%s'. Error: %s", msg->data.c_str(), e.what());
    } catch (const std::out_of_range& e) {
        RCLCPP_ERROR(this->get_logger(), "TaskID out of range: '%s'. Error: %s", msg->data.c_str(), e.what());
    }
}

void observerInfoCallback(const observer_msgs::msg::ObserverInfo::SharedPtr msg) {
    double smoothed_distance = smoothDistance(msg->distance);
    smoothed_distance_ = smoothed_distance;

    std_msgs::msg::Float64 smoothed_distance_msg;
    smoothed_distance_msg.data = smoothed_distance;
    smoothed_distance_publisher_->publish(smoothed_distance_msg);

    // Handle transitions based on real-time distance information

    // Transition from ACCELERATING to DECELERATING if an obstacle is detected within 40 meters
    if (current_state_ == State::ACCELERATING && smoothed_distance_ < 40.0) {
        current_state_ = State::DECELERATING;
        target_velocity_ = 0.0;
        target_distance_ = (smoothed_distance_ > 10.0) ? smoothed_distance_ - 10.0 : 0.0;
        RCLCPP_INFO(this->get_logger(), "Obstacle detected within 40 meters. Switching to DECELERATING state.");
    }

    // Transition from DECELERATING to ACCELERATING if the path clears
    if (current_state_ == State::DECELERATING && smoothed_distance_ > 40.0) {
        current_state_ = State::ACCELERATING;
        target_velocity_ = v_max_;
        target_distance_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Obstacle cleared, switching back to ACCELERATING.");
    }

      // Transition from DECELERATING to ACCELERATING if the path clears
    if (current_state_ == State::ATTACHING && smoothed_distance_ > 40.0) {
        current_state_ = State::ACCELERATING;
        target_velocity_ = v_max_;
        target_distance_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Obstacle cleared, switching back to ACCELERATING.");
    }


    // Handle CLOSINGGAP state: target is to stop 10 meters in front of the obstacle
    if (current_state_ == State::CLOSINGGAP) {
        target_distance_ = (smoothed_distance_ > 10.0) ? smoothed_distance_ - 10.0 : 1.0;

        // If Task ID = 1 and within 10 meters, transition to ATTACHING state
        if (smoothed_distance_ > 3.0 && smoothed_distance <= 10.0 && current_task_id_ == 1) {
            current_state_ = State::ATTACHING;
            target_velocity_ = 1.0;  // Slow speed for fine approach
            target_distance_ = (smoothed_distance_ > 3.0) ? smoothed_distance_ - 3.0 : 1.0;
            RCLCPP_INFO(this->get_logger(), "Task ID = 1, transitioning to ATTACHING state.");
        } else if (smoothed_distance_ <= 10.0 && current_task_id_ != 1) {
            // If not Task ID = 1, transition to DECELERATING to stop 10 meters before the obstacle
            current_state_ = State::DECELERATING;
            target_velocity_ = 0.0;
            target_distance_ = 1.0;
            RCLCPP_INFO(this->get_logger(), "Approaching obstacle, switching to DECELERATING state to stop 10 meters before obstacle.");
        }
    }

     // Handle transition to CLOSING GAP or ATTACHING after approaching
    if (current_state_ == State::IDLE && current_task_id_ == 1) {
        // within 10 meters, transition to ATTACHING state
        if (smoothed_distance_ > 3.0 && smoothed_distance_ && smoothed_distance_ <= 10.0) {
            current_state_ = State::ATTACHING;
            target_velocity_ = 1.0;  // Slow speed for fine approach
            target_distance_ = 0.0 ;
            RCLCPP_INFO(this->get_logger(), "Task ID = 1, transitioning to ATTACHING state.");
        } else if (smoothed_distance_ > 10.0 && smoothed_distance_ <= 20.0 ) {
            // If not Task ID = 1, transition to DECELERATING to stop 10 meters before the obstacle
            current_state_ = State::CLOSINGGAP;
            target_velocity_ = v_max_ / 2.0;
            target_distance_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Approaching obstacle, switching to DECELERATING state to stop 10 meters before obstacle.");
        }
    }

    publishTargetVelocityAndDistance();
    publishCurrentState();
}

void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double time_elapsed = (this->now() - last_detection_time_).seconds();
    double distance_travelled = last_velocity_ * time_elapsed;

    // Publish "IDLE" only after the first velocity message is received
    if (!has_published_idle_) {
        if (msg->linear.x == 0.0) {
            current_state_ = State::IDLE;
            has_published_idle_ = true;
            RCLCPP_INFO(this->get_logger(), "Vehicle velocity is 0. Publishing IDLE state.");
            publishCurrentState();
        }
        return;
    }

    // DECELERATING: Adjust target distance dynamically
    if (current_state_ == State::DECELERATING) {
        target_distance_ = smoothed_distance_ - 10.0;  // Continuously update target distance
        if (msg->linear.x == 0.0) {  // Vehicle has come to a stop
            current_state_ = State::IDLE;
            target_velocity_ = 0.0;
            target_distance_ = 1.0;
            RCLCPP_INFO(this->get_logger(), "Vehicle stopped. Switching to STOPPED state.");
        }
        publishTargetVelocityAndDistance();
        return;
    }

    // ATTACHING: Stop the vehicle when within 2.5 meters
    if (current_state_ == State::ATTACHING) {
        if (smoothed_distance_ <= 4.5) {
            target_velocity_ = 0.0;
            target_distance_ = 1.0;
            RCLCPP_INFO(this->get_logger(), "Reached target for ATTACHING. Switching to STOPPED.");
        } else {
            target_distance_ = std::max(target_distance_ - distance_travelled, 0.0);
        }
        publishTargetVelocityAndDistance();
        
    }

    //STOPPED to IDLE after ATTACHING (only when completely stopped)
    if (current_state_ == State::ATTACHING && msg->linear.x == 0.0 && smoothed_distance_ < 3.0) {
        current_state_ = State::IDLE;
        RCLCPP_INFO(this->get_logger(), "Vehicle has stopped after ATTACHING. Switching to IDLE state.");
        publishCurrentState();
       
    }
      //STOPPED to IDLE after ATTACHING (only when completely stopped)
    if (current_state_ == State::EMERGENCY_STOP && msg->linear.x == 0.0) {
        current_state_ = State::IDLE;
        RCLCPP_INFO(this->get_logger(), "Vehicle has stopped after ATTACHING. Switching to IDLE state.");
        publishCurrentState();
       
    }

    // STOPPED to IDLE after ensuring the vehicle has stopped
    // if (current_state_ == State::STOPPED && msg->linear.x == 0.0) {
    //     current_state_ = State::IDLE;
    //     RCLCPP_INFO(this->get_logger(), "Vehicle has stopped. Switching to IDLE state.");
    //     publishCurrentState();
    //     return;
    // }

    last_velocity_ = msg->linear.x;
    last_detection_time_ = this->now();

    publishTargetVelocityAndDistance();
    publishCurrentState();
}




//     void orderInfoCallback(const std_msgs::msg::String::SharedPtr msg) {
//         try {
//             // Extract TaskID
//             std::string data = msg->data;
//             std::size_t pos = data.find("TaskID: ");
//             if (pos == std::string::npos) {
//                 throw std::invalid_argument("TaskID not found in the message");
//             }

//             std::string task_id_str = data.substr(pos + 8);
//             task_id_str = task_id_str.substr(0, task_id_str.find_first_of(", \n\r"));

//             int task_id = std::stoi(task_id_str);
//             current_task_id_ = task_id;

//             // Clear the "Task Completed" message if a new task is set
//             if (task_id > 0) {
//                 clearTaskCompletedMessage();  // Publish an empty message
//                 RCLCPP_INFO(this->get_logger(), "New task received, clearing 'Task Completed' state.");
//             }
//               // Handle transition && to ACCELERATING if TaskID > 0 and vehicle is in IDLE state
//             if (current_state_ == State::IDLE && smoothed_distance_ > 40.0 && task_id > 0) {
//                 current_state_ = State::ACCELERATING;
//                 target_velocity_ = v_max_;      
//                 target_distance_ = 0; // Set target distance to 0.5 meters before the obstacle
//                 RCLCPP_INFO(this->get_logger(), "Track free - switching to ACCELERATING state.");
//             }

//               // Handle transition to CLOSING the GAP if TaskID > 0 and vehicle is in IDLE state
//             if (current_state_ == State::IDLE && smoothed_distance_ > 10 && smoothed_distance_ <= 40.0 && task_id > 0) {
//                 current_state_ = State::CLOSINGGAP;
//                 target_velocity_ = v_max_ / 2.0;  // Moderate speed for approaching
//                 target_distance_ = smoothed_distance_/2;
//                 RCLCPP_INFO(this->get_logger(), "Switching to ACCELERATING state to approach the target to 0.5 meters.");
//             }

//             // Handle transition to ATTACHING if TaskID = 1 and vehicle is in IDLE state
//             if (current_state_ == State::IDLE && smoothed_distance_ > 3.5 && smoothed_distance_ <= 10.0 && task_id == 1) {
//                 current_state_ = State::ATTACHING;
//                 target_velocity_ = 1.0;  // Moderate speed for approaching
//                 target_distance_ = 0; //smoothed_distance_ - 0.5; // Set target distance to 0.5 meters before the obstacle (offset Lidar - )
//                 RCLCPP_INFO(this->get_logger(), "Switching to ATTACING state.");
//             }
      

//             // Handle transition to ATTACHING if TaskID = 1 and vehicle is in IDLE state
//             if (task_id == 0) {
//                 current_state_ = State::EMERGENCY_STOP;
//                 target_velocity_ = 0;  // Stop 
//                 target_distance_ = 0.0;  // as soon as possible
//                 RCLCPP_INFO(this->get_logger(), "Switching to EM.STOP state.");
//             }
           

//             publishTargetVelocityAndDistance();
//             publishCurrentState();

//         } catch (const std::invalid_argument& e) {
//             RCLCPP_ERROR(this->get_logger(), "Invalid TaskID received: '%s'. Error: %s", msg->data.c_str(), e.what());
//         } catch (const std::out_of_range& e) {
//             RCLCPP_ERROR(this->get_logger(), "TaskID out of range: '%s'. Error: %s", msg->data.c_str(), e.what());
//         }
//     }
// void observerInfoCallback(const observer_msgs::msg::ObserverInfo::SharedPtr msg) {
//     double smoothed_distance = smoothDistance(msg->distance);
//     smoothed_distance_ = smoothed_distance;

//     std_msgs::msg::Float64 smoothed_distance_msg;
//     smoothed_distance_msg.data = smoothed_distance;
//     smoothed_distance_publisher_->publish(smoothed_distance_msg);

//     // Handle transitions based on real-time distance information

//     // Transition from ACCELERATING to DECELERATING if an obstacle is detected within 40 meters
//     if (current_state_ == State::ACCELERATING && smoothed_distance_ < 40.0) {
//         current_state_ = State::DECELERATING;
//         target_velocity_ = 0.0;
//         target_distance_ = smoothed_distance_ - 10.0;
//         RCLCPP_INFO(this->get_logger(), "Obstacle detected. Switching to DECELERATING state.");
//     }

//        // Check if the vehicle is in STOPPED state and Task ID = 1, then switch to ATTACHING
//     if (current_state_ == State::IDLE && current_task_id_ == 1 && smoothed_distance_ > 10 && smoothed_distance_ <= 15.0) {
//         current_state_ = State::CLOSINGGAP;
//         target_velocity_ = 1.0;
//         target_distance_ = 10.0;
//         RCLCPP_INFO(this->get_logger(), "Switching to CLOSINGGAP state.");
//     }
//     // Transition from CLOSINGGAP to DECELERATING as it approaches the obstacle
//     if (current_state_ == State::CLOSINGGAP && smoothed_distance_ <= 10.0) {
//         current_state_ = State::IDLE;
//         target_velocity_ = 0.0;
//         target_distance_ = 2.0;
//         RCLCPP_INFO(this->get_logger(), "Approaching obstacle, switching to DECELERATING state.");
//     }

//     // Check if the vehicle is in STOPPED state and Task ID = 1, then switch to ATTACHING
//     if (current_state_ == State::IDLE && current_task_id_ == 1 && smoothed_distance_ > 0.5 && smoothed_distance_ <= 10.0) {
//         current_state_ = State::ATTACHING;
//         target_velocity_ = 1.0;
//         target_distance_ = 0.0;
//         RCLCPP_INFO(this->get_logger(), "Switching to ATTACHING state.");
//     }

//     // Transition from ATTACHING to STOPPED when within 0.5 meters of the obstacle
//     if (current_state_ == State::ATTACHING && smoothed_distance_ <= 3.5) {
//         current_state_ = State::IDLE;
//         target_velocity_ = 0.0;
//         target_distance_ = 2.0;
//         RCLCPP_INFO(this->get_logger(), "Reached target for ATTACHING. Switching to STOPPED.");
        
//           }
//         rclcpp::sleep_for(std::chrono::seconds(2));  // Wait for 2 seconds
//         current_state_ = State::IDLE;
//         RCLCPP_INFO(this->get_logger(), "Task Completed after stopping at the obstacle.");
//         std_msgs::msg::String task_msg;
//         task_msg.data = "Task Completed";
//         task_completed_publisher_->publish(task_msg);
  

//     // Transition from DECELERATING to ACCELERATING if the path clears
//     if (current_state_ == State::DECELERATING && smoothed_distance_ > 40.0) {
//         current_state_ = State::ACCELERATING;
//         target_velocity_ = v_max_;
//         target_distance_ = 0.0;
//         RCLCPP_INFO(this->get_logger(), "Obstacle cleared, switching back to ACCELERATING.");
//     }

//     publishTargetVelocityAndDistance();
//     publishCurrentState();
// }

// void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
//     double time_elapsed = (this->now() - last_detection_time_).seconds();
//     double distance_travelled = last_velocity_ * time_elapsed;

//     // Publish "IDLE" only after the first velocity message is received
//     if (!has_published_idle_) {
//         if (msg->linear.x == 0.0) {
//             current_state_ = State::IDLE;
//             has_published_idle_ = true;
//             RCLCPP_INFO(this->get_logger(), "Vehicle velocity is 0. Publishing IDLE state.");
//             publishCurrentState();
//         }
//         return;
//     }

//     // Handle dynamic updates while in DECELERATING state
//     if (current_state_ == State::DECELERATING) {
//         target_distance_ = smoothed_distance_ - 10.0;  // Continuously update target distance
//         if (msg->linear.x == 0.0) {  // Vehicle has come to a stop
//             current_state_ = State::IDLE;
//             target_velocity_ = 0.0;
//             target_distance_ = 0.5;
//             RCLCPP_INFO(this->get_logger(), "Vehicle stopped. Switching to STOPPED state.");
//         }
//         publishTargetVelocityAndDistance();
//         return;
//     }

//     // Handle dynamic updates while in ATTACHING state
//     if (current_state_ == State::ATTACHING) {
//         if (smoothed_distance_ <= 0.5) {
//             current_state_ = State::STOPPED;
//             target_velocity_ = 0.0;
//             target_distance_ = 0.0;
//             RCLCPP_INFO(this->get_logger(), "Reached target for ATTACHING. Switching to STOPPED.");
//         } else {
//             target_distance_ = std::max(target_distance_ - distance_travelled, 0.0);
//         }
//         publishTargetVelocityAndDistance();
//         return;
//     }

//     // Handle transition from STOPPED to IDLE
//     if (current_state_ == State::STOPPED && msg->linear.x == 0.0) {
//         current_state_ = State::IDLE;
//         RCLCPP_INFO(this->get_logger(), "Vehicle has stopped. Switching to IDLE state.");
//         publishCurrentState();
//     }

//     last_velocity_ = msg->linear.x;
//     last_detection_time_ = this->now();

//     publishTargetVelocityAndDistance();
//     publishCurrentState();
// }


    // void observerInfoCallback(const observer_msgs::msg::ObserverInfo::SharedPtr msg) {
    //     double smoothed_distance = smoothDistance(msg->distance);
    //     smoothed_distance_ = smoothed_distance;

    //     std_msgs::msg::Float64 smoothed_distance_msg;
    //     smoothed_distance_msg.data = smoothed_distance;
    //     smoothed_distance_publisher_->publish(smoothed_distance_msg);

    //     // Handle ACCELERATING to DECELERATING transition if the obstacle disappears
    //     if (current_state_ == State::ACCELERATING && smoothed_distance_ < 40.0) {
    //         current_state_ = State::DECELERATING;
    //         target_velocity_ = 0.0;
    //         target_distance_ = smoothed_distance - 10.0; // stap 10 m in front of an obstacle
    //         RCLCPP_INFO(this->get_logger(), "Obstacle cleared, switching back to ACCELERATING.");
    //         publishTargetVelocityAndDistance();
    //         publishCurrentState();
    //         return;
    //     }
    //     if (current_state_ == State::DECELERATING && smoothed_distance_ <= 10.0) {
    //         current_state_ = State::STOPPED;
    //         target_velocity_ = 0.0;
    //         target_distance_ = 0.0; 
    //         RCLCPP_INFO(this->get_logger(), "Stopped after getting closer to an obstacle");
    //         publishTargetVelocityAndDistance();
    //         publishCurrentState();
    //         return;
    //     }

    //        // Handle transition to CLOSING the GAP if TaskID > 0 and vehicle is in IDLE state
    //     if (current_state_ == State::CLOSINGGAP && smoothed_distance_ > 10 ) {
    //             current_state_ = State::DECELERATING;
    //             target_velocity_ = 0;  // Moderate speed for approaching
    //             target_distance_ = smoothed_distance_ - 10.0;
    //             RCLCPP_INFO(this->get_logger(), "Switching to DECELERATING state to stop 10 meters in front of obstacle.");
    //            publishTargetVelocityAndDistance();
    //         publishCurrentState();
    //         return;
    //         }

    //     // Handle DECELERATING to ACCELERATING transition if the obstacle disappears
    //     if (current_state_ == State::DECELERATING && smoothed_distance_ > 40.0) {
    //         current_state_ = State::ACCELERATING;
    //         target_velocity_ = v_max_;
    //         target_distance_ = 0.0;
    //         RCLCPP_INFO(this->get_logger(), "Obstacle cleared, switching back to ACCELERATING.");
    //         publishTargetVelocityAndDistance();
    //         publishCurrentState();
    //         return;
    //     }
    //    // Handle transition to ATTACHING if TaskID = 1 and vehicle is in IDLE state
    //     if (current_state_ == State::ATTACHING && smoothed_distance_ < 0.5) {
    //             current_state_ = State::STOPPED;
    //             target_velocity_ = 0;  // Stop at the obstacle
    //             target_distance_ = 0;  // right now
    //             RCLCPP_INFO(this->get_logger(), "Switching to STOPPED state.");
    //             publishTargetVelocityAndDistance();
    //         publishCurrentState();
    //         return;
    //         }

    //     if (current_state_ == State::IDLE && smoothed_distance_ > 10 && smoothed_distance_ <= 40.0) {
    //             current_state_ = State::CLOSINGGAP;
    //             target_velocity_ = v_max_ / 2.0;  // Moderate speed for approaching
    //             target_distance_ = smoothed_distance_/2;
    //             RCLCPP_INFO(this->get_logger(), "Switching to ACCELERATING state to approach the target to 0.5 meters.");
    //         }
    //               // Handle transition to CLOSING the GAP if TaskID > 0 and vehicle is in IDLE state
    //     if (current_state_ == State::CLOSINGGAP && smoothed_distance_ > 10 ) {
    //             current_state_ = State::DECELERATING;
    //             target_velocity_ = 0;  // Moderate speed for approaching
    //             target_distance_ = smoothed_distance_ - 10.0;
    //             RCLCPP_INFO(this->get_logger(), "Switching to DECELERATING state to stop 10 meters in front of obstacle.");
    //         }

    //     // Update target distance dynamically based on new smoothed distance ____________________________!!!!!!!!!!!!!!!!!!!!!!!!!
    //     // if (current_state_ == State::ACCELERATING) {
    //     //     if (smoothed_distance <= 10.0) {
    //     //         current_state_ = State::ATTACHING;
    //     //         target_velocity_ = 1.0;  // Reduce speed for fine approach
    //     //         target_distance_ = smoothed_distance_ - 0.5; // Stop 0.5 meters before the obstacle
    //     //         RCLCPP_INFO(this->get_logger(), "Approaching within 1 meter. Switching to ATTACHING state to stop 0.5 meters before the obstacle.");
    //     //         publishTargetVelocityAndDistance();
    //     //         publishCurrentState();
    //     //         return;
    //     //     }
    //     // }

    //     // if (smoothed_distance < 40.0 && velocity_subscription_.linear.x > 0.0) {
    //     //     current_state_ = State::DECELERATING;
    //     //     target_velocity_ = 0.0;
    //     //     target_distance_ = smoothed_distance - 10.0;
    //     //     RCLCPP_INFO(this->get_logger(), "Obstacle detected within 40 meters. Decelerating to stop.");
    //     // }

    //     publishTargetVelocityAndDistance();
    //     publishCurrentState();
    // }

    // void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    //     double time_elapsed = (this->now() - last_detection_time_).seconds();
    //     double distance_travelled = last_velocity_ * time_elapsed;

    //     // Publish "IDLE" only after the first velocity message is received
    //     if (!has_published_idle_) {
    //         if (msg->linear.x == 0.0) {
    //             current_state_ = State::IDLE;
    //             has_published_idle_ = true;
    //             RCLCPP_INFO(this->get_logger(), "Vehicle velocity is 0. Publishing IDLE state.");
    //             publishCurrentState();
    //         }
    //         return;
    //     }

    //     if (current_state_ == State::ACCELERATING && msg->linear.x >= v_max_/3.6) {
    //         // Stay in ACCELERATING state if target velocity hasn't been reached
    //         RCLCPP_INFO(this->get_logger(), "Continuing to ACCELERATE. Current velocity: %f", msg->linear.x);
    //     }

    //     if (current_state_ == State::DECELERATING) {
    //         target_distance_ = std::max(target_distance_ - distance_travelled, 0.0);
    //         publishTargetVelocityAndDistance();
    //     }

    //     if (current_task_id_ == 0 && current_state_ == State::EMERGENCY_STOP) {
    //         target_distance_ = std::max(target_distance_ - msg->linear.x * time_elapsed, 0.0);
    //         if (target_distance_ <= 0.0) {
    //             current_state_ = State::STOPPED;
    //             target_velocity_ = 0.0;
    //             target_distance_ = 0.0;
    //             RCLCPP_INFO(this->get_logger(), "Emergency stop completed. Vehicle stopped.");
    //         }
    //         publishTargetVelocityAndDistance();
    //         return;
    //     }

    //     if (current_state_ == State::DECELERATING && msg->linear.x == 0.0) {
    //         current_state_ = State::STOPPED;
    //         target_velocity_ = 0.0;
    //         target_distance_ = 0.0;
    //         RCLCPP_INFO(this->get_logger(), "Vehicle stopped. Switching to STOPPED state.");

    //         rclcpp::sleep_for(std::chrono::seconds(2));  // Wait for 2 seconds
    //         if (current_state_ == State::STOPPED) {
    //             current_state_ = State::IDLE;
    //             RCLCPP_INFO(this->get_logger(), "Task Completed after stopping for 2 seconds.");
    //             std_msgs::msg::String task_msg;
    //             task_msg.data = "Task Completed";
    //             task_completed_publisher_->publish(task_msg);
    //         }
    //     } else if (current_state_ == State::ATTACHING && target_distance_ == 0.0) {
    //         current_state_ = State::STOPPED;
    //         target_velocity_ = 0.0;
    //         target_distance_ = 0.0;
    //         RCLCPP_INFO(this->get_logger(), "Reached target for ATTACHING. Switching to STOPPED.");

    //         current_state_ = State::IDLE;
    //         RCLCPP_INFO(this->get_logger(), "Task Completed");
    //         std_msgs::msg::String task_msg;
    //         task_msg.data = "Task Completed";
    //         task_completed_publisher_->publish(task_msg);
    //     } else if (std::isinf(smoothed_distance_) && msg->linear.x == 0.0 && current_task_id_ > 0) {
    //         current_state_ = State::ACCELERATING;
    //         target_velocity_ = v_max_;
    //         target_distance_ = 30.0;
    //         RCLCPP_INFO(this->get_logger(), "No obstacles detected and vehicle stopped. Switching to ACCELERATING state.");
    //     }

    //     last_velocity_ = msg->linear.x;
    //     last_detection_time_ = this->now();

    //     publishTargetVelocityAndDistance();
    //     publishCurrentState();
    // }

    double smoothed_distance_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ActorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;

}






// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/string.hpp>
// #include <std_msgs/msg/float64.hpp>
// #include <geometry_msgs/msg/twist.hpp>
// #include <observer_msgs/msg/observer_info.hpp>
// #include <sys/socket.h>
// #include <netinet/in.h>
// #include <arpa/inet.h>
// #include <unistd.h>
// #include <deque>
// #include <chrono>
// #include <limits>
// #include <stdexcept>
// #include <string>
// #include <cstring>

// class ActorNode : public rclcpp::Node {
// public:
//     ActorNode() : Node("actor_node"), current_state_(State::Idle), 
//                   smoothing_window_size_(10), detection_threshold_(40.0), 
//                   last_velocity_(0.0), v_max_(5.0) {

//         current_task_id_ = -1;  // Initialize TaskID to -1 (no task received)

//         // Subscriptions
//         observer_info_subscription_ = this->create_subscription<observer_msgs::msg::ObserverInfo>(
//             "observer_info", 10, std::bind(&ActorNode::observerInfoCallback, this, std::placeholders::_1));
        
//         velocity_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
//             "vehicle_velocity", 10, std::bind(&ActorNode::velocityCallback, this, std::placeholders::_1));
        
//         order_info_subscription_ = this->create_subscription<std_msgs::msg::String>(
//             "order_info", 10, std::bind(&ActorNode::orderInfoCallback, this, std::placeholders::_1));

//         // Publishers
//         target_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>("target_velocity", 10);
//         target_distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>("target_distance", 10);
//         current_state_publisher_ = this->create_publisher<std_msgs::msg::String>("current_state", 10);
//         smoothed_distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>("smoothed_distance", 10);
//         task_completed_publisher_ = this->create_publisher<std_msgs::msg::String>("task_completed", 10);

//         // Initialization
//         target_velocity_ = 0.0;
//         target_distance_ = 30.0; // Default value, to be updated based on state
//         last_detection_time_ = this->now();
//         last_velocity_ = 0.0;

//         // Initialize UDP socket
//         udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
//         if (udp_socket_ < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
//             rclcpp::shutdown();
//         }

//         // Setup the UDP server address structure
//         server_address_.sin_family = AF_INET;
//         server_address_.sin_port = htons(50131);  // Port number as provided
//         server_address_.sin_addr.s_addr = inet_addr("172.23.60.118");  // IP address as provided

//         RCLCPP_INFO(this->get_logger(), "Actor node initialized and UDP communication setup complete.");

//         // Timer to periodically send UDP messages
//         udp_timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(100),  // Send UDP message every 100 ms
//             [this]() {
//                 sendUDPMessage();  // Continuously send UDP messages
//             }
//         );
//     }

//     ~ActorNode() {
//         close(udp_socket_);
//     }

// private:
//     enum class State {
//         Idle,
//         Accelerating,
//         Cruising,
//         Coasting,
//         Decelerating,
//         Stopped,
//         Attaching,
//         EmergencyStop
//     };

//     State current_state_;
//     int current_task_id_;  
//     double target_velocity_;
//     double target_distance_;
//     rclcpp::Time last_detection_time_;
//     double last_velocity_;
//     double v_max_;

//     size_t smoothing_window_size_;
//     double detection_threshold_;
//     std::deque<double> distance_history_;

//     int udp_socket_;
//     struct sockaddr_in server_address_;

//     rclcpp::Subscription<observer_msgs::msg::ObserverInfo>::SharedPtr observer_info_subscription_;
//     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr order_info_subscription_;

//     rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_velocity_publisher_;
//     rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_distance_publisher_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_state_publisher_;
//     rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr smoothed_distance_publisher_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_completed_publisher_;

//     rclcpp::TimerBase::SharedPtr udp_timer_;

//     uint8_t encodeVelocity(double velocity) {
//         return static_cast<uint8_t>(std::min(std::max(velocity / 40.0 * 255.0, 0.0), 255.0));
//     }

//     uint8_t encodeDistance(double distance) {
//         double max_distance = 255.0; // Example: 255 meters max
//         return static_cast<uint8_t>(std::min(std::max(distance, 0.0), max_distance));
//     }

//     void sendUDPMessage() {
//         RCLCPP_INFO(this->get_logger(), "Sending UDP - Target Velocity: %f, Target Distance: %f", target_velocity_, target_distance_);
        
//         uint8_t encoded_velocity = encodeVelocity(target_velocity_);
//         uint8_t encoded_distance = encodeDistance(target_distance_);

//         uint8_t data[2] = {encoded_velocity, encoded_distance};
        
//         ssize_t sent_bytes = sendto(udp_socket_, data, sizeof(data), 0,
//                                     (struct sockaddr *)&server_address_, sizeof(server_address_));
//         if (sent_bytes < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to send data via UDP. Retrying...");
//             for (int i = 0; i < 3; ++i) {
//                 sent_bytes = sendto(udp_socket_, data, sizeof(data), 0,
//                                     (struct sockaddr *)&server_address_, sizeof(server_address_));
//                 if (sent_bytes >= 0) {
//                     RCLCPP_INFO(this->get_logger(), "Successfully sent UDP message after retry.");
//                     break;
//                 }
//             }
//             if (sent_bytes < 0) {
//                 RCLCPP_FATAL(this->get_logger(), "Failed to send UDP message after multiple attempts.");
//             }
//         } else {
//             RCLCPP_INFO(this->get_logger(), "Sent UDP message: Velocity: %d, Distance: %d", data[0], data[1]);
//         }
//     }

//     double smoothDistance(double new_distance) {
//         if (std::isinf(new_distance)) {
//             return std::numeric_limits<double>::infinity();  
//         }

//         if (!std::isnan(new_distance) && !std::isinf(new_distance)) {
//             if (distance_history_.size() >= smoothing_window_size_) {
//                 distance_history_.pop_front();
//             }
//             distance_history_.push_back(new_distance);
//         }

//         if (distance_history_.empty()) {
//             return 0.0;  
//         }

//         double sum = 0.0;
//         for (double distance : distance_history_) {
//             sum += distance;
//         }

//         return sum / distance_history_.size();
//     }

//     void publishCurrentState() {
//         std_msgs::msg::String state_msg;
//         switch (current_state_) {
//             case State::Idle:
//                 state_msg.data = "Idle";
//                 break;
//             case State::Accelerating:
//                 state_msg.data = "Accelerating";
//                 break;
//             case State::Cruising:
//                 state_msg.data = "Cruising";
//                 break;
//             case State::Coasting:
//                 state_msg.data = "Coasting";
//                 break;
//             case State::Decelerating:
//                 state_msg.data = "Decelerating";
//                 break;
//             case State::Stopped:
//                 state_msg.data = "Stopped";
//                 break;
//             case State::Attaching:
//                 state_msg.data = "Attaching";
//                 break;
//             case State::EmergencyStop:
//                 state_msg.data = "EmergencyStop";
//                 break;
//         }
//         current_state_publisher_->publish(state_msg);
//     }

//     void publishTargetVelocityAndDistance() {
//         std_msgs::msg::Float64 velocity_msg;
//         velocity_msg.data = target_velocity_;
//         target_velocity_publisher_->publish(velocity_msg);

//         std_msgs::msg::Float64 distance_msg;
//         distance_msg.data = target_distance_;
//         target_distance_publisher_->publish(distance_msg);
//     }

//     void orderInfoCallback(const std_msgs::msg::String::SharedPtr msg) {
//     try {
//         // Extract TaskID
//         std::string data = msg->data;
//         std::size_t pos = data.find("TaskID: ");
//         if (pos == std::string::npos) {
//             throw std::invalid_argument("TaskID not found in the message");
//         }
        
//         std::string task_id_str = data.substr(pos + 8);
//         task_id_str = task_id_str.substr(0, task_id_str.find_first_of(", \n\r")); 

//         int task_id = std::stoi(task_id_str);
//         current_task_id_ = task_id;  

//         if (task_id == 0) {
//             RCLCPP_INFO(this->get_logger(), "Emergency stop task detected.");
//             current_state_ = State::EmergencyStop;
//             target_velocity_ = 0.0;
//             target_distance_ = 5.0;  // Emergency stop target distance
//             publishTargetVelocityAndDistance();
//             publishCurrentState();
//             return;
//         }

//         if (task_id == 1 && smoothed_distance_ <= 15.0) {
//             current_state_ = State::Attaching;
//             target_velocity_ = 1.0;
//             target_distance_ = smoothed_distance_-2.2; //Distance Scanner - Pufferbohle
//             RCLCPP_INFO(this->get_logger(), "Switching to ATTACHING state due to TaskID 1.");
//         } else if (current_state_ == State::Idle && task_id > 0) {
//             current_state_ = State::Accelerating;
//             target_velocity_ = 5.0;
//             target_distance_ = 30.0;
//             RCLCPP_INFO(this->get_logger(), "Valid task received. Target velocity: 5 km/h.");
//         }
//         publishTargetVelocityAndDistance();
//         publishCurrentState();

//     } catch (const std::invalid_argument& e) {
//         RCLCPP_ERROR(this->get_logger(), "Invalid TaskID received: '%s'. Error: %s", msg->data.c_str(), e.what());
//     } catch (const std::out_of_range& e) {
//         RCLCPP_ERROR(this->get_logger(), "TaskID out of range: '%s'. Error: %s", msg->data.c_str(), e.what());
//     }
// }


//     void observerInfoCallback(const observer_msgs::msg::ObserverInfo::SharedPtr msg) {
//         double smoothed_distance = smoothDistance(msg->distance);
//         smoothed_distance_ = smoothed_distance;

//         std_msgs::msg::Float64 smoothed_distance_msg;
//         smoothed_distance_msg.data = smoothed_distance;
//         smoothed_distance_publisher_->publish(smoothed_distance_msg);

//         if (current_task_id_ == 0 && current_state_ == State::EmergencyStop) {
//             // Decrease target distance as the vehicle travels forward during emergency stop
//             target_distance_ = std::max(target_distance_ - msg->distance, 0.0);
//             publishTargetVelocityAndDistance();
//             return;
//         }

//         if (smoothed_distance > 40.0 && target_velocity_ < v_max_ && current_task_id_ > 0) { //changed 40 to 10
//             current_state_ = State::Accelerating;
//             target_velocity_ = v_max_;
//             target_distance_ = 25.0;  // Delta distance to reach v_max
//             RCLCPP_INFO(this->get_logger(), "Path is clear, accelerating. Target velocity: 5 km/h, Target distance: 25 meters.");
//         } else if (smoothed_distance < 40.0 && target_velocity_ > 0.0) {
//             current_state_ = State::Decelerating;
//             target_velocity_ = 0.0;
//             if (current_task_id_ == 1) {
//                 target_distance_ = smoothed_distance - 10.0;
//             }
//             RCLCPP_INFO(this->get_logger(), "Obstacle detected within 30 meters. Decelerating to stop.");
//         }

//         if (smoothed_distance < 10.0 && current_task_id_ == 1 && current_state_ != State::Attaching) {
//         current_state_ = State::Attaching;
//         target_velocity_ = 1.0;
//         target_distance_ = smoothed_distance_;
//         RCLCPP_INFO(this->get_logger(), "Small smoothed distance detected. Switching to ATTACHING.");
//     }

//         publishTargetVelocityAndDistance();
//         publishCurrentState();
//     }

//     void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
//         double time_elapsed = (this->now() - last_detection_time_).seconds();
//         double distance_travelled = last_velocity_ * time_elapsed;

//         if (current_state_ == State::Accelerating && msg->linear.x >= v_max_/3.6) {
//             current_state_ = State::Cruising;
//             target_velocity_ = v_max_;
//             target_distance_ = 0.0;
//             RCLCPP_INFO(this->get_logger(), "Reached maximum velocity. Switching to CRUISING state.");
//         }

//         if (current_state_ == State::Decelerating) {
//             target_distance_ = std::max(target_distance_ - distance_travelled, 0.0);
//             publishTargetVelocityAndDistance();
//         }

//         if (current_task_id_ == 0 && current_state_ == State::EmergencyStop) {
//             // Emergency stop handling: Decrease target distance as the vehicle travels
//             target_distance_ = std::max(target_distance_ - msg->linear.x * time_elapsed, 0.0);
//             if (target_distance_ <= 0.0) {
//                 current_state_ = State::Stopped;
//                 target_velocity_ = 0.0;
//                 target_distance_ = 0.0;
//                 RCLCPP_INFO(this->get_logger(), "Emergency stop completed. Vehicle stopped.");
//             }
//             publishTargetVelocityAndDistance();
//             return;
//         }

//         if (current_state_ == State::Decelerating && msg->linear.x == 0.0) {
//             current_state_ = State::Stopped;
//             target_velocity_ = 0.0;
//             target_distance_ = 0.0;
//             RCLCPP_INFO(this->get_logger(), "Velocity is 0. Switching to Stopped state.");

//             auto stop_time = this->now();
//             rclcpp::sleep_for(std::chrono::seconds(2));
//             if (current_state_ == State::Stopped) {
//                 current_state_ = State::Idle;
//                 RCLCPP_INFO(this->get_logger(), "Task Completed after stopping for 2 seconds.");
//                 std_msgs::msg::String task_msg;
//                 task_msg.data = "Task Completed";
//                 task_completed_publisher_->publish(task_msg);
//             }
//         } else if (current_state_ == State::Attaching && target_distance_ == 0.0) {
//             current_state_ = State::Stopped;
//             target_velocity_ = 0.0;
//             target_distance_ = 0.0;
//             RCLCPP_INFO(this->get_logger(), "Reached target for ATTACHING. Switching to STOPPED.");

//             current_state_ = State::Idle;
//             RCLCPP_INFO(this->get_logger(), "Task Completed");
//             std_msgs::msg::String task_msg;
//             task_msg.data = "Task Completed";
//             task_completed_publisher_->publish(task_msg);
//         } else if (std::isinf(smoothed_distance_) && msg->linear.x == 0.0 && current_task_id_ > 0) {
//             current_state_ = State::Accelerating;
//             target_velocity_ = v_max_;
//             target_distance_ = 30.0;
//             RCLCPP_INFO(this->get_logger(), "No obstacles detected and vehicle stopped. Switching to ACCELERATING state.");
//         }

//         last_velocity_ = msg->linear.x;
//         last_detection_time_ = this->now();

//         publishTargetVelocityAndDistance();
//         publishCurrentState();
//     }

//     double smoothed_distance_;
// };

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<ActorNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }


// #include <rclcpp/rclcpp.hpp>
// #include <std_msgs/msg/string.hpp>
// #include "std_msgs/msg/float64.hpp"
// #include <geometry_msgs/msg/twist.hpp>
// #include <observer_msgs/msg/observer_info.hpp>
// #include <sys/socket.h>
// #include <netinet/in.h>
// #include <arpa/inet.h>
// #include <unistd.h>
// #include <deque>
// #include <chrono>
// #include <limits>
// #include <stdexcept>
// #include <string>

// class ActorNode : public rclcpp::Node {
// public:
//     ActorNode() : Node("actor_node"), current_state_(State::Idle), 
//                   smoothing_window_size_(10), detection_threshold_(30.0) {

//         current_task_id_ = 0;  // Initialize TaskID to 0 (default)

//         // Subscriptions
//         observer_info_subscription_ = this->create_subscription<observer_msgs::msg::ObserverInfo>(
//             "observer_info", 10, std::bind(&ActorNode::observerInfoCallback, this, std::placeholders::_1));
        
//         velocity_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
//             "vehicle_velocity", 10, std::bind(&ActorNode::velocityCallback, this, std::placeholders::_1));
        
//         order_info_subscription_ = this->create_subscription<std_msgs::msg::String>(
//             "order_info", 10, std::bind(&ActorNode::orderInfoCallback, this, std::placeholders::_1));

//         // Publishers
//         target_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>("target_velocity", 10);
//         target_distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>("target_distance", 10);
//         current_state_publisher_ = this->create_publisher<std_msgs::msg::String>("current_state", 10);
//         smoothed_distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>("smoothed_distance", 10);

        
//         // Initialization
//         target_velocity_ = 0.0;
//         target_distance_ = 0.0;
//         last_detection_time_ = this->now();

//         // Initialize UDP socket
//         udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
//         if (udp_socket_ < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
//             rclcpp::shutdown();
//         }

//         // Setup the UDP server address structure
//         server_address_.sin_family = AF_INET;
//         server_address_.sin_port = htons(50131);  // Port number as provided
//         server_address_.sin_addr.s_addr = inet_addr("172.23.60.118");  // IP address as provided

//         RCLCPP_INFO(this->get_logger(), "Actor node initialized and UDP communication setup complete.");

//         // Timer to periodically send UDP messages
//         udp_timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(100),  // Send UDP message every 100 ms
//             std::bind(&ActorNode::sendUDPMessage, this)
//         );
//     }

//     ~ActorNode() {
//         close(udp_socket_);
//     }

// private:
//     // Enum for the different states
//     enum class State {
//         Idle,
//         Accelerating,
//         Cruising,
//         Decelerating,
//         Stopped,
//         Attaching,
//         EmergencyStop
//     };

//     State current_state_;
//     int current_task_id_;  // Store the current TaskID
//     double target_velocity_;
//     double target_distance_;
//     rclcpp::Time last_detection_time_;

//     size_t smoothing_window_size_;
//     double detection_threshold_;
//     std::deque<double> distance_history_;

//     int udp_socket_;
//     struct sockaddr_in server_address_;

//     rclcpp::Subscription<observer_msgs::msg::ObserverInfo>::SharedPtr observer_info_subscription_;
//     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr order_info_subscription_;

//     rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_velocity_publisher_;
//     rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_distance_publisher_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_state_publisher_;
//     rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr smoothed_distance_publisher_;


//     rclcpp::TimerBase::SharedPtr udp_timer_;

//     uint8_t encodeVelocity(double velocity) {
//         return static_cast<uint8_t>(std::min(std::max(velocity / 40.0 * 255.0, 0.0), 255.0));
//     }

//     uint8_t encodeDistance(double distance) {
//         return static_cast<uint8_t>(std::min(std::max(distance / 255.0, 0.0), 255.0));
//     }

//     void sendUDPMessage() {
//         // Check if data is available
//         if (std::isnan(target_velocity_) || std::isinf(target_velocity_)) {
//             target_velocity_ = 0.0;  // Invalid data, reset to 0
//         }

//         if (std::isnan(target_distance_) || std::isinf(target_distance_)) {
//             target_distance_ = 0.0;  // Invalid data, reset to 0
//         }

//         uint8_t encoded_velocity = encodeVelocity(target_velocity_);
//         uint8_t encoded_distance = encodeDistance(target_distance_);

//         uint8_t data[2] = {encoded_velocity, encoded_distance};
        
//         ssize_t sent_bytes = sendto(udp_socket_, data, sizeof(data), 0,
//                                     (struct sockaddr *)&server_address_, sizeof(server_address_));
//         if (sent_bytes < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to send data via UDP");
//         } else {
//             RCLCPP_INFO(this->get_logger(), "Sent UDP message: Velocity: %d, Distance: %d", data[0], data[1]);
//         }
//     }

//     double smoothDistance(double new_distance) {
//     // Check if the new distance is infinity
//     if (std::isinf(new_distance)) {
//         return std::numeric_limits<double>::infinity();  // Pass through infinity
//     }

//     // Add new distance to history only if it's valid
//     if (!std::isnan(new_distance) && !std::isinf(new_distance)) {
//         if (distance_history_.size() >= smoothing_window_size_) {
//             distance_history_.pop_front();
//         }
//         distance_history_.push_back(new_distance);
//     }

//     if (distance_history_.empty()) {
//         return 0.0;  // Default to 0 if no valid distances are available
//     }

//     double sum = 0.0;
//     for (double distance : distance_history_) {
//         sum += distance;
//     }

//     return sum / distance_history_.size();
// }


//     void publishCurrentState() {
//         std_msgs::msg::String state_msg;
//         switch (current_state_) {
//             case State::Idle:
//                 state_msg.data = "Idle";
//                 break;
//             case State::Accelerating:
//                 state_msg.data = "Accelerating";
//                 break;
//             case State::Cruising:
//                 state_msg.data = "Cruising";
//                 break;
//             case State::Decelerating:
//                 state_msg.data = "Decelerating";
//                 break;
//             case State::Stopped:
//                 state_msg.data = "Stopped";
//                 break;
//             case State::Attaching:
//                 state_msg.data = "Attaching";
//                 break;
//             case State::EmergencyStop:
//                 state_msg.data = "EmergencyStop";
//                 break;
//         }
//         current_state_publisher_->publish(state_msg);
//     }

//     void publishTargetVelocityAndDistance() {
//         std_msgs::msg::Float64 velocity_msg;
//         velocity_msg.data = target_velocity_;
//         target_velocity_publisher_->publish(velocity_msg);

//         std_msgs::msg::Float64 distance_msg;
//         distance_msg.data = target_distance_;
//         target_distance_publisher_->publish(distance_msg);
//     }

//     void orderInfoCallback(const std_msgs::msg::String::SharedPtr msg) {
//     try {
//         // Extract the "TaskID" from the received string
//         std::string data = msg->data;
//         std::size_t pos = data.find("TaskID: ");
//         if (pos == std::string::npos) {
//             throw std::invalid_argument("TaskID not found in the message");
//         }
        
//         // Extract the TaskID value from the string
//         std::string task_id_str = data.substr(pos + 8);  // 8 is the length of "TaskID: "
//         task_id_str = task_id_str.substr(0, task_id_str.find_first_of(", \n\r"));  // Trim to the next delimiter

//         // Convert the extracted TaskID to an integer
//         int task_id = std::stoi(task_id_str);
//         current_task_id_ = task_id;  // Store the current TaskID

//         if (task_id == 1 && current_state_ == State::Stopped) {
//             current_state_ = State::Attaching;
//             target_velocity_ = 1.0;
//             RCLCPP_INFO(this->get_logger(), "Attaching mode activated. Moving at 1 km/h.");
//             sendUDPMessage();
//             publishTargetVelocityAndDistance();
//             publishCurrentState();
//         } else if (task_id == 0 && current_state_ == State::Stopped) {
//             rclcpp::sleep_for(std::chrono::seconds(5));
//             current_state_ = State::Accelerating;
//             target_velocity_ = 5.0;
//             target_distance_ = 20.0;
//             RCLCPP_INFO(this->get_logger(), "Resuming movement with target velocity 5 km/h and target distance 20 meters.");
//             sendUDPMessage();
//             publishTargetVelocityAndDistance();
//             publishCurrentState();
//         }
//     } catch (const std::invalid_argument& e) {
//         RCLCPP_ERROR(this->get_logger(), "Invalid TaskID received: '%s'. Error: %s", msg->data.c_str(), e.what());
//     } catch (const std::out_of_range& e) {
//         RCLCPP_ERROR(this->get_logger(), "TaskID out of range: '%s'. Error: %s", msg->data.c_str(), e.what());
//     }
// }


//    void observerInfoCallback(const observer_msgs::msg::ObserverInfo::SharedPtr msg) {
//     double smoothed_distance = smoothDistance(msg->distance);

//     // Publish smoothed distance for debugging
//     std_msgs::msg::Float64 smoothed_distance_msg;
//     smoothed_distance_msg.data = smoothed_distance;
//     smoothed_distance_publisher_->publish(smoothed_distance_msg);

//     // Handle infinity as "no object ahead"
//     if (std::isinf(smoothed_distance)) {
//         RCLCPP_INFO(this->get_logger(), "No object ahead, proceeding to accelerate or cruise.");
//         if (current_state_ == State::Idle) {
//             current_state_ = State::Accelerating;
//             target_velocity_ = 5.0;  // Target cruising speed in km/h
//             target_distance_ = 20.0; // Distance over which to reach target speed (in meters)
//             RCLCPP_INFO(this->get_logger(), "Transitioning to Accelerating state. Target velocity: 5 km/h, Target distance: 20 meters.");
//             sendUDPMessage();
//             publishTargetVelocityAndDistance();
//             publishCurrentState();
//         } else if (current_state_ == State::Cruising) {
//             target_velocity_ = 5.0;  // Maintain cruising speed
//             RCLCPP_INFO(this->get_logger(), "Already cruising at target velocity.");
//             sendUDPMessage();
//             publishTargetVelocityAndDistance();
//             publishCurrentState();
//         }
//     } else {
//         // Normal handling of smoothed distance
//         target_distance_ = smoothed_distance;

//         if (smoothed_distance < detection_threshold_ &&
//             (msg->object_class == "vehicle" || msg->object_class == "pedestrian")) {
//             rclcpp::Time now = this->now();
//             if ((now - last_detection_time_).seconds() >= 1.0) {
//                 if (current_state_ == State::Cruising || current_state_ == State::Accelerating) {
//                     current_state_ = State::Decelerating;
//                     target_velocity_ = 0.0;
//                     target_distance_ = std::max(smoothed_distance - 10.0, 0.0);  // Ensure positive distance
//                     RCLCPP_INFO(this->get_logger(), "Stable obstacle detected: %s. Decelerating to stop.",
//                                 msg->object_class.c_str());
//                     sendUDPMessage();  // Send the stop command
//                     publishTargetVelocityAndDistance();
//                     publishCurrentState();
//                 }
//             }
//         } else {
//             // Reset detection time if the object is no longer detected or out of range
//             last_detection_time_ = this->now();
//         }
//     }

//     // Always publish the target distance and state after processing the observer info
//     publishTargetVelocityAndDistance();
//     publishCurrentState();
// }


//     void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
//         // Logic to handle velocity updates
//         if (current_state_ == State::Accelerating && msg->linear.x >= 5.0) {
//             current_state_ = State::Cruising;
//             RCLCPP_INFO(this->get_logger(), "Reached cruising speed.");
//             sendUDPMessage();
//             publishCurrentState();
//         }
//     }
// };

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<ActorNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }



// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/float64.hpp"
// #include "geometry_msgs/msg/twist.hpp"
// #include "observer_msgs/msg/observer_info.hpp"
// #include "std_msgs/msg/string.hpp"
// #include <sys/socket.h>
// #include <arpa/inet.h>
// #include <unistd.h>
// #include <nlohmann/json.hpp>
// #include <cmath>
// #include <deque>
// #include <limits>
// #include <chrono>

// using json = nlohmann::json;

// class ActorNode : public rclcpp::Node {
// public:
//     ActorNode()
//         : Node("actor_node"), udp_socket_(-1), current_velocity_(0.0), target_velocity_(0.0), target_distance_(std::numeric_limits<double>::infinity()), state_(State::IDLE), last_state_change_time_(this->now()) {

//         // Subscription to ObserverInfo
//         observer_info_subscription_ = this->create_subscription<observer_msgs::msg::ObserverInfo>(
//             "observer_info", 10,
//             std::bind(&ActorNode::observerInfoCallback, this, std::placeholders::_1));

//         // Subscription to the vehicle velocity
//         velocity_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
//             "vehicle_velocity", 10,
//             std::bind(&ActorNode::velocityCallback, this, std::placeholders::_1));

//         // Subscription to the order info
//         order_info_subscription_ = this->create_subscription<std_msgs::msg::String>(
//             "order_info", 10,
//             std::bind(&ActorNode::orderInfoCallback, this, std::placeholders::_1));

//         // Publishers
//         target_distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>("target_distance", 10);
//         target_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>("target_velocity", 10);

//         // Initialize UDP socket
//         udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
//         if (udp_socket_ < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
//             rclcpp::shutdown();
//         }

//         // Setup the UDP server address structure
//         server_address_.sin_family = AF_INET;
//         server_address_.sin_port = htons(50131);  // Port number as provided
//         server_address_.sin_addr.s_addr = inet_addr("172.23.60.118");  // IP address as provided

//         RCLCPP_INFO(this->get_logger(), "Actor node initialized and UDP communication setup complete.");
//     }

//     ~ActorNode() {
//         if (udp_socket_ >= 0) {
//             close(udp_socket_);
//         }
//     }

// private:
//     enum class State {
//         IDLE,
//         ACCELERATING,
//         CRUISING,
//         DECELERATING,
//         STOPPED
//     };

//     State state_;
//     rclcpp::Time last_state_change_time_;  // Time when the state last changed
//     double current_velocity_; // Current velocity of the vehicle in km/h
//     double target_velocity_;  // Target velocity to send to the vehicle (0 or 5 km/h)
//     double target_distance_;  // Target distance to obstacle
//     int task_id_;             // Task ID from the order information

//     void observerInfoCallback(const observer_msgs::msg::ObserverInfo::SharedPtr msg) {
//         double distance_to_obstacle = msg->distance;

//         // Ensure state remains for at least one second before transitioning
//         if ((this->now() - last_state_change_time_).seconds() < 1.0) {
//             return;  // Do not change state until 1 second has passed
//         }

//         // State machine logic
//         switch (state_) {
//             case State::IDLE:
//                 if (task_id_ == 1 && distance_to_obstacle == std::numeric_limits<double>::infinity()) {
//                     state_ = State::ACCELERATING;
//                     target_velocity_ = 5.0;
//                     last_state_change_time_ = this->now();
//                     RCLCPP_INFO(this->get_logger(), "Switching to ACCELERATING state.");
//                 }
//                 break;

//             case State::ACCELERATING:
//                 if (distance_to_obstacle <= 30.0) {
//                     state_ = State::DECELERATING;
//                     target_velocity_ = 0.0;
//                     target_distance_ = std::max(0.0, distance_to_obstacle);
//                     last_state_change_time_ = this->now();
//                     RCLCPP_INFO(this->get_logger(), "Switching to DECELERATING state.");
//                 } else {
//                     state_ = State::CRUISING;
//                     last_state_change_time_ = this->now();
//                     RCLCPP_INFO(this->get_logger(), "Switching to CRUISING state.");
//                 }
//                 break;

//             case State::CRUISING:
//                 if (distance_to_obstacle <= 30.0) {
//                     state_ = State::DECELERATING;
//                     target_velocity_ = 0.0;
//                     target_distance_ = std::max(0.0, distance_to_obstacle);
//                     last_state_change_time_ = this->now();
//                     RCLCPP_INFO(this->get_logger(), "Switching to DECELERATING state.");
//                 }
//                 break;

//             case State::DECELERATING:
//                 if (distance_to_obstacle <= 10.0) {
//                     state_ = State::STOPPED;
//                     target_velocity_ = 0.0;
//                     target_distance_ = 0.0;
//                     last_state_change_time_ = this->now();
//                     RCLCPP_INFO(this->get_logger(), "Switching to STOPPED state.");
//                 } else if (distance_to_obstacle > 30.0) {
//                     state_ = State::CRUISING;
//                     target_velocity_ = 5.0;
//                     last_state_change_time_ = this->now();
//                     RCLCPP_INFO(this->get_logger(), "Switching back to CRUISING state.");
//                 }
//                 break;

//             case State::STOPPED:
//                 if (msg->signal_detected && msg->signal_setting == "white" && distance_to_obstacle > 10.0) {
//                     state_ = State::ACCELERATING;
//                     target_velocity_ = 5.0;
//                     last_state_change_time_ = this->now();
//                     RCLCPP_INFO(this->get_logger(), "Signal is white, switching to ACCELERATING state.");
//                 }
//                 break;
//         }

//         // Publish the target velocity and distance
//         publishTargetVelocity();
//         publishTargetDistance();

//         // Send UDP message with the target velocity and distance
//         sendUDPMessage();
//     }

//     void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
//         current_velocity_ = msg->linear.x * 3.6;  // Convert m/s to km/h
//     }

//     void orderInfoCallback(const std_msgs::msg::String::SharedPtr msg) {
//         try {
//             auto jsonDataParsed = json::parse(msg->data);
//             task_id_ = jsonDataParsed["TaskID"];

//             RCLCPP_INFO(this->get_logger(), "Received order info: TaskID = %d", task_id_);
//         } catch (json::exception& e) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", e.what());
//         }
//     }

//     void publishTargetVelocity() {
//         auto msg = std_msgs::msg::Float64();
//         msg.data = target_velocity_;
//         target_velocity_publisher_->publish(msg);
//     }

//     void publishTargetDistance() {
//         auto msg = std_msgs::msg::Float64();
//         msg.data = target_distance_;
//         target_distance_publisher_->publish(msg);
//     }

//     uint8_t encodeVelocity(double velocity_kmh) {
//         return static_cast<uint8_t>(std::min(255.0, std::max(0.0, velocity_kmh * 255.0 / 40.0)));
//     }

//     uint8_t encodeDistance(double distance_m) {
//         return static_cast<uint8_t>(std::min(255.0, std::max(0.0, distance_m)));
//     }

//     void sendUDPMessage() {
//         uint8_t encoded_velocity = encodeVelocity(target_velocity_);
//         uint8_t encoded_distance = encodeDistance(target_distance_);

//         uint8_t data[2] = {encoded_velocity, encoded_distance};
        
//         ssize_t sent_bytes = sendto(udp_socket_, data, sizeof(data), 0,
//                                     (struct sockaddr *)&server_address_, sizeof(server_address_));
//         if (sent_bytes < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to send data via UDP");
//         } else {
//             RCLCPP_INFO(this->get_logger(), "Sent UDP message: Velocity: %d, Distance: %d", data[0], data[1]);
//         }
//     }

//     rclcpp::Subscription<observer_msgs::msg::ObserverInfo>::SharedPtr observer_info_subscription_;
//     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr order_info_subscription_;
//     rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_distance_publisher_;
//     rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_velocity_publisher_;

//     int udp_socket_;
//     struct sockaddr_in server_address_;
// };

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ActorNode>());
//     rclcpp::shutdown();
//     return 0;
// }



// #include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/float64.hpp"
// #include "std_msgs/msg/string.hpp"
// #include "geometry_msgs/msg/twist.hpp"  // Include for Twist message type
// #include <sys/socket.h>
// #include <arpa/inet.h>
// #include <unistd.h>
// #include <nlohmann/json.hpp> // Include nlohmann/json for JSON parsing
// #include <cmath>     // For calculations
// #include <string>
// #include <deque>     // For maintaining a history of distances
// #include <chrono>    // For handling timeouts

// using json = nlohmann::json;
// using namespace std::chrono_literals;

// class ActorNode : public rclcpp::Node {
// public:
//     ActorNode()
//         : Node("actor_node"), udp_socket_(-1), task_id_(0), target_velocity_(0.0), distance_x_(0.0),
//           last_data_time_(this->now()), zero_velocity_start_time_(this->now()), holding_zero_velocity_(false) {

//         // Subscriptions
//         distance_subscription_ = this->create_subscription<std_msgs::msg::Float64>(
//             "distance_to_collision", 10,
//             std::bind(&ActorNode::distanceCallback, this, std::placeholders::_1));

//         velocity_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
//             "vehicle_velocity", 10,
//             std::bind(&ActorNode::velocityCallback, this, std::placeholders::_1));

//         order_info_subscription_ = this->create_subscription<std_msgs::msg::String>(
//             "order_info", 10,
//             std::bind(&ActorNode::orderInfoCallback, this, std::placeholders::_1));

//         // Publishers
//         target_distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>("target_distance", 10);
//         target_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>("target_velocity", 10);

//         // Initialize UDP socket
//         udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
//         if (udp_socket_ < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
//             rclcpp::shutdown();
//         }

//         // Setup the UDP server address structure
//         server_address_.sin_family = AF_INET;
//         server_address_.sin_port = htons(50131);  // Port number as provided
//         server_address_.sin_addr.s_addr = inet_addr("172.23.60.118");  // IP address as provided

//         // Set a timer to periodically check for data timeout and send zero values if necessary
//         timer_ = this->create_wall_timer(
//             100ms, std::bind(&ActorNode::checkAndSendZeroIfNeeded, this)
//         );

//         RCLCPP_INFO(this->get_logger(), "UDP communication setup complete. Ready to send data.");
//     }

//     ~ActorNode() {
//         if (udp_socket_ >= 0) {
//             close(udp_socket_);
//         }
//     }

// private:
//     static constexpr size_t HISTORY_SIZE = 5;  // Number of past distances to keep
//     std::deque<double> distance_history_;  // Circular buffer for distance history
//     rclcpp::Time last_data_time_;  // Time when the last data was received
//     rclcpp::Time zero_velocity_start_time_;  // Time when the velocity was set to zero
//     rclcpp::TimerBase::SharedPtr timer_;  // Timer to check for data timeout

//     rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_distance_publisher_;
//     rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_velocity_publisher_;

//     bool holding_zero_velocity_;  // Flag to indicate whether we are holding zero velocity for confirmation

//     void distanceCallback(const std_msgs::msg::Float64::SharedPtr msg) {
//         last_data_time_ = this->now();  // Update the last data received time
//         double distance_to_collision = msg->data;

//         // Add new distance to history and remove oldest if the buffer is full
//         if (distance_history_.size() >= HISTORY_SIZE) {
//             distance_history_.pop_front();
//         }
//         distance_history_.push_back(distance_to_collision);

//         // Smooth the distance
//         distance_x_ = smoothDistance();

//         if (holding_zero_velocity_) {
//             // Check if we have held zero velocity for one second
//             if ((this->now() - zero_velocity_start_time_).seconds() >= 1.0) {
//                 RCLCPP_INFO(this->get_logger(), "Zero velocity confirmed after 1 second.");
//                 publishTargetVelocity();
//                 publishTargetDistance();
//                 sendUDPMessage(true);
//                 holding_zero_velocity_ = false;  // Reset the holding flag
//             }
//             return;  // Do nothing else until zero velocity is confirmed
//         }

//         // Logic for obstacle avoidance and acceleration
//         if (distance_x_ > 30.0) {
//             // No obstacle detected within 30 meters
//             target_velocity_ = 5.0;  // Accelerate to 5 km/h
//             publishTargetVelocity();  // Publish target velocity
//             sendUDPMessage(false);  // Send only velocity, omit distance
//         } else {
//             // Obstacle detected within 30 meters, start decelerating
//             if (task_id_ == 1) {
//                 // Stop right at the obstacle
//                 if (distance_x_ <= 0.0) {
//                     startHoldingZeroVelocity();  // Start holding zero velocity for confirmation
//                 } else {
//                     target_velocity_ = std::min(5.0, distance_x_ / 5.0);
//                     publishTargetVelocity();  // Publish target velocity
//                     publishTargetDistance();  // Publish target distance
//                     sendUDPMessage(true);  // Send both velocity and distance
//                 }
//             } else if (task_id_ == 0) {
//                 // Stop 10 meters in front of the obstacle
//                 if (distance_x_ <= 10.0) {
//                     startHoldingZeroVelocity();  // Start holding zero velocity for confirmation
//                 } else {
//                     target_velocity_ = std::min(5.0, 5.0 * (distance_x_ - 10.0) / 20.0);  // Decelerate based on distance
//                     publishTargetVelocity();  // Publish target velocity
//                     sendUDPMessage(false);  // Send only velocity, omit distance
//                 }
//             } else {
//                 // Handle other task IDs if necessary
//                 target_velocity_ = 0.0;
//                 publishTargetVelocity();  // Publish target velocity
//                 publishTargetDistance();  // Publish target distance
//                 sendUDPMessage(true);  // Send both velocity and distance
//             }
//         }
//     }

//     void startHoldingZeroVelocity() {
//         target_velocity_ = 0.0;
//         holding_zero_velocity_ = true;
//         zero_velocity_start_time_ = this->now();
//         RCLCPP_INFO(this->get_logger(), "Holding zero velocity for confirmation...");
//     }

//     void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
//         last_data_time_ = this->now();  // Update the last data received time
//         current_velocity_ = msg->linear.x;  // Velocity is already in m/s
//     }

//     void orderInfoCallback(const std_msgs::msg::String::SharedPtr msg) {
//         try {
//             last_data_time_ = this->now();  // Update the last data received time
//             auto jsonDataParsed = json::parse(msg->data);

//             int currPos = jsonDataParsed["CurrentPosition"];
//             int targetPos = jsonDataParsed["TargetPosition"];
//             task_id_ = jsonDataParsed["TaskID"];

//             RCLCPP_INFO(this->get_logger(), "Current Position: %d, Target Position: %d, TaskID: %d",
//                         currPos, targetPos, task_id_);
//         } catch (json::exception& e) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to parse JSON: %s", e.what());
//         }
//     }

//     double smoothDistance() {
//         if (distance_history_.empty()) return distance_x_;  // No data to smooth

//         double sum = 0.0;
//         for (double distance : distance_history_) {
//             sum += distance;
//         }

//         double avg_distance = sum / distance_history_.size();

//         // Verify the distance consistency based on vehicle movement
//         if (!distance_history_.empty() && current_velocity_ > 0.1) {  // Consider only forward movement
//             double expected_decrease = current_velocity_ * 0.1;  // Example of expected decrease per time step
//             if (distance_history_.back() - avg_distance < -expected_decrease) {
//                 // If the distance increased significantly despite forward movement, ignore the latest reading
//                 RCLCPP_WARN(this->get_logger(), "Inconsistent distance reading detected. Keeping the previous value.");
//                 return distance_history_.front();
//             }
//         }

//         return avg_distance;
//     }

//     void checkAndSendZeroIfNeeded() {
//         auto now = this->now();
//         if ((now - last_data_time_).seconds() > 0.5) {  // Timeout threshold of 0.5 seconds
//             RCLCPP_WARN(this->get_logger(), "No data received for 0.5 seconds, sending zeros.");
//             target_velocity_ = 0.0;
//             distance_x_ = 0.0;
//             publishTargetVelocity();  // Publish zero target velocity
//             publishTargetDistance();  // Publish zero target distance
//             sendUDPMessage(true);  // Send zeros for both velocity and distance
//         }
//     }

//     void publishTargetVelocity() {
//         auto msg = std_msgs::msg::Float64();
//         msg.data = target_velocity_;  // Publish in km/h
//         target_velocity_publisher_->publish(msg);
//     }

//     void publishTargetDistance() {
//         auto msg = std_msgs::msg::Float64();
//         msg.data = distance_x_;  // Publish distance in meters
//         target_distance_publisher_->publish(msg);
//     }

//     uint8_t encodeVelocity(double velocity_kmh) {
//         // Convert velocity from km/h to uint8 (0 = 0km/h, 255 = 40km/h)
//         return static_cast<uint8_t>(std::min(255.0, std::max(0.0, velocity_kmh * 255.0 / 40.0)));
//     }

//     uint8_t encodeDistance(double distance_m) {
//         // Convert distance in meters to uint8 (capped at 255 meters)
//         return static_cast<uint8_t>(std::min(255.0, std::max(0.0, distance_m)));
//     }

//     void sendUDPMessage(bool send_distance) {
//         uint8_t encoded_velocity = encodeVelocity(target_velocity_);
//         uint8_t encoded_distance = send_distance ? encodeDistance(distance_x_) : 0;

//         // Prepare the data packet
//         uint8_t data[2] = {encoded_velocity, encoded_distance};
        
//         // Send the data via UDP
//         ssize_t sent_bytes = sendto(udp_socket_, data, sizeof(data), 0,
//                                     (struct sockaddr *)&server_address_, sizeof(server_address_));
//         if (sent_bytes < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to send data via UDP");
//         } else {
//             RCLCPP_INFO(this->get_logger(), "Sent UDP message: Velocity: %d, Distance: %d", data[0], data[1]);
//         }
//     }

//     rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr distance_subscription_;
//     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr order_info_subscription_;

//     int udp_socket_;
//     struct sockaddr_in server_address_;

//     int task_id_;
//     double current_velocity_;
//     double target_velocity_;
//     double distance_x_;
// };

// int main(int argc, char *argv[]) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<ActorNode>());
//     rclcpp::shutdown();
//     return 0;
// }
