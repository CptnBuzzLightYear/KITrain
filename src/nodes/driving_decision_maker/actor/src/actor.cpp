
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
                  smoothing_window_size_(10), detection_threshold_(50.0), 
                  last_velocity_(0.0), v_max_(15.0), s_target_(40.0), 
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
        target_distance_ = 0.0; // Default value, to be updated based on state
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
        server_address_.sin_port = htons(2869);  // Port number as provided
        server_address_.sin_addr.s_addr = inet_addr("10.8.3.3");  //receiver port (Kinematics model)

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
        WAITING_FOR_WHITE_SIGNAL,
        ACCELERATING,
        CLOSINGGAP,
        DECELERATING,
        STOPPED,
        STOP,
        ATTACHING,
        EMERGENCY_STOP
    };
    // Convert string to State enum
    State stringToState(const std::string &state_str) {
        if (state_str == "idle") {
            return State::IDLE;
        } else if (state_str == "waitingforwhite") {
            return State::WAITING_FOR_WHITE_SIGNAL;
        } else if (state_str == "accelerating") {
            return State::ACCELERATING;
        } else if (state_str == "closing_gap") {
            return State::CLOSINGGAP;
        } else if (state_str == "decelerating") {
            return State::DECELERATING;
        } else if (state_str == "stopped") {
            return State::STOPPED;
        } else if (state_str == "stop") {
            return State::STOP;
        } else if (state_str == "attaching") {
            return State::ATTACHING;
        } else if (state_str == "emergency_stop") {
            return State::EMERGENCY_STOP;
        } else {
            return State::UNINITIALIZED;  // Default case for unknown state
        }
    }

    // Convert State enum to string for logging
    std::string stateToString(State state) {
        switch (state) {
            case State::IDLE: return "Idle";
            case State::WAITING_FOR_WHITE_SIGNAL: return "Waitingforwhite";
            case State::ACCELERATING: return "Accelerating";
            case State::CLOSINGGAP: return "Closing Gap";
            case State::DECELERATING: return "Decelerating";
            case State::STOPPED: return "Stopped";
            case State::STOP: return "Stop";
            case State::ATTACHING: return "Attaching";
            case State::EMERGENCY_STOP: return "Emergency Stop";
            default: return "Uninitialized";
        }
    }

    State current_state_;
    int current_task_id_;  
    double target_velocity_;
    double target_distance_;
    rclcpp::Time last_detection_time_;
    double last_velocity_;
    double v_max_;
    double s_target_;
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
    rclcpp::Time path_clear_start_time_;
    bool is_path_clear_timer_active_;

      // Signal detection and setting from the observer_info message
    bool signal_detected_ = true;
    std::string signal_setting_ = "red";

    uint8_t encodeVelocity(double velocity) {
        return static_cast<uint8_t>(std::min(std::max(velocity / 40.0 * 255.0, 0.0), 255.0));
    }

    uint8_t encodeDistance(double distance) {
        double max_distance = 255.0; // Example: 255 meters max
        return static_cast<uint8_t>(std::min(std::max(distance, 0.0), max_distance));
    }

    void sendUDPMessage() {
     //   RCLCPP_INFO(this->get_logger(), "Sending UDP - Target Velocity: %f, Target Distance: %f", target_velocity_, target_distance_);
        
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
          //  RCLCPP_INFO(this->get_logger(), "Sent UDP message: Velocity: %d, Distance: %d", data[0], data[1]);
        }
    }

   double smoothDistance(double new_distance) {
    // Check if the new distance is infinity
    if (std::isinf(new_distance)) {
        // Verify if it's infinite for 2 seconds
        auto start_time = std::chrono::steady_clock::now();
        bool is_infinite = true;

        while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2)) {
            if (!std::isinf(new_distance)) {
                is_infinite = false;
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Sleep for 100ms between checks
        }

        // If the value remained infinite for 2 seconds, return infinity
        if (is_infinite) {
            return std::numeric_limits<double>::infinity();
        }
    }

    // If the new distance is neither infinite nor NaN, proceed with smoothing
    if (!std::isnan(new_distance) && !std::isinf(new_distance)) {
        if (distance_history_.size() >= smoothing_window_size_) {
            distance_history_.pop_front();
        }
        distance_history_.push_back(new_distance);
    }

    // If there's no history yet, return 0.0
    if (distance_history_.empty()) {
        return 0.0;
    }

    // Compute the smoothed value by averaging the history
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
                state_msg.data = "IDLE";
                break;
            case State::WAITING_FOR_WHITE_SIGNAL:
                state_msg.data = "waiting for white signal ...";
                break;
            case State::ACCELERATING:
                state_msg.data = "ACCLEREATING";
                break;
            case State::CLOSINGGAP:
                state_msg.data = "CLOSING GAP";
                break;
            case State::DECELERATING:
                state_msg.data = "DECELERATING";
                break;
            case State::STOPPED:
                state_msg.data = "STOPPED";
                break;
            case State::STOP:
                state_msg.data = "STOPPING";
                break;
            case State::ATTACHING:
                state_msg.data = "ATTACHING";
                break;
            case State::EMERGENCY_STOP:
                state_msg.data = "EMERGENCY STOP";
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

         // Check current state and handle transitions based on signal conditions
        // if (current_state_ == State::IDLE) {
        //     if (signal_detected_ && signal_setting_ != "white") {
        //         // If signal is detected and not white, stay in idle
        //         RCLCPP_WARN(this->get_logger(), "Cannot transition from idle, waiting for signal to turn white.");
        //         return;  // Exit the callback without changing the state
        //     }
        // }
         // If conditions are met, proceed with the state transition
      //  current_state_ = stringToState(msg->data);  // Change the state to the new one from the order message
      //  RCLCPP_INFO(this->get_logger(), "Transitioning to state: %s", current_state_.c_str());
    

        // Clear the "Task Completed" message if a new task is set
        if (task_id > 0) {
            clearTaskCompletedMessage();
            RCLCPP_INFO(this->get_logger(), "New task received, clearing 'Task Completed' state.");
        }

        // Emergency Stop
        if (task_id == 0) {
            current_state_ = State::EMERGENCY_STOP;
            target_velocity_ = 0.0;
            target_distance_ = 5.0;
            RCLCPP_INFO(this->get_logger(), "Switching to EMERGENCY_STOP state.");
        } else {
            // Regular Task Handling
           if (current_state_ == State::IDLE && (!signal_detected_ || signal_setting_ == "white")) {
                if (smoothed_distance_ > 40.0) {
                    current_state_ = State::ACCELERATING;
                    target_velocity_ = v_max_;
                    target_distance_ = s_target_;
                    RCLCPP_INFO(this->get_logger(), "Track free - switching to ACCELERATING state.");
                } else if (smoothed_distance_ > 10.0 && smoothed_distance_ < 40.0) {
                    current_state_ = State::CLOSINGGAP;
                    target_velocity_ = v_max_ / 2.0;
                    target_distance_ = 10.0;
                    RCLCPP_INFO(this->get_logger(), "Switching to CLOSINGGAP state.");
                } else if (task_id == 1 && smoothed_distance_ > 3.5 && smoothed_distance_ <= 10.0) {
                    current_state_ = State::ATTACHING;
                    target_velocity_ = 3.0;
                    target_distance_ = 5.0;
                    RCLCPP_INFO(this->get_logger(), "Switching to ATTACHING state.");
                }
                else {
                    current_state_ = State::STOPPED;
                    target_velocity_ = 0.0;
                    target_distance_ = 0.0;

                }
            }
        }

        publishTargetVelocityAndDistance();
        publishCurrentState();

    } catch (const std::invalid_argument& e) {
       // RCLCPP_ERROR(this->get_logger(), "Invalid TaskID received: '%s'. Error: %s", msg->data.c_str(), e.what());
    } catch (const std::out_of_range& e) {
       // RCLCPP_ERROR(this->get_logger(), "TaskID out of range: '%s'. Error: %s", msg->data.c_str(), e.what());
    }
}

void observerInfoCallback(const observer_msgs::msg::ObserverInfo::SharedPtr msg) {
    double smoothed_distance = smoothDistance(msg->distance);
    smoothed_distance_ = smoothed_distance;

    std_msgs::msg::Float64 smoothed_distance_msg;
    smoothed_distance_msg.data = smoothed_distance;
    smoothed_distance_publisher_->publish(smoothed_distance_msg);

      signal_detected_ = msg->signal_detected;
      signal_setting_ = msg->signal_setting;    

    // Handle transitions based on real-time distance information

    if(signal_detected_ && signal_setting_ == "red"){
        current_state_ = State::WAITING_FOR_WHITE_SIGNAL;
        target_velocity_ = 0.0;
        target_distance_ = 5.0;
          RCLCPP_INFO(this->get_logger(), "RED signal, waiting for WHITE signal setting.");
    }
 if (current_state_ == State::WAITING_FOR_WHITE_SIGNAL && (!signal_detected_ || signal_setting_ == "white")) {
        current_state_ = State::DECELERATING;
        target_velocity_ = 0.0;
        target_distance_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Obstacle detected within 40 meters. Switching to DECELERATING state.");
    }


    // if (current_state_ == State::WAITING_FOR_WHITE_SIGNAL && signal_setting_ == "white"){
    //      if (smoothed_distance_ > 40.0) {
    //                 current_state_ = State::ACCELERATING;
    //                 target_velocity_ = v_max_;
    //                 target_distance_ = s_target_;
    //                 RCLCPP_INFO(this->get_logger(), "Track free - switching to ACCELERATING state.");
    //             } else if (smoothed_distance_ > 10.0 && smoothed_distance_ < 40.0) {
    //                 current_state_ = State::CLOSINGGAP;
    //                 target_velocity_ = v_max_ / 2.0;
    //                 target_distance_ = 10.0;
    //                 RCLCPP_INFO(this->get_logger(), "Switching to CLOSINGGAP state.");
    //             } else if (current_task_id_ == 1 && smoothed_distance_ > 3.5 && smoothed_distance_ <= 10.0) {
    //                 current_state_ = State::ATTACHING;
    //                 target_velocity_ = 3.0;
    //                 target_distance_ = 5.0;
    //                 RCLCPP_INFO(this->get_logger(), "Switching to ATTACHING state.");
    //             }
    //             else {
    //                 current_state_ = State::STOPPED;
    //                 target_velocity_ = 0.0;
    //                 target_distance_ = 0.0;

    //             }
    // }

    // Transition from ACCELERATING to DECELERATING if an obstacle is detected within 40 meters
    if (current_state_ == State::ACCELERATING && smoothed_distance_ < 40.0) {
        current_state_ = State::DECELERATING;
        target_velocity_ = 0.0;
        target_distance_ = (smoothed_distance_ > 10.0) ? smoothed_distance_ - 10.0 : 0.0;
        RCLCPP_INFO(this->get_logger(), "Obstacle detected within 40 meters. Switching to DECELERATING state.");
    }

     // Transition from ACCELERATING to DECELERATING if an obstacle is detected within 40 meters
    if (current_state_ == State::CLOSINGGAP && smoothed_distance_ > 40.0) {
        current_state_ = State::ACCELERATING;
        target_velocity_ = v_max_;
        target_distance_ = s_target_;
        RCLCPP_INFO(this->get_logger(), "Obstacle detected within 40 meters. Switching to DECELERATING state.");
    }

       // Transition from DECELERATION to STOP during ATTACHING
    if (current_state_ == State::DECELERATING && current_task_id_ == 1 && smoothed_distance_ < 2.5) {
        current_state_ = State::IDLE;
        target_velocity_ = 0.0;
        target_distance_ = 0.5;
        RCLCPP_INFO(this->get_logger(), "Attaching to the target vehicle successfull.");
    }
    

    // Transition from DECELERATING to ACCELERATING if the path clears
    if (current_state_ == State::DECELERATING && smoothed_distance_ > 40.0) {
      //  if (!is_path_clear_timer_active_) {
            // Start the 2-second verification timer
       //     path_clear_start_time_ = this->now();
       //     is_path_clear_timer_active_ = true;
       //     RCLCPP_INFO(this->get_logger(), "Path appears clear, starting 2-second verification.");
       // } else {
            // Check if 2 seconds have passed with the path remaining clear
       //     if ((this->now() - path_clear_start_time_).seconds() > 2.0) {
                // Path has been clear for 2 seconds, safe to transition to ACCELERATING
                current_state_ = State::ACCELERATING;
                target_velocity_ = v_max_;
                target_distance_ = s_target_;
         //       is_path_clear_timer_active_ = false;  // Reset the timer flag
                RCLCPP_INFO(this->get_logger(), "Path confirmed clear for 2 seconds. Obstacle cleared, switching back to ACCELERATING.");
         //   }
        }
  //  } else if (smoothed_distance_ < 39.0){
        // Reset the timer if the distance drops below 40 meters during the verification period
  //      is_path_clear_timer_active_ = false;
  //  }

    //Transition from ATTACHING to ACCELERATING if the path has been clear for 2 seconds
    if (current_state_ == State::ATTACHING && smoothed_distance_ > 40.0) {
        if (!is_path_clear_timer_active_) {
            // Start the 2-second verification timer
            path_clear_start_time_ = this->now();
            is_path_clear_timer_active_ = true;
            RCLCPP_INFO(this->get_logger(), "Path appears clear, starting 2-second verification.");
        } else {
            // Check if 2 seconds have passed with the path remaining clear
            if ((this->now() - path_clear_start_time_).seconds() > 2.0) {
                // Path has been clear for 2 seconds, safe to transition to ACCELERATING
                current_state_ = State::ACCELERATING;
                target_velocity_ = v_max_;
                target_distance_ = s_target_;
                is_path_clear_timer_active_ = false;  // Reset the timer flag
                RCLCPP_INFO(this->get_logger(), "Path confirmed clear for 2 seconds. Switching to ACCELERATING.");
            }
        }
    } else if (smoothed_distance_ < 39.0){
        // Reset the timer if the distance drops below 40 meters during the verification period
        is_path_clear_timer_active_ = false;
    }

    // Handle CLOSINGGAP state: target is to stop 10 meters in front of the obstacle
    if (current_state_ == State::CLOSINGGAP || current_state_ == State::DECELERATING /*|| current_state_ == State::ATTACHING*/) {
       // target_distance_ = (smoothed_distance_ > 10.0) ? smoothed_distance_ - 10.0 : 1.0;

        // If Task ID = 1 and within 10 meters, transition to ATTACHING state
        if (smoothed_distance_ > 5.0 && smoothed_distance <= 10.0 && current_task_id_ == 1) {
            current_state_ = State::ATTACHING;
            target_velocity_ = 3.0;  // Slow speed for fine approach
            target_distance_ = (smoothed_distance_ > 3.0) ? smoothed_distance_ - 3.0 : 1.0;
            RCLCPP_INFO(this->get_logger(), "Task ID = 1, transitioning to ATTACHING state.");
        } else if (smoothed_distance_ <= 10.0 && current_task_id_ != 1) {
            // If not Task ID = 1, transition to DECELERATING to stop 10 meters before the obstacle
            current_state_ = State::STOPPED;
            target_velocity_ = 0.0;
            target_distance_ = 2.0;
            RCLCPP_INFO(this->get_logger(), "Approaching obstacle, switching to DECELERATING state to stop 10 meters before obstacle.");
        }
    }

    //Transition to Stopp and IDLE after ATTACHING
        // Transition from DECELERATION to STOP during ATTACHING
    // if (current_state_ == State::ATTACHING && smoothed_distance_ < 5.0) {
    //     current_state_ = State::STOP;
    //     target_velocity_ = 0.0;
    //     target_distance_ = 1.5;
    //     RCLCPP_INFO(this->get_logger(), "Attaching to the target vehicle successfull.");
    // }

     // Handle transition to CLOSING GAP or ATTACHING after approaching
    // // if (current_state_ == State::IDLE && current_task_id_ == 1) {
    // //     // within 10 meters, transition to ATTACHING state
    // //     if (smoothed_distance_ > 3.0 && smoothed_distance_ && smoothed_distance_ <= 10.0) {
    // //         current_state_ = State::ATTACHING;
    // //         target_velocity_ = 3.0;  // Slow speed for fine approach
    // //         target_distance_ = 5.0 ;
    // //         RCLCPP_INFO(this->get_logger(), "Task ID = 1, transitioning to ATTACHING state.");
    // //     } else if (smoothed_distance_ > 10.0 && smoothed_distance_ <= 20.0 ) {
    // //         // If not Task ID = 1, transition to DECELERATING to stop 10 meters before the obstacle
    // //         current_state_ = State::CLOSINGGAP;
    // //         target_velocity_ = v_max_ / 2.0;
    // //         target_distance_ = 15.0;
    // //         RCLCPP_INFO(this->get_logger(), "Approaching obstacle, switching to DECELERATING state to stop 10 meters before obstacle.");
    // //      }
    // // }

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
            target_velocity_ = 0.0;
            target_distance_ = 0.0;
            has_published_idle_ = true;
            RCLCPP_INFO(this->get_logger(), "Vehicle velocity is 0. Publishing IDLE state.");
            publishCurrentState();
        }
        return;
    }

      // Handle Attaching mode
    if (current_state_ == State::ATTACHING){
        if (smoothed_distance_ > 5.0) {
       target_distance_ = smoothed_distance_ - 4.0; // Continuously update target distance
        }
        else if (smoothed_distance_ < 5.0){
            if (msg->linear.x > 0.2){
                current_state_ = State::STOP;
                target_distance_ = 2.7;
                target_velocity_ = 0.0;
            }
            else if(msg->linear.x == 0.0){
                target_distance_ = 2.0;
                target_velocity_ = 1.0;
            }
        }
        // if (msg->linear.x == 0.0) {  // Vehicle has come to a stop
        //     current_state_ = State::IDLE;
        //     target_velocity_ = 0.0;
        //     target_distance_ = 0.0;
        //     RCLCPP_INFO(this->get_logger(), "Vehicle stopped. Switching to STOPPED state.");
        // }
        publishTargetVelocityAndDistance();
        return;
    }

    // DECELERATING: Adjust target distance dynamically
    if (current_state_ == State::DECELERATING && smoothed_distance_ > 10) {
       target_distance_ = smoothed_distance_ - 10.0;  // Continuously update target distance
        if (msg->linear.x == 0.0) {  // Vehicle has come to a stop
            current_state_ = State::IDLE;
            target_velocity_ = 0.0;
            target_distance_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Vehicle stopped. Switching to STOPPED state.");
        }
        publishTargetVelocityAndDistance();
        return;
    }

    // ATTACHING: Stop the vehicle when within 2.5 meters
    if (current_state_ == State::ATTACHING && msg->linear.x > 0.5) {
        if (smoothed_distance_ <= 5.0) {
            current_state_ = State::STOP;
            target_velocity_ = 0.0;
            target_distance_ = smoothed_distance_ - 3.3;
            RCLCPP_INFO(this->get_logger(), "Reached target for ATTACHING. Switching to STOPPED.");
         } //else {
        //     target_distance_ = std::max(target_distance_ - distance_travelled, 0.0);
        // }
        publishTargetVelocityAndDistance();
        
    }

    //STOPPED to IDLE after ATTACHING (only when completely stopped)
    if ((current_state_ == State::DECELERATING || current_state_ == State::STOP) && msg->linear.x == 0.0 /*&& smoothed_distance_ < 3.0*/) {
        current_state_ = State::IDLE;
        target_velocity_ = 0.0;
        target_distance_ = 0.0;
        RCLCPP_INFO(this->get_logger(), "Vehicle has stopped after ATTACHING. Switching to IDLE state.");
        publishCurrentState();
       
    }
      //STOPPED to IDLE after ATTACHING (only when completely stopped)
    if (current_state_ == State::EMERGENCY_STOP && msg->linear.x == 0.0) {
        current_state_ = State::IDLE;
        RCLCPP_INFO(this->get_logger(), "Vehicle has stopped after ATTACHING. Switching to IDLE state.");
        publishCurrentState();
       
    }
    

    last_velocity_ = msg->linear.x;
    last_detection_time_ = this->now();

    publishTargetVelocityAndDistance();
    publishCurrentState();
}
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
// #include <std_msgs/msg/bool.hpp>
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
//     ActorNode() 
//         : Node("actor_node"), 
//           current_state_(State::UNINITIALIZED),
//           smoothing_window_size_(10), 
//           detection_threshold_(50.0), 
//           last_velocity_(0.0), 
//           v_max_(15.0), 
//           s_target_(40.0), 
//           has_published_idle_(false), 
//           signal_detected_(false), 
//           signal_setting_("white"),  // Default to white signal
//           is_path_clear_timer_active_(false) 
//     {
//         // Initialize TaskID to -1 (no task received)
//         current_task_id_ = -1;  

//         // Subscriptions
//         observer_info_subscription_ = this->create_subscription<observer_msgs::msg::ObserverInfo>(
//             "observer_info", 10, 
//             std::bind(&ActorNode::observerInfoCallback, this, std::placeholders::_1));
        
//         velocity_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
//             "vehicle_velocity", 10, 
//             std::bind(&ActorNode::velocityCallback, this, std::placeholders::_1));
        
//         order_info_subscription_ = this->create_subscription<std_msgs::msg::String>(
//             "order_info", 10, 
//             std::bind(&ActorNode::orderInfoCallback, this, std::placeholders::_1));

//         // Signal Subscriptions
//         signal_detected_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
//             "signal_detected", 10, 
//             std::bind(&ActorNode::signalDetectedCallback, this, std::placeholders::_1));
        
//         signal_setting_subscription_ = this->create_subscription<std_msgs::msg::String>(
//             "signal_setting", 10, 
//             std::bind(&ActorNode::signalSettingCallback, this, std::placeholders::_1));

//         // Publishers
//         target_velocity_publisher_ = this->create_publisher<std_msgs::msg::Float64>("target_velocity", 10);
//         target_distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>("target_distance", 10);
//         current_state_publisher_ = this->create_publisher<std_msgs::msg::String>("current_state", 10);
//         smoothed_distance_publisher_ = this->create_publisher<std_msgs::msg::Float64>("smoothed_distance", 10);
//         task_completed_publisher_ = this->create_publisher<std_msgs::msg::String>("task_completed", 10);

//         // Initialize UDP socket
//         udp_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
//         if (udp_socket_ < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to create UDP socket");
//             rclcpp::shutdown();
//         }

//         // Setup the UDP server address structure
//         memset(&server_address_, 0, sizeof(server_address_));
//         server_address_.sin_family = AF_INET;
//         server_address_.sin_port = htons(2869);  // Port number as provided
//         server_address_.sin_addr.s_addr = inet_addr("10.8.3.2");  // Receiver IP

//         // Timer to periodically send UDP messages
//         udp_timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(100),  // Send UDP message every 100 ms
//             std::bind(&ActorNode::sendUDPMessage, this)
//         );

//         // Timer for checking if IDLE state has lasted 2 seconds
//         idle_timer_ = this->create_wall_timer(
//             std::chrono::seconds(2),
//             std::bind(&ActorNode::idleTimerCallback, this)
//         );

//         RCLCPP_INFO(this->get_logger(), "ActorNode has been initialized.");
//     }

//     ~ActorNode() {
//         close(udp_socket_);
//     }

// private:
//     // Enumeration for different states
//     enum class State {
//         UNINITIALIZED,  // Initial state before receiving any velocity messages
//         IDLE,
//         ACCELERATING,
//         CLOSINGGAP,
//         DECELERATING,
//         ATTACHING,
//         EMERGENCY_STOP
//     };

//     // Current state of the node
//     State current_state_;
//     int current_task_id_;  
//     double target_velocity_;
//     double target_distance_;
//     rclcpp::Time last_detection_time_;
//     double last_velocity_;
//     double v_max_;
//     double s_target_;
//     bool has_published_idle_;

//     size_t smoothing_window_size_;
//     double detection_threshold_;
//     std::deque<double> distance_history_;

//     int udp_socket_;
//     struct sockaddr_in server_address_;

//     // Subscriptions
//     rclcpp::Subscription<observer_msgs::msg::ObserverInfo>::SharedPtr observer_info_subscription_;
//     rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr order_info_subscription_;
//     rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr signal_detected_subscription_;
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr signal_setting_subscription_;

//     // Publishers
//     rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_velocity_publisher_;
//     rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr target_distance_publisher_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr current_state_publisher_;
//     rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr smoothed_distance_publisher_;
//     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr task_completed_publisher_;

//     // Timers
//     rclcpp::TimerBase::SharedPtr udp_timer_;
//     rclcpp::TimerBase::SharedPtr idle_timer_;  // Timer for IDLE state
//     rclcpp::Time path_clear_start_time_;
//     bool is_path_clear_timer_active_;

//     // Signal Handling Variables
//     bool signal_detected_;          // Signal detection status
//     std::string signal_setting_;    // Signal setting ("red", "white", etc.)

//     // Helper function to encode velocity into a byte
//     uint8_t encodeVelocity(double velocity) {
//         return static_cast<uint8_t>(std::min(std::max(velocity / 40.0 * 255.0, 0.0), 255.0));
//     }

//     // Helper function to encode distance into a byte
//     uint8_t encodeDistance(double distance) {
//         double max_distance = 255.0; // Example: 255 meters max
//         return static_cast<uint8_t>(std::min(std::max(distance, 0.0), max_distance));
//     }

//     // Function to send UDP messages
//     void sendUDPMessage() {
//         uint8_t encoded_velocity = encodeVelocity(target_velocity_);
//         uint8_t encoded_distance = encodeDistance(target_distance_);

//         uint8_t data[2] = {encoded_velocity, encoded_distance};
//         ssize_t sent_bytes = sendto(udp_socket_, data, sizeof(data), 0,
//                                     (struct sockaddr *)&server_address_, sizeof(server_address_));
//         if (sent_bytes < 0) {
//             RCLCPP_ERROR(this->get_logger(), "Failed to send data via UDP.");
//         }
//     }

//     // Function to smooth distance measurements
//     double smoothDistance(double new_distance) {
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

//     // Function to publish the current state as a string
//     void publishCurrentState() {
//         std_msgs::msg::String state_msg;
//         switch (current_state_) {
//             case State::IDLE:
//                 state_msg.data = "Idle";
//                 break;
//             case State::ACCELERATING:
//                 state_msg.data = "Accelerating";
//                 break;
//             case State::CLOSINGGAP:
//                 state_msg.data = "ClosingGap";
//                 break;
//             case State::DECELERATING:
//                 state_msg.data = "Decelerating";
//                 break;
//             case State::ATTACHING:
//                 state_msg.data = "Attaching";
//                 break;
//             case State::EMERGENCY_STOP:
//                 state_msg.data = "Emergency Stop";
//                 break;
//             default:
//                 state_msg.data = "Uninitialized";
//                 break;
//         }
//         current_state_publisher_->publish(state_msg);
//     }

//     // Function to publish target velocity and distance
//     void publishTargetVelocityAndDistance() {
//         std_msgs::msg::Float64 velocity_msg;
//         velocity_msg.data = target_velocity_;
//         target_velocity_publisher_->publish(velocity_msg);

//         std_msgs::msg::Float64 distance_msg;
//         distance_msg.data = target_distance_;
//         target_distance_publisher_->publish(distance_msg);
//     }

//     // Callback for handling "order_info" messages
//     void orderInfoCallback(const std_msgs::msg::String::SharedPtr msg) {
//         try {
//             int task_id = std::stoi(msg->data);
//             current_task_id_ = task_id;
//             RCLCPP_INFO(this->get_logger(), "Received Task ID: %d", task_id);

//             // Based on task_id, set initial state or parameters
//             if (current_task_id_ == 1) {
//                 // Example: Task ID 1 might correspond to a specific behavior
//                 if (current_state_ == State::IDLE) {
//                     current_state_ = State::ACCELERATING;
//                     target_velocity_ = v_max_;
//                     target_distance_ = s_target_;
//                     RCLCPP_INFO(this->get_logger(), "Task ID 1 received. Switching to ACCELERATING state.");
//                 }
//             }
//             // Add more task handling as needed
//         } catch (const std::invalid_argument& e) {
//             RCLCPP_ERROR(this->get_logger(), "Invalid TaskID received: '%s'. Error: %s", msg->data.c_str(), e.what());
//         } catch (const std::out_of_range& e) {
//             RCLCPP_ERROR(this->get_logger(), "TaskID out of range: '%s'. Error: %s", msg->data.c_str(), e.what());
//         }
//     }

//     // Callback for handling "observer_info" messages
//     void observerInfoCallback(const observer_msgs::msg::ObserverInfo::SharedPtr msg) {
//         double smoothed_distance = smoothDistance(msg->distance);
//         smoothed_distance_ = smoothed_distance;

//         // Publish the smoothed distance
//         std_msgs::msg::Float64 smoothed_distance_msg;
//         smoothed_distance_msg.data = smoothed_distance;
//         smoothed_distance_publisher_->publish(smoothed_distance_msg);

//         // Check signal setting before state transitions
//         if (signal_detected_ && signal_setting_ == "red") {
//             // If the signal is red, stay in IDLE regardless of the distance
//             if (current_state_ != State::IDLE) {
//                 current_state_ = State::IDLE;
//                 target_velocity_ = 0.0;
//                 target_distance_ = 0.0;
//                 RCLCPP_INFO(this->get_logger(), "Signal is RED, switching to IDLE state.");
//                 publishCurrentState();
//                 publishTargetVelocityAndDistance();
//             }
//             return;  // Early exit to prevent further transitions
//         }

//         // If the signal is white or no signal detected, continue with state transitions
//         switch (current_state_) {
//             case State::IDLE:
//                 handleIdleState(smoothed_distance);
//                 break;
            
//             case State::ACCELERATING:
//                 handleAcceleratingState(smoothed_distance);
//                 break;
            
//             case State::CLOSINGGAP:
//                 handleClosingGapState(smoothed_distance);
//                 break;
            
//             case State::DECELERATING:
//                 handleDeceleratingState(smoothed_distance);
//                 break;
            
//             case State::ATTACHING:
//                 handleAttachingState(smoothed_distance);
//                 break;
            
//             case State::EMERGENCY_STOP:
//                 // Handle emergency stop logic if needed
//                 break;
            
//             default:
//                 break;
//         }

//         publishTargetVelocityAndDistance();
//         publishCurrentState();
//     }

//     // Handler for IDLE state transitions
//     void handleIdleState(double smoothed_distance) {
//         // Example condition to transition from IDLE to ACCELERATING
//         if (smoothed_distance > 40.0) {
//             current_state_ = State::ACCELERATING;
//             target_velocity_ = v_max_;
//             target_distance_ = s_target_;
//             RCLCPP_INFO(this->get_logger(), "Switching from IDLE to ACCELERATING state.");
//         }
//         // Add more conditions as needed
//     }

//     // Handler for ACCELERATING state transitions
//     void handleAcceleratingState(double smoothed_distance) {
//         if (smoothed_distance < 40.0) {
//             current_state_ = State::DECELERATING;
//             target_velocity_ = 0.0;
//             target_distance_ = (smoothed_distance > 10.0) ? smoothed_distance - 10.0 : 0.0;
//             RCLCPP_INFO(this->get_logger(), "Obstacle detected within 40 meters. Switching to DECELERATING state.");
//         }
//     }

//     // Handler for DECELERATING state transitions
//     void handleDeceleratingState(double smoothed_distance) {
//         if (current_task_id_ == 1 && smoothed_distance > 3.5 && smoothed_distance <= 10.0) {
//             current_state_ = State::ATTACHING;
//             target_velocity_ = 3.0;
//             target_distance_ = 5.0;
//             RCLCPP_INFO(this->get_logger(), "Switching to ATTACHING state.");
//         } else if (smoothed_distance > 10.0 && smoothed_distance < 40.0) {
//             current_state_ = State::CLOSINGGAP;
//             target_velocity_ = v_max_ / 2.0;
//             target_distance_ = 10.0;
//             RCLCPP_INFO(this->get_logger(), "Switching to CLOSINGGAP state.");
//         } else if (smoothed_distance > 40.0) {
//             if (!is_path_clear_timer_active_) {
//                 // Start the 2-second verification timer
//                 path_clear_start_time_ = this->now();
//                 is_path_clear_timer_active_ = true;
//                 RCLCPP_INFO(this->get_logger(), "Path appears clear, starting 2-second verification.");
//             } else {
//                 // Check if 2 seconds have passed with the path remaining clear
//                 if ((this->now() - path_clear_start_time_).seconds() > 2.0) {
//                     // Path has been clear for 2 seconds, safe to transition to ACCELERATING
//                     current_state_ = State::ACCELERATING;
//                     target_velocity_ = v_max_;
//                     target_distance_ = s_target_;
//                     is_path_clear_timer_active_ = false;  // Reset the timer flag
//                     RCLCPP_INFO(this->get_logger(), "Path confirmed clear for 2 seconds. Switching back to ACCELERATING.");
//                 }
//             }
//         }
//     }

//     // Handler for CLOSINGGAP state transitions
//     void handleClosingGapState(double smoothed_distance) {
//         if (current_task_id_ == 1 && smoothed_distance > 3.0 && smoothed_distance <= 10.0) {
//             current_state_ = State::ATTACHING;
//             target_velocity_ = 1.0;  // Slow speed for fine approach
//             target_distance_ = (smoothed_distance_ > 3.0) ? smoothed_distance_ - 3.0 : 1.0;
//             RCLCPP_INFO(this->get_logger(), "Task ID = 1, transitioning to ATTACHING state.");
//         } else if (smoothed_distance <= 10.0 && current_task_id_ != 1) {
//             // If not Task ID = 1, transition to DECELERATING to stop 10 meters before the obstacle
//             current_state_ = State::DECELERATING;
//             target_velocity_ = 0.0;
//             target_distance_ = 1.0;
//             RCLCPP_INFO(this->get_logger(), "Approaching obstacle, switching to DECELERATING state to stop 10 meters before obstacle.");
//         }
//     }

//     // Handler for ATTACHING state transitions
//     void handleAttachingState(double smoothed_distance) {
//         if (smoothed_distance < 2.5) {
//             target_velocity_ = 0.0;
//             target_distance_ = 0.5;
//             current_state_ = State::IDLE;
//             RCLCPP_INFO(this->get_logger(), "Attaching to the target vehicle successful. Switching to IDLE state.");
//             publishCurrentState();
//             publishTargetVelocityAndDistance();
//         }
//     }

//     // Callback for handling "vehicle_velocity" messages
//     void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
//         double time_elapsed = (this->now() - last_detection_time_).seconds();
//         double distance_travelled = last_velocity_ * time_elapsed;

//         // Publish "IDLE" only after the first velocity message is received
//         if (!has_published_idle_) {
//             if (msg->linear.x == 0.0) {
//                 current_state_ = State::IDLE;
//                 target_velocity_ = 0.0;
//                 target_distance_ = 0.0;
//                 has_published_idle_ = true;
//                 RCLCPP_INFO(this->get_logger(), "Vehicle velocity is 0. Publishing IDLE state.");
//                 publishCurrentState();
//             }
//             last_velocity_ = msg->linear.x;
//             last_detection_time_ = this->now();
//             return;
//         }

//         // DECELERATING: Adjust target distance dynamically
//         if (current_state_ == State::DECELERATING && smoothed_distance_ > 10) {
//             target_distance_ = smoothed_distance_ - 10.0;  // Continuously update target distance
//             if (msg->linear.x == 0.0) {  // Vehicle has come to a stop
//                 current_state_ = State::IDLE;
//                 target_velocity_ = 0.0;
//                 target_distance_ = 0.0;
//                 RCLCPP_INFO(this->get_logger(), "Vehicle stopped. Switching to IDLE state.");
//             }
//             publishTargetVelocityAndDistance();
//             last_velocity_ = msg->linear.x;
//             last_detection_time_ = this->now();
//             return;
//         }

//         // ATTACHING: Stop the vehicle when within 2.5 meters
//         if (current_state_ == State::ATTACHING && msg->linear.x > 0) {
//             if (smoothed_distance_ <= 5.0) {
//                 current_state_ = State::DECELERATING;
//                 target_velocity_ = 0.0;
//                 target_distance_ = smoothed_distance_ - 2.0;
//                 RCLCPP_INFO(this->get_logger(), "Reached target for ATTACHING. Switching to DECELERATING.");
//             }
//             publishTargetVelocityAndDistance();
//             last_velocity_ = msg->linear.x;
//             last_detection_time_ = this->now();
//         }

//         // STOPPED to IDLE after ATTACHING (only when completely stopped)
//         if ((current_state_ == State::DECELERATING || current_state_ == State::EMERGENCY_STOP) && msg->linear.x == 0.0) {
//             current_state_ = State::IDLE;
//             target_velocity_ = 0.0;
//             target_distance_ = 0.0;
//             RCLCPP_INFO(this->get_logger(), "Vehicle has stopped. Switching to IDLE state.");
//             publishCurrentState();
//         }

//         last_velocity_ = msg->linear.x;
//         last_detection_time_ = this->now();

//         publishTargetVelocityAndDistance();
//         publishCurrentState();
//     }

//     // Callback for handling "signal_detected" messages
//     void signalDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg) {
//         signal_detected_ = msg->data;
//         RCLCPP_INFO(this->get_logger(), "Signal detected: %s", signal_detected_ ? "true" : "false");
//     }

//     // Callback for handling "signal_setting" messages
//     void signalSettingCallback(const std_msgs::msg::String::SharedPtr msg) {
//         signal_setting_ = msg->data;
//         RCLCPP_INFO(this->get_logger(), "Signal setting: %s", signal_setting_.c_str());

//         // If signal is red, force vehicle into IDLE
//         if (signal_detected_ && signal_setting_ == "red") {
//             if (current_state_ != State::IDLE) {
//                 current_state_ = State::IDLE;
//                 target_velocity_ = 0.0;
//                 target_distance_ = 0.0;
//                 RCLCPP_INFO(this->get_logger(), "Signal is RED, switching to IDLE state.");
//                 publishCurrentState();
//                 publishTargetVelocityAndDistance();
//             }
//         } else if (signal_detected_ && signal_setting_ == "white") {
//             RCLCPP_INFO(this->get_logger(), "Signal is WHITE.");
//             // If previously in IDLE due to red signal, allow state transitions
//             if (current_state_ == State::IDLE) {
//                 // Example: Automatically transition to ACCELERATING if conditions are met
//                 if (smoothed_distance_ > 40.0) {
//                     current_state_ = State::ACCELERATING;
//                     target_velocity_ = v_max_;
//                     target_distance_ = s_target_;
//                     RCLCPP_INFO(this->get_logger(), "Signal is WHITE and conditions met. Switching to ACCELERATING state.");
//                     publishCurrentState();
//                     publishTargetVelocityAndDistance();
//                 }
//             }
//         }
//     }

//     // Timer callback to handle IDLE state duration
//     void idleTimerCallback() {
//         if (current_state_ == State::IDLE) {
//             std_msgs::msg::String task_msg;
//             task_msg.data = "Task Completed";
//             task_completed_publisher_->publish(task_msg);
//             RCLCPP_INFO(this->get_logger(), "Task Completed after staying in IDLE for 2 seconds.");
//         }
//     }

//     // Smoothed distance variable
//     double smoothed_distance_;
// };

// // Main function
// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<ActorNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }



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
//     ActorNode() : Node("actor_node"), 
//                   current_state_(State::UNINITIALIZED), // Start in an uninitialized state
//                   smoothing_window_size_(10), detection_threshold_(40.0), 
//                   last_velocity_(0.0), v_max_(15.0), s_target_(20.0), 
//                   has_published_idle_(false) { // Track whether IDLE has been published

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
//         target_distance_ = 0.0; // Default value, to be updated based on state
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
//         server_address_.sin_addr.s_addr = inet_addr("172.22.17.234");  // IP address as provided

//         RCLCPP_INFO(this->get_logger(), "Actor node initialized and UDP communication setup complete.");

//         // Timer to periodically send UDP messages
//         udp_timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(100),  // Send UDP message every 100 ms
//             [this]() {
//                 sendUDPMessage();  // Continuously send UDP messages
//             }
//         );

//           // Timer for checking if IDLE state has lasted 2 seconds
//         idle_timer_ = this->create_wall_timer(
//             std::chrono::seconds(2),
//             [this]() {
//                 if (current_state_ == State::IDLE) {
//                     std_msgs::msg::String task_msg;
//                     task_msg.data = "Task Completed";
//                     task_completed_publisher_->publish(task_msg);
//                     RCLCPP_INFO(this->get_logger(), "Task Completed after staying in IDLE for 2 seconds.");
//                 }
//             }
//         );
//     }

//     ~ActorNode() {
//         close(udp_socket_);
//     }

// private:
//     enum class State {
//         UNINITIALIZED,  // Initial state before receiving any velocity messages
//         IDLE,
//         ACCELERATING,
//         CLOSINGGAP,
//         DECELERATING,
//         STOPPED,
//         ATTACHING,
//         EMERGENCY_STOP
//     };

//     State current_state_;
//     int current_task_id_;  
//     double target_velocity_;
//     double target_distance_;
//     rclcpp::Time last_detection_time_;
//     double last_velocity_;
//     double v_max_;
//     double s_target_;
//     bool has_published_idle_;
//     double initial_decelerating_distance_;


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
//      rclcpp::TimerBase::SharedPtr idle_timer_;  // Timer for IDLE state

//     uint8_t encodeVelocity(double velocity) {
//         return static_cast<uint8_t>(std::min(std::max(velocity / 40.0 * 255.0, 0.0), 255.0));
//     }

//     uint8_t encodeDistance(double distance) {
//         double max_distance = 255.0; // Example: 255 meters max
//         return static_cast<uint8_t>(std::min(std::max(distance, 0.0), max_distance));
//     }

//     void sendUDPMessage() {
//      //   RCLCPP_INFO(this->get_logger(), "Sending UDP - Target Velocity: %f, Target Distance: %f", target_velocity_, target_distance_);
        
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
//            // RCLCPP_INFO(this->get_logger(), "Sent UDP message: Velocity: %d, Distance: %d", data[0], data[1]);
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
//             case State::UNINITIALIZED:
//                 // Do not publish anything in UNINITIALIZED state
//                 return;
//             case State::IDLE:
//                 state_msg.data = "Idle";
//                 break;
//             case State::ACCELERATING:
//                 state_msg.data = "Accelerating";
//                 break;
//             case State::CLOSINGGAP:
//                 state_msg.data = "CLOSINGGAP";
//                 break;
//             case State::DECELERATING:
//                 state_msg.data = "Decelerating";
//                 break;
//             // case State::STOPPED:
//             //     state_msg.data = "Stopped";
//             //     break;
//             case State::ATTACHING:
//                 state_msg.data = "Attaching";
//                 break;
//             case State::EMERGENCY_STOP:
//                 state_msg.data = "Emergency Stop";
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

//     void clearTaskCompletedMessage() {
//         std_msgs::msg::String empty_msg;
//         empty_msg.data = "";  // Publish an empty string to indicate the reset
//         task_completed_publisher_->publish(empty_msg);
//     }

// void orderInfoCallback(const std_msgs::msg::String::SharedPtr msg) {
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

//         // Clear the "Task Completed" message if a new task is set
//         if (task_id > 0) {
//             clearTaskCompletedMessage();
//             RCLCPP_INFO(this->get_logger(), "New task received, clearing 'Task Completed' state.");
//         }

//         // Emergency Stop
//         if (task_id == 0) {
//             current_state_ = State::EMERGENCY_STOP;
//             target_velocity_ = 0.0;
//             target_distance_ = 255.0;
//             RCLCPP_INFO(this->get_logger(), "Switching to EMERGENCY_STOP state.");
//         } else {
//             // Regular Task Handling
//             if (current_state_ == State::IDLE) {
//                 if (smoothed_distance_ > 40.0) {
//                     current_state_ = State::ACCELERATING;
//                     target_velocity_ = v_max_;
//                     target_distance_ = s_target_;
//                     RCLCPP_INFO(this->get_logger(), "Track free - switching to ACCELERATING state.");
//                 } else if (smoothed_distance_ > 10.0 && smoothed_distance_ < 40.0) {
//                     current_state_ = State::CLOSINGGAP;
//                     target_velocity_ = v_max_ / 2.0;
//                     target_distance_ = 10.0;
//                     RCLCPP_INFO(this->get_logger(), "Switching to CLOSINGGAP state.");
//                 } else if (task_id == 1 && smoothed_distance_ > 3.5 && smoothed_distance_ <= 10.0) {
//                     current_state_ = State::ATTACHING;
//                     target_velocity_ = 1.0;
//                     target_distance_ = 2.0;
//                     RCLCPP_INFO(this->get_logger(), "Switching to ATTACHING state.");
//                 }
//                 else {
//                     current_state_ == State::IDLE;

//                 }
//             }
//         }

//         publishTargetVelocityAndDistance();
//         publishCurrentState();

//     } catch (const std::invalid_argument& e) {
//         RCLCPP_ERROR(this->get_logger(), "Invalid TaskID received: '%s'. Error: %s", msg->data.c_str(), e.what());
//     } catch (const std::out_of_range& e) {
//         RCLCPP_ERROR(this->get_logger(), "TaskID out of range: '%s'. Error: %s", msg->data.c_str(), e.what());
//     }
// }

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
//         // initial_decelerating_distance_ = (smoothed_distance_ > 10.0) ? smoothed_distance_ - 10.0 : 0.0;  // de-comment these lines for handing over the first approximated stop distance.
//         // target_distance_ = initial_decelerating_distance_;
//         target_distance_ = (smoothed_distance_ > 10.0) ? smoothed_distance_ - 10.0 : 0.0;
//           target_distance_ = (smoothed_distance_ > 10.0) ? smoothed_distance_ - 10.0 : 0.0;
//     RCLCPP_INFO(this->get_logger(), "Updated target distance to: %f", target_distance_);
//     publishTargetVelocityAndDistance();
//         RCLCPP_INFO(this->get_logger(), "Obstacle detected within 40 meters. Switching to DECELERATING state.");
//     }

//      // Transition from ACCELERATING to DECELERATING if an obstacle is detected within 40 meters
//     if (current_state_ == State::CLOSINGGAP && smoothed_distance_ > 40.0) {
//         current_state_ = State::ACCELERATING;
//         target_velocity_ = v_max_;
//         target_distance_ = s_target_;
//         RCLCPP_INFO(this->get_logger(), "Obstacle detected within 40 meters. Switching to DECELERATING state.");
//     }

//     // Transition from DECELERATING to ACCELERATING if the path clears
//     if (current_state_ == State::DECELERATING && smoothed_distance_ > 40.0) {
//         current_state_ = State::ACCELERATING;
//         target_velocity_ = v_max_;
//         target_distance_ = s_target_;
//         RCLCPP_INFO(this->get_logger(), "Obstacle cleared, switching back to ACCELERATING.");
//     }

//       // Transition from DECELERATING to ACCELERATING if the path clears
//     if (current_state_ == State::ATTACHING && smoothed_distance_ > 40.0) {
//         current_state_ = State::ACCELERATING;
//         target_velocity_ = v_max_;
//         target_distance_ = s_target_;
//         RCLCPP_INFO(this->get_logger(), "Obstacle cleared, switching back to ACCELERATING.");
//     }


//     // Handle CLOSINGGAP state: target is to stop 10 meters in front of the obstacle
//     if (current_state_ == State::CLOSINGGAP) {
//         target_distance_ = (smoothed_distance_ > 10.0) ? smoothed_distance_ - 10.0 : 1.0;

//         // If Task ID = 1 and within 10 meters, transition to ATTACHING state
//         if (smoothed_distance_ > 3.0 && smoothed_distance <= 10.0 && current_task_id_ == 1) {
//             current_state_ = State::ATTACHING;
//             target_velocity_ = 1.0;  // Slow speed for fine approach
//             target_distance_ = (smoothed_distance_ > 3.0) ? smoothed_distance_ - 3.0 : 1.0;
//             RCLCPP_INFO(this->get_logger(), "Task ID = 1, transitioning to ATTACHING state.");
//         } else if (smoothed_distance_ <= 10.0 && current_task_id_ != 1) {
//             // If not Task ID = 1, transition to DECELERATING to stop 10 meters before the obstacle
//             current_state_ = State::DECELERATING;
//             target_velocity_ = 0.0;
//             target_distance_ = 0.0;
//             RCLCPP_INFO(this->get_logger(), "Approaching obstacle, switching to DECELERATING state to stop 10 meters before obstacle.");
//         }
//     }

//      // Handle transition to CLOSING GAP or ATTACHING after approaching
//     if (current_state_ == State::IDLE && current_task_id_ == 1) {
//         // within 10 meters, transition to ATTACHING state
//         if (smoothed_distance_ > 3.0 && smoothed_distance_ && smoothed_distance_ <= 10.0) {
//             current_state_ = State::ATTACHING;
//             target_velocity_ = 1.0;  // Slow speed for fine approach
//             target_distance_ = 5.0 ;
//             RCLCPP_INFO(this->get_logger(), "Task ID = 1, transitioning to ATTACHING state.");
//         } else if (smoothed_distance_ > 10.0 && smoothed_distance_ <= 20.0 ) {
//             // If not Task ID = 1, transition to DECELERATING to stop 10 meters before the obstacle
//             current_state_ = State::CLOSINGGAP;
//             target_velocity_ = v_max_ / 2.0;
//             target_distance_ = s_target_/2;
//             RCLCPP_INFO(this->get_logger(), "Approaching obstacle, switching to DECELERATING state to stop 10 meters before obstacle.");
//         }
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

//     // DECELERATING: Adjust target distance dynamically
//     if (current_state_ == State::DECELERATING) {
       
//         if (msg->linear.x == 0.0) {  // Vehicle has come to a stop
//             current_state_ = State::IDLE;
//             // target_velocity_ = 0.0;
//             // target_distance_ = 0.0;
//             RCLCPP_INFO(this->get_logger(), "Vehicle stopped. Switching to STOPPED state.");
//         }
//         publishTargetVelocityAndDistance();
//         return;
//     }

//     // ATTACHING: Stop the vehicle when within 2.5 meters
//     if (current_state_ == State::ATTACHING) {
//         if (smoothed_distance_ <= 3.5) {
//             target_velocity_ = 0.0;
//             target_distance_ = 1.0;
//             RCLCPP_INFO(this->get_logger(), "Reached target for ATTACHING. Switching to STOPPED.");
//         } else {
//             target_distance_ = std::max(target_distance_ - distance_travelled, 0.0);
//         }
//         publishTargetVelocityAndDistance();
        
//     }

//     //STOPPED to IDLE after ATTACHING (only when completely stopped)
//     if (current_state_ == State::ATTACHING && msg->linear.x == 0.0 && smoothed_distance_ < 3.0) {
//         current_state_ = State::IDLE;
//         RCLCPP_INFO(this->get_logger(), "Vehicle has stopped after ATTACHING. Switching to IDLE state.");
//         publishCurrentState();
       
//     }
//       //STOPPED to IDLE after ATTACHING (only when completely stopped)
//     if (current_state_ == State::EMERGENCY_STOP && msg->linear.x == 0.0) {
//         current_state_ = State::IDLE;
//         RCLCPP_INFO(this->get_logger(), "Vehicle has stopped after ATTACHING. Switching to IDLE state.");
//         publishCurrentState();
       
//     }

//     last_velocity_ = msg->linear.x;
//     last_detection_time_ = this->now();

//     publishTargetVelocityAndDistance();
//     publishCurrentState();
// }
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
//     ActorNode() : Node("actor_node"), 
//                   current_state_(State::UNINITIALIZED), // Start in an uninitialized state
//                   smoothing_window_size_(10), detection_threshold_(50.0), 
//                   last_velocity_(0.0), v_max_(15.0), s_target_(40.0), 
//                   has_published_idle_(false) { // Track whether IDLE has been published

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
//         target_distance_ = 0.0; // Default value, to be updated based on state
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
//         server_address_.sin_port = htons(2869);  // Port number as provided
//         server_address_.sin_addr.s_addr = inet_addr("10.8.3.2");  //receiver port (Kinematics model)

//         RCLCPP_INFO(this->get_logger(), "Actor node initialized and UDP communication setup complete.");

//         // Timer to periodically send UDP messages
//         udp_timer_ = this->create_wall_timer(
//             std::chrono::milliseconds(100),  // Send UDP message every 100 ms
//             [this]() {
//                 sendUDPMessage();  // Continuously send UDP messages
//             }
//         );

//           // Timer for checking if IDLE state has lasted 2 seconds
//         idle_timer_ = this->create_wall_timer(
//             std::chrono::seconds(2),
//             [this]() {
//                 if (current_state_ == State::IDLE) {
//                     std_msgs::msg::String task_msg;
//                     task_msg.data = "Task Completed";
//                     task_completed_publisher_->publish(task_msg);
//                     RCLCPP_INFO(this->get_logger(), "Task Completed after staying in IDLE for 2 seconds.");
//                 }
//             }
//         );
//     }

//     ~ActorNode() {
//         close(udp_socket_);
//     }

// private:
//     enum class State {
//         UNINITIALIZED,  // Initial state before receiving any velocity messages
//         IDLE,
//         ACCELERATING,
//         CLOSINGGAP,
//         DECELERATING,
//         STOPPED,
//         ATTACHING,
//         EMERGENCY_STOP
//     };

//     State current_state_;
//     int current_task_id_;  
//     double target_velocity_;
//     double target_distance_;
//     rclcpp::Time last_detection_time_;
//     double last_velocity_;
//     double v_max_;
//     double s_target_;
//     bool has_published_idle_;

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
//     rclcpp::TimerBase::SharedPtr idle_timer_;  // Timer for IDLE state
//     rclcpp::Time path_clear_start_time_;
//     bool is_path_clear_timer_active_;

//     uint8_t encodeVelocity(double velocity) {
//         return static_cast<uint8_t>(std::min(std::max(velocity / 40.0 * 255.0, 0.0), 255.0));
//     }

//     uint8_t encodeDistance(double distance) {
//         double max_distance = 255.0; // Example: 255 meters max
//         return static_cast<uint8_t>(std::min(std::max(distance, 0.0), max_distance));
//     }

//     void sendUDPMessage() {
//      //   RCLCPP_INFO(this->get_logger(), "Sending UDP - Target Velocity: %f, Target Distance: %f", target_velocity_, target_distance_);
        
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
//           //  RCLCPP_INFO(this->get_logger(), "Sent UDP message: Velocity: %d, Distance: %d", data[0], data[1]);
//         }
//     }

//    double smoothDistance(double new_distance) {
//     // Check if the new distance is infinity
//     if (std::isinf(new_distance)) {
//         // Verify if it's infinite for 2 seconds
//         auto start_time = std::chrono::steady_clock::now();
//         bool is_infinite = true;

//         while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(2)) {
//             if (!std::isinf(new_distance)) {
//                 is_infinite = false;
//                 break;
//             }
//             std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Sleep for 100ms between checks
//         }

//         // If the value remained infinite for 2 seconds, return infinity
//         if (is_infinite) {
//             return std::numeric_limits<double>::infinity();
//         }
//     }

//     // If the new distance is neither infinite nor NaN, proceed with smoothing
//     if (!std::isnan(new_distance) && !std::isinf(new_distance)) {
//         if (distance_history_.size() >= smoothing_window_size_) {
//             distance_history_.pop_front();
//         }
//         distance_history_.push_back(new_distance);
//     }

//     // If there's no history yet, return 0.0
//     if (distance_history_.empty()) {
//         return 0.0;
//     }

//     // Compute the smoothed value by averaging the history
//     double sum = 0.0;
//     for (double distance : distance_history_) {
//         sum += distance;
//     }

//     return sum / distance_history_.size();
// }

//     void publishCurrentState() {
//         std_msgs::msg::String state_msg;
//         switch (current_state_) {
//             case State::UNINITIALIZED:
//                 // Do not publish anything in UNINITIALIZED state
//                 return;
//             case State::IDLE:
//                 state_msg.data = "Idle";
//                 break;
//             case State::ACCELERATING:
//                 state_msg.data = "Accelerating";
//                 break;
//             case State::CLOSINGGAP:
//                 state_msg.data = "CLOSINGGAP";
//                 break;
//             case State::DECELERATING:
//                 state_msg.data = "Decelerating";
//                 break;
//             // case State::STOPPED:
//             //     state_msg.data = "Stopped";
//             //     break;
//             case State::ATTACHING:
//                 state_msg.data = "Attaching";
//                 break;
//             case State::EMERGENCY_STOP:
//                 state_msg.data = "Emergency Stop";
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

//     void clearTaskCompletedMessage() {
//         std_msgs::msg::String empty_msg;
//         empty_msg.data = "";  // Publish an empty string to indicate the reset
//         task_completed_publisher_->publish(empty_msg);
//     }

// void orderInfoCallback(const std_msgs::msg::String::SharedPtr msg) {
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

//         // Clear the "Task Completed" message if a new task is set
//         if (task_id > 0) {
//             clearTaskCompletedMessage();
//             RCLCPP_INFO(this->get_logger(), "New task received, clearing 'Task Completed' state.");
//         }

//         // Emergency Stop
//         if (task_id == 0) {
//             current_state_ = State::EMERGENCY_STOP;
//             target_velocity_ = 0.0;
//             target_distance_ = 0.5;
//             RCLCPP_INFO(this->get_logger(), "Switching to EMERGENCY_STOP state.");
//         } else {
//             // Regular Task Handling
//             if (current_state_ == State::IDLE) {
//                 if (smoothed_distance_ > 40.0) {
//                     current_state_ = State::ACCELERATING;
//                     target_velocity_ = v_max_;
//                     target_distance_ = s_target_;
//                     RCLCPP_INFO(this->get_logger(), "Track free - switching to ACCELERATING state.");
//                 } else if (smoothed_distance_ > 10.0 && smoothed_distance_ < 40.0) {
//                     current_state_ = State::CLOSINGGAP;
//                     target_velocity_ = v_max_ / 2.0;
//                     target_distance_ = 10.0;
//                     RCLCPP_INFO(this->get_logger(), "Switching to CLOSINGGAP state.");
//                 } else if (task_id == 1 && smoothed_distance_ > 3.5 && smoothed_distance_ <= 10.0) {
//                     current_state_ = State::ATTACHING;
//                     target_velocity_ = 3.0;
//                     target_distance_ = 5.0;
//                     RCLCPP_INFO(this->get_logger(), "Switching to ATTACHING state.");
//                 }
//                 else {
//                     current_state_ == State::IDLE;
//                     target_velocity_ = 0.0;
//             target_distance_ = 0.0;

//                 }
//             }
//         }

//         publishTargetVelocityAndDistance();
//         publishCurrentState();

//     } catch (const std::invalid_argument& e) {
//        // RCLCPP_ERROR(this->get_logger(), "Invalid TaskID received: '%s'. Error: %s", msg->data.c_str(), e.what());
//     } catch (const std::out_of_range& e) {
//        // RCLCPP_ERROR(this->get_logger(), "TaskID out of range: '%s'. Error: %s", msg->data.c_str(), e.what());
//     }
// }

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
//         target_distance_ = (smoothed_distance_ > 10.0) ? smoothed_distance_ - 10.0 : 0.0;
//         RCLCPP_INFO(this->get_logger(), "Obstacle detected within 40 meters. Switching to DECELERATING state.");
//     }

//      // Transition from ACCELERATING to DECELERATING if an obstacle is detected within 40 meters
//     if (current_state_ == State::CLOSINGGAP && smoothed_distance_ > 40.0) {
//         current_state_ = State::ACCELERATING;
//         target_velocity_ = v_max_;
//         target_distance_ = s_target_;
//         RCLCPP_INFO(this->get_logger(), "Obstacle detected within 40 meters. Switching to DECELERATING state.");
//     }

//        // Transition from DECELERATION to STOP during ATTACHING
//     if (current_state_ == State::DECELERATING && current_task_id_ == 1 && smoothed_distance_ < 2.5) {
//         target_velocity_ = 0.0;
//         target_distance_ = 0.5;
//         RCLCPP_INFO(this->get_logger(), "Attaching to the target vehicle successfull.");
//     }

//     // Transition from DECELERATING to ACCELERATING if the path clears
//     if (current_state_ == State::DECELERATING && smoothed_distance_ > 40.0) {
//         if (!is_path_clear_timer_active_) {
//             // Start the 2-second verification timer
//             path_clear_start_time_ = this->now();
//             is_path_clear_timer_active_ = true;
//             RCLCPP_INFO(this->get_logger(), "Path appears clear, starting 2-second verification.");
//         } else {
//             // Check if 2 seconds have passed with the path remaining clear
//             if ((this->now() - path_clear_start_time_).seconds() > 2.0) {
//                 // Path has been clear for 2 seconds, safe to transition to ACCELERATING
//                 current_state_ = State::ACCELERATING;
//                 target_velocity_ = v_max_;
//                 target_distance_ = s_target_;
//                 is_path_clear_timer_active_ = false;  // Reset the timer flag
//                 RCLCPP_INFO(this->get_logger(), "Path confirmed clear for 2 seconds. Obstacle cleared, switching back to ACCELERATING.");
//             }
//         }
//     } else {
//         // Reset the timer if the distance drops below 40 meters during the verification period
//         is_path_clear_timer_active_ = false;
//     }

//     // Transition from ATTACHING to ACCELERATING if the path has been clear for 2 seconds
//     if (current_state_ == State::ATTACHING && smoothed_distance_ > 40.0) {
//         if (!is_path_clear_timer_active_) {
//             // Start the 2-second verification timer
//             path_clear_start_time_ = this->now();
//             is_path_clear_timer_active_ = true;
//             RCLCPP_INFO(this->get_logger(), "Path appears clear, starting 2-second verification.");
//         } else {
//             // Check if 2 seconds have passed with the path remaining clear
//             if ((this->now() - path_clear_start_time_).seconds() > 2.0) {
//                 // Path has been clear for 2 seconds, safe to transition to ACCELERATING
//                 current_state_ = State::ACCELERATING;
//                 target_velocity_ = v_max_;
//                 target_distance_ = s_target_;
//                 is_path_clear_timer_active_ = false;  // Reset the timer flag
//                 RCLCPP_INFO(this->get_logger(), "Path confirmed clear for 2 seconds. Switching to ACCELERATING.");
//             }
//         }
//     } else {
//         // Reset the timer if the distance drops below 40 meters during the verification period
//         is_path_clear_timer_active_ = false;
//     }

//     // Handle CLOSINGGAP state: target is to stop 10 meters in front of the obstacle
//     if (current_state_ == State::CLOSINGGAP) {
//        // target_distance_ = (smoothed_distance_ > 10.0) ? smoothed_distance_ - 10.0 : 1.0;

//         // If Task ID = 1 and within 10 meters, transition to ATTACHING state
//         if (smoothed_distance_ > 3.0 && smoothed_distance <= 10.0 && current_task_id_ == 1) {
//             current_state_ = State::ATTACHING;
//             target_velocity_ = 1.0;  // Slow speed for fine approach
//             target_distance_ = (smoothed_distance_ > 3.0) ? smoothed_distance_ - 3.0 : 1.0;
//             RCLCPP_INFO(this->get_logger(), "Task ID = 1, transitioning to ATTACHING state.");
//         } else if (smoothed_distance_ <= 10.0 && current_task_id_ != 1) {
//             // If not Task ID = 1, transition to DECELERATING to stop 10 meters before the obstacle
//             current_state_ = State::DECELERATING;
//             target_velocity_ = 0.0;
//             target_distance_ = 1.0;
//             RCLCPP_INFO(this->get_logger(), "Approaching obstacle, switching to DECELERATING state to stop 10 meters before obstacle.");
//         }
//     }

//      // Handle transition to CLOSING GAP or ATTACHING after approaching
//     if (current_state_ == State::IDLE && current_task_id_ == 1) {
//         // within 10 meters, transition to ATTACHING state
//         if (smoothed_distance_ > 3.0 && smoothed_distance_ && smoothed_distance_ <= 10.0) {
//             current_state_ = State::ATTACHING;
//             target_velocity_ = 3.0;  // Slow speed for fine approach
//             target_distance_ = 5.0 ;
//             RCLCPP_INFO(this->get_logger(), "Task ID = 1, transitioning to ATTACHING state.");
//         } else if (smoothed_distance_ > 10.0 && smoothed_distance_ <= 20.0 ) {
//             // If not Task ID = 1, transition to DECELERATING to stop 10 meters before the obstacle
//             current_state_ = State::CLOSINGGAP;
//             target_velocity_ = v_max_ / 2.0;
//             target_distance_ = 15.0;
//             RCLCPP_INFO(this->get_logger(), "Approaching obstacle, switching to DECELERATING state to stop 10 meters before obstacle.");
//          }
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
//             target_velocity_ = 0.0;
//             target_distance_ = 0.0;
//             has_published_idle_ = true;
//             RCLCPP_INFO(this->get_logger(), "Vehicle velocity is 0. Publishing IDLE state.");
//             publishCurrentState();
//         }
//         return;
//     }

//     // DECELERATING: Adjust target distance dynamically
//     if (current_state_ == State::DECELERATING && smoothed_distance_ > 10) {
//        target_distance_ = smoothed_distance_ - 10.0;  // Continuously update target distance
//         if (msg->linear.x == 0.0) {  // Vehicle has come to a stop
//             current_state_ = State::IDLE;
//             target_velocity_ = 0.0;
//             target_distance_ = 0.0;
//             RCLCPP_INFO(this->get_logger(), "Vehicle stopped. Switching to STOPPED state.");
//         }
//         publishTargetVelocityAndDistance();
//         return;
//     }

//     // ATTACHING: Stop the vehicle when within 2.5 meters
//     if (current_state_ == State::ATTACHING && msg->linear.x > 0) {
//         if (smoothed_distance_ <= 5.0) {
//             current_state_ = State::DECELERATING;
//             target_velocity_ = 0.0;
//             target_distance_ = smoothed_distance_ - 2.0;
//             RCLCPP_INFO(this->get_logger(), "Reached target for ATTACHING. Switching to STOPPED.");
//          } //else {
//         //     target_distance_ = std::max(target_distance_ - distance_travelled, 0.0);
//         // }
//         publishTargetVelocityAndDistance();
        
//     }

//     //STOPPED to IDLE after ATTACHING (only when completely stopped)
//     if (current_state_ == State::DECELERATING && msg->linear.x == 0.0 /*&& smoothed_distance_ < 3.0*/) {
//         current_state_ = State::IDLE;
//         target_velocity_ = 0.0;
//         target_distance_ = 0.0;
//         RCLCPP_INFO(this->get_logger(), "Vehicle has stopped after ATTACHING. Switching to IDLE state.");
//         publishCurrentState();
       
//     }
//       //STOPPED to IDLE after ATTACHING (only when completely stopped)
//     if (current_state_ == State::EMERGENCY_STOP && msg->linear.x == 0.0) {
//         current_state_ = State::IDLE;
//         RCLCPP_INFO(this->get_logger(), "Vehicle has stopped after ATTACHING. Switching to IDLE state.");
//         publishCurrentState();
       
//     }

//     last_velocity_ = msg->linear.x;
//     last_detection_time_ = this->now();

//     publishTargetVelocityAndDistance();
//     publishCurrentState();
// }
//     double smoothed_distance_;
// };

// int main(int argc, char* argv[]) {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<ActorNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;

// }