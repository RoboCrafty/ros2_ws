#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/srv/load_map.hpp"
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <filesystem> // C++17 feature, include <experimental/filesystem> for older compilers
#include <regex>

namespace fs = std::filesystem;

// #include <chrono> 

// Define a constant for the timeout duration (20 seconds)
// const std::chrono::seconds TIMEOUT_DURATION(20);


#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"

class CustomNode : public rclcpp::Node
{
public:
    CustomNode() : Node("sim-multi_floor_goal")
    {
        // Ask the user for the starting floor
        std::cout << "Please enter the starting floor (e.g., 0, 1, 2, ...): ";
        std::cin >> starting_floor_;
        std::cout << "Current floor selected as floor " << starting_floor_ << std::endl;
        current_floor_ = starting_floor_;
        // Get elevator coordinates from parameters
        this->declare_parameter("elevator_goal_x", -6.14521);
        this->declare_parameter("elevator_goal_y", -3.1067);
        this->declare_parameter("elevator_goal_z", 0.0);
    
        this->get_parameter("elevator_goal_x", elevator_goal_x_);
        this->get_parameter("elevator_goal_y", elevator_goal_y_);
        this->get_parameter("elevator_goal_z", elevator_goal_z_);

        loadFloorMapConfig();

        goal_subscriber_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "user_goal",
            10,
            std::bind(&CustomNode::goal_callback, this, std::placeholders::_1));

        goal_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "goal_pose",
            10);
        elevator_publisher_ = this->create_publisher<std_msgs::msg::Int32>(
            "/model/elevator/cmd",
            10);

        map_load_client_ = this->create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");

        // Create a timer for the timeout
        // timer_ = this->create_wall_timer(TIMEOUT_DURATION, std::bind(&CustomNode::timeoutCallback, this));

        // Initialize the action client
        action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

        
        // goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal", 10);
        feedback_sub_ = this->create_subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>(
            "/navigate_to_pose/_action/feedback", 10, std::bind(&CustomNode::feedbackCallback, this, std::placeholders::_1));

         elevator_feedback_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/model/elevator/state",
            10,
            std::bind(&CustomNode::elevatorFeedbackCallback, this, std::placeholders::_1));


    }

private:
    void goal_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        // int current_floor = static_cast<int>(msg->position.z);
        // Check if goal is on the same floor or different floor
        // For simplicity, let's assume if the z coordinate is different, it's on a different floor
        if (msg->position.z != current_floor_)
        {
           
    
            // Goal is on a different floor, publish elevator coordinates
            auto elevator_goal = geometry_msgs::msg::PoseStamped();
            elevator_goal.header.frame_id = "map"; // Example frame ID
            elevator_goal.header.stamp = this->now(); // Current time
            elevator_goal.pose.position.x = elevator_goal_x_; // Elevator coordinates from parameters
            elevator_goal.pose.position.y = elevator_goal_y_;
            elevator_goal.pose.position.z = 0.0;
            auto elevator_command = std::make_shared<std_msgs::msg::Int32>();

            RCLCPP_INFO(this->get_logger(), ANSI_COLOR_CYAN "Goal is on a different level, sending coordinates of elevator and calling elevator to current floor: %d" ANSI_COLOR_RESET, current_floor_);
            // Send goal to action server
            auto goal_pub = nav2_msgs::action::NavigateToPose::Goal();
            goal_pub.pose = elevator_goal;
            auto goal_handle_future = action_client_->async_send_goal(goal_pub);
            // RCLCPP_INFO(this->get_logger(), "Goal sent to action server.");

            elevator_command->data = current_floor_;
            elevator_publisher_->publish(*elevator_command);


            x = 1;
            done = false;
            is_elevator_goal = true;
            goal_floor = msg->position.z;

            user_goal_x_ = msg->position.x;
            user_goal_y_ = msg->position.y;
            user_goal_z_ = msg->position.z;

    
        }
        else
        {   
            msg->position.z = 0.0;
            auto goal_on_same_floor = geometry_msgs::msg::PoseStamped();
            goal_on_same_floor.header.frame_id = "map"; // Example frame ID
            goal_on_same_floor.header.stamp = this->now(); // Current time
            goal_on_same_floor.pose.position.x = msg->position.x; // Use x from user_goal
            goal_on_same_floor.pose.position.y = msg->position.y; // Use y from user_goal
            goal_on_same_floor.pose.position.z = 0.0; // Assuming the same floor
            // goal_publisher_->publish(*goal_on_same_floor);
            RCLCPP_INFO(this->get_logger(), ANSI_COLOR_CYAN "Received Goal is on the same floor, sending coordinates straight to nav2: %f, %f, %f" ANSI_COLOR_RESET,
                        goal_on_same_floor.pose.position.x, goal_on_same_floor.pose.position.y,
                        goal_on_same_floor.pose.position.z);

            // Send goal to action server
            auto goal_pub = nav2_msgs::action::NavigateToPose::Goal();
            goal_pub.pose = goal_on_same_floor;
            auto goal_handle_future = action_client_->async_send_goal(goal_pub);
            RCLCPP_INFO(this->get_logger(), ANSI_COLOR_MAGENTA "Goal sent to action server." ANSI_COLOR_RESET);
            x=1;
            done = false;
            is_elevator_goal = false;
            
        }
    }

    void feedbackCallback(const nav2_msgs::action::NavigateToPose_FeedbackMessage::SharedPtr feedback)
    {
        x= x+1;
        last_feedback_distance_remaining_ = feedback->feedback.distance_remaining;
        // RCLCPP_INFO(this->get_logger(), "feedback from navigate to pose: %f, Value of x: %d, value of done: %i, value of is elv goal: %i", feedback->feedback.distance_remaining, x, done, is_elevator_goal);
        if (feedback->feedback.distance_remaining < 0.1 && feedback->feedback.distance_remaining > 0 && x > 15 && !done && is_elevator_goal)
        {
            x = 0;
            done = true;
            RCLCPP_INFO(this->get_logger(), ANSI_COLOR_GREEN "Elevator goal reached!" ANSI_COLOR_RESET);
            done2 = true;
            // Command the elevator to go to the floor specified in the user_goal
            auto elevator_command = std::make_shared<std_msgs::msg::Int32>();
            elevator_command->data = goal_floor; // Replace user_goal_floor_ with appropriate variable
            elevator_publisher_->publish(*elevator_command);
            RCLCPP_INFO(this->get_logger(), ANSI_COLOR_MAGENTA "Sent command to elevator to go to floor: %d" ANSI_COLOR_RESET, goal_floor);

            // Goal is on a different floor, load the corresponding map YAML file
            auto it = floor_map_config_.find(user_goal_z_);
            if (it != floor_map_config_.end())
            {
                std::string map_yaml = it->second;
                // Call the map loading service
                loadMap(map_yaml);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Map YAML file not found for floor %f", user_goal_z_);
                return;
            }
            
            // Reset flag for future goals
            is_elevator_goal = false;

            // Update current floor variable
            current_floor_ = goal_floor;

            take_elevator_feedback = true;
            
        }
        else if (feedback->feedback.distance_remaining < 0.1 && feedback->feedback.distance_remaining > 0 && x > 15 && !done)
        {   
            x = 0;    
            done = true;
            RCLCPP_INFO(this->get_logger(), ANSI_COLOR_GREEN "Goal reached!" ANSI_COLOR_RESET);   
            take_elevator_feedback = false;      
            is_elevator_goal = false;  
        }

        
    }

    void elevatorFeedbackCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {        
        //Check if the elevator has reached the goal floor
        
        if (msg->data == goal_floor && take_elevator_feedback)
        {   

            take_elevator_feedback = false;
            is_elevator_goal = false;


            RCLCPP_INFO(this->get_logger(), ANSI_COLOR_GREEN "Elevator has reached the requested floor: %d" ANSI_COLOR_RESET, goal_floor);
            // Send the coordinates from the initial user_goal to the elevator
            auto elevator_goal = geometry_msgs::msg::PoseStamped();
            elevator_goal.header.frame_id = "map"; // Example frame ID
            elevator_goal.header.stamp = this->now(); // Current time
            elevator_goal.pose.position.x = user_goal_x_; // Use the coordinates from the initial user_goal
            elevator_goal.pose.position.y = user_goal_y_;
            elevator_goal.pose.position.z = user_goal_z_;

            // Publish the goal to the action server
            auto goal_pub = nav2_msgs::action::NavigateToPose::Goal();
            goal_pub.pose = elevator_goal;
            auto goal_handle_future = action_client_->async_send_goal(goal_pub);
            RCLCPP_INFO(this->get_logger(), ANSI_COLOR_MAGENTA "Sent initial goal to action server after elevator reached the floor." ANSI_COLOR_RESET);
            done = false;
            x=0;
        }
    }

    void loadFloorMapConfig()
    {
        std::string maps_dir = "/home/crabbycat/ros2_ws/src/tarkbot_robot/maps/";
        std::regex floor_regex("Floor(\\d+)_.*\\.yaml");

        RCLCPP_INFO(this->get_logger(), "Loading floor map configuration from directory: %s", maps_dir.c_str());

        for (const auto& entry : fs::directory_iterator(maps_dir))
        {
            if (fs::is_regular_file(entry))
            {
                std::string filename = entry.path().filename().string();
                std::smatch match;

                if (std::regex_search(filename, match, floor_regex))
                {
                    int floor = std::stoi(match[1].str());
                    std::string map_yaml = entry.path().string();
                    floor_map_config_[floor] = map_yaml;

                    RCLCPP_INFO(this->get_logger(), "Found floor map: Floor %d, Map YAML: %s", floor, map_yaml.c_str());
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Ignoring file with unsupported filename format: %s", filename.c_str());
                }
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Ignoring directory entry: %s", entry.path().c_str());
            }
        }
    }

    void loadMap(const std::string& map_url)
    {
        RCLCPP_INFO(this->get_logger(), ANSI_COLOR_CYAN "Loaded Map for new floor." ANSI_COLOR_RESET);

        auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
        request->map_url = map_url;

        // Send the service request
        auto future = map_load_client_->async_send_request(request);
        // rclcpp::spin_until_future_complete(this->get_node_base_interface(), future);

        

        // if (future.wait_for(std::chrono::seconds(1)) == std::future_status::ready)
        // {
        //     auto result = future.get();
        //     if (result->success)
        //     {
        //         RCLCPP_INFO(this->get_logger(), "Map loaded successfully.");
        //     }
        //     else
        //     {
        //         RCLCPP_ERROR(this->get_logger(), "Failed to load map.");
        //     }
        // }
        // else
        // {
        //     RCLCPP_ERROR(this->get_logger(), "Service call to load map timed out.");
        // }
    }

    // void timeoutCallback()
    // {
    //     // If the goal involves using the elevator and the distance remaining is not close to zero after 20 seconds,
    //     // send a command to the elevator with the current floor to open the door again
    //     if (done2 && last_feedback_distance_remaining_ > 0.1) // Modify the threshold as needed
    //     {
    //         done2 = false;
    //         // Create a command to send to the elevator with the current floor
    //         auto elevator_command = std::make_shared<std_msgs::msg::Int32>();
    //         elevator_command->data = current_floor_;
    //         elevator_publisher_->publish(*elevator_command);

    //         RCLCPP_INFO(this->get_logger(), ANSI_COLOR_YELLOW "Timeout: Sending command to elevator to open the door again." ANSI_COLOR_RESET);
    //     }
    // }


    // rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr elevator_publisher_;
    rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr map_load_client_;

    rclcpp::Subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>::SharedPtr feedback_sub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_reached_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr elevator_feedback_sub_;

    // Member variable to store floor map configuration
    std::unordered_map<int, std::string> floor_map_config_;

    int starting_floor_;
    int current_floor_;
    double elevator_goal_x_;
    double elevator_goal_y_;
    double elevator_goal_z_;
    bool goal_reached_;
    int x;
    bool done;
    bool done2 = false;
    bool is_elevator_goal;
    int goal_floor;
    bool take_elevator_feedback;

    double user_goal_x_;
    double user_goal_y_;
    double user_goal_z_;

    double last_feedback_distance_remaining_;
    
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CustomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
