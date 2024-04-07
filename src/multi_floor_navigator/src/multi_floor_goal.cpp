#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class CustomNode : public rclcpp::Node
{
public:
    CustomNode() : Node("multi_floor_goal")
    {
        // Ask the user for the starting floor
        std::cout << "Please enter the starting floor (e.g., 1, 2, 3, ...): ";
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

            RCLCPP_INFO(this->get_logger(), "Goal is on a different level, sending coordinates of elevator and calling elevator to current floor: %d", current_floor_);
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
            RCLCPP_INFO(this->get_logger(), "Received Goal is on the same floor, sending coordinates straight to nav2: %f, %f, %f",
                        goal_on_same_floor.pose.position.x, goal_on_same_floor.pose.position.y,
                        goal_on_same_floor.pose.position.z);

            // Send goal to action server
            auto goal_pub = nav2_msgs::action::NavigateToPose::Goal();
            goal_pub.pose = goal_on_same_floor;
            auto goal_handle_future = action_client_->async_send_goal(goal_pub);
            RCLCPP_INFO(this->get_logger(), "Goal sent to action server.");
            x=1;
            done = false;
            is_elevator_goal = true;
            
        }
    }

    void feedbackCallback(const nav2_msgs::action::NavigateToPose_FeedbackMessage::SharedPtr feedback)
    {
        x= x+1;
        //  RCLCPP_INFO(this->get_logger(), "feedback from navigate to pose: %f", feedback->feedback.distance_remaining);
        if (feedback->feedback.distance_remaining < 0.1 && feedback->feedback.distance_remaining > 0 && x > 5 && !done && is_elevator_goal)
        {
            x = 0;
            done = true;
            RCLCPP_INFO(this->get_logger(), "Elevator goal reached!");

            // Command the elevator to go to the floor specified in the user_goal
            auto elevator_command = std::make_shared<std_msgs::msg::Int32>();
            elevator_command->data = goal_floor; // Replace user_goal_floor_ with appropriate variable
            elevator_publisher_->publish(*elevator_command);
            RCLCPP_INFO(this->get_logger(), "Sent command to elevator to go to floor: %d", goal_floor);
            
            // Reset flag for future goals
            is_elevator_goal = false;

            // Update current floor variable
            current_floor_ = goal_floor;

            take_elevator_feedback = true;
            
        }
        else if (feedback->feedback.distance_remaining < 0.1 && feedback->feedback.distance_remaining > 0 && x > 5 && !done)
        {   
            x = 0;    
            done = true;
            RCLCPP_INFO(this->get_logger(), "Goal reached!");            
        }

        
    }

    void elevatorFeedbackCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {        
        //Check if the elevator has reached the goal floor
        
        if (msg->data == goal_floor && take_elevator_feedback)
        {   

            take_elevator_feedback = false;
            is_elevator_goal = false;


            RCLCPP_INFO(this->get_logger(), "Elevator has reached the requested floor: %d", goal_floor);
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
            RCLCPP_INFO(this->get_logger(), "Sent initial goal to action server after elevator reached the floor.");
            done = false;
            x=0;
        }
    }
    
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr goal_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr elevator_publisher_;

    rclcpp::Subscription<nav2_msgs::action::NavigateToPose_FeedbackMessage>::SharedPtr feedback_sub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr goal_reached_pub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr elevator_feedback_sub_;

    int starting_floor_;
    int current_floor_;
    double elevator_goal_x_;
    double elevator_goal_y_;
    double elevator_goal_z_;
    bool goal_reached_;
    int x;
    bool done;
    bool is_elevator_goal;
    int goal_floor;
    bool take_elevator_feedback;

    double user_goal_x_;
    double user_goal_y_;
    double user_goal_z_;
    
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CustomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
