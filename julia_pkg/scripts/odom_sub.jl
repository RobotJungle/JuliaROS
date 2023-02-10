#!/usr/bin/env julia167

import Pkg; Pkg.activate(joinpath(@__DIR__,"../../../..")); Pkg.instantiate()
using RobotOS
@rosimport nav_msgs.msg.Odometry
rostypegen()
using .nav_msgs.msg: Odometry

function tb0_callback(msg)
    #loginfo("$(RobotOS.get_caller_id()) I heard $(msg.data)")
    #println(msg.pose.pose.position)
    xpos = msg.pose.pose.position.x
    ypos = msg.pose.pose.position.y

    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w

    theta = atan(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    # println("theta 0: ",theta)
    #botsSt0 .= [xpos; ypos; theta]
end

function tb1_callback(msg)
    xpos = msg.pose.pose.position.x
    ypos = msg.pose.pose.position.y

    qx = msg.pose.pose.orientation.x
    qy = msg.pose.pose.orientation.y
    qz = msg.pose.pose.orientation.z
    qw = msg.pose.pose.orientation.w

    theta = atan(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))

    println("theta 1: ",theta)
    # botsSt1 .= [xpos; ypos; theta]
end


function main()
    init_node("turtlebot3_odom") # node name

    tbb1_odom = Subscriber{Odometry}("/tb3_0/odom", tb0_callback; queue_size=10)
    tbb2_odom = Subscriber{Odometry}("/tb3_1/odom", tb1_callback; queue_size=10)

    spin()
    #return X
end

if !isinteractive()
    main()
end
