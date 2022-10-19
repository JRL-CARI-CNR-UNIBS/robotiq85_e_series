#!/usr/bin/env python3

import rospy
import signal
import sys

import ur_dashboard_msgs.srv
import std_srvs.srv
import std_msgs.msg
import configuration_msgs.srv
import manipulation_msgs.srv


from pathlib import Path

cont = True
target_position = 255
target_position_mm = 255
force_setpoint = 1
velocity_setpoint = 10
new_command = False
force_percentage=0
vel_percentage=0

def handler(signal, frame):
    global cont
    cont = False


def job_exec_srv(req):
    global target_position_mm
    global force_percentage
    global force_setpoint
    global vel_percentage
    global new_command
    global execution_finished

    print("req.property_id: {}".format(req.property_id))
    string=req.property_id
    print(string)
    if string.startswith('pos_'):
        string=req.property_id
        idx_force=string.find('_force_')
        target_position_mm = float(string[4:idx_force])

        idx_vel=string.find('_vel_')
        force_percentage = float(string[idx_force+7:idx_vel])

        vel_percentage = float(string[idx_vel+5:])

        new_command = True
    elif req.property_id == "close":
        print("Close")
        new_command = True
        target_position_mm = 0.0
        force_percentage = 10.0
        vel_percentage = 10.0
    elif req.property_id == "open":
        print("Open");
        new_command = True
        target_position_mm = 40.0
        force_percentage = 10.0
        vel_percentage = 10.0
    else:
        print("ELSE-open 50 mm");
        new_command = True
        target_position_mm = 50
        velocity_setpoint = 1
        force_setpoint = 1
        vel_percentage = 10.0
    r = rospy.Rate(10) # 10hz
    print("wait for execution. Target pose (mm) = ",target_position_mm)
    while (new_command):
        r.sleep()
    print("executed")

    return manipulation_msgs.srv.JobExecutionResponse(0)


def test_robotiq(serviceName):
    global target_position
    global new_command

    rospy.init_node(serviceName)

    pub = rospy.Publisher('/ur10e_hw/script_command', std_msgs.msg.String, queue_size=10)
    s = rospy.Service(serviceName, manipulation_msgs.srv.JobExecution, job_exec_srv)
    r = rospy.Rate(10) # 10hz

    template = Path('template.script').read_text()



    while cont:
        if (new_command):

            command=template
            target_position =  int(255 -  target_position_mm/85.0 * (255.0))
            force_setpoint=int(force_percentage/100.0*255.0)
            velocity_setpoint=int(vel_percentage/100.0*255.0)
            command=command.replace("POSITION_BYTE",str(target_position))
            command=command.replace("SPEED_BYTE",str(force_setpoint))
            command=command.replace("FORCE_BYTE",str(velocity_setpoint))
            pub.publish(command)

            new_command = False
        r.sleep()

    new_command=False
    if (not debug_flag):
        hand.disconnect()

if __name__ == '__main__':
    signal.signal(signal.SIGINT, handler)
    if len(sys.argv) < 1:
        print("The script need to take at least one input argument.")
    else:
        serviceName = sys.argv[1]
        test_robotiq(serviceName)
        rospy.spin()
