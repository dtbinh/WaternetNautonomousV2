#!/usr/bin/env python

import requests, time, rospy
import netifaces as ni
from std_msgs.msg import String

isConnected = False
ipAddr = ""
nautName = "Nautonomous-1"
disconnectPub = rospy.Publisher("/nautonomous/webserver/disconnect", String, queue_size=1)

def getOwnIP():
    # gets all available interfaces
    interfaces = ni.interfaces()
    tun = ""

    # loop through interfaces to find the used tun iface
    for i in interfaces:
        if 'tun' in i:
            addr = ni.ifaddresses(i)[2][0]['addr']
            if '10.8.0.' in addr:
                return addr

    return False    

def postOnlineStatusToServer(ip, name):
    global isConnected
    try:
        while not isConnected:
            r = requests.post('https://nautonomous.nl/online_status', data = {'name': name, 'ip': ip}) #, verify=False)
            if str(r.status_code) == '200':
                isConnected = True
                rospy.loginfo("Post message sent, status for this nautonomous is now 'online'.")
            else:
                rospy.logerr("Could not send post message, retrying in 5 sec")
                time.sleep(5)           
    except Exception as e:
        print("The error is: %s", e.message)

def postOfflineStatusToServer(name):
    global isConnected
    try:
        while isConnected:
            r = requests.post('https://nautonomous.nl/offline_status', data = {'name': name}) #, verify=False)
            if str(r.status_code) == '200':
                isConnected = False
                rospy.loginfo("Post message sent, status for this nautonomous is now 'offline'.")
            else:
                rospy.logerr("Could not send post message, retrying in 5 sec")
                time.sleep(5)           
    except Exception as e:
        print("The error is: %s", e.message)

def connStatusCb(msg):
    global ipAddr, nautName

    if (msg.data == "reconnection" or msg.data == "online") and not isConnected:
        postOnlineStatusToServer(ipAddr, nautName)
    elif msg.data == "offline" and isConnected:
        rospy.loginfo("Disconnecting rosbridge connection")
        disconnectPub.publish(nautName)
        postOfflineStatusToServer(nautName)

def finalPost():
    global nautName

    postOfflineStatusToServer(nautName)

    rospy.loginfo("Shutting down connection handler node.")

def main():
    global ipAddr, nautName

    ipAddr = getOwnIP()

    rospy.init_node("connection_handler", anonymous=True)
    rospy.on_shutdown(finalPost)
    rospy.Subscriber("/nautonomous/webserver/connection", String, connStatusCb)
 
    if ipAddr:
        # name should become a variable as well, instead of static text.
        postOnlineStatusToServer(ipAddr, nautName)
        rospy.spin()
    else:
        #might need logic that handles (re)connecting to VPN
        print("No VPN IP found. Check VPN connection")
       

    return 0

if __name__ == '__main__':
    main()
