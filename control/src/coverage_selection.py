#!/usr/bin/env python
import copy
import rospy
import roslib
import sys
import auxiliary.kgstripes as kgstripes
import numpy as np

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import *

import tf_conversions
import tf2_ros
import tf2_geometry_msgs

from dynamic_reconfigure.server import Server
# from get_pose.cfg import CoverageSelectionConfig
from mallard_urdf.cfg import CoverageSelectionConfig


#BREAK MARKERS OUT INTO OTHER CLASSES?

class areaMarker(object):
    """docstring forareaMarker."""
    def __init__(self, globalFrame = "map"):
        # create an interactive marker server on the topic namespace areaSelectionMarkerServer
        self.server = InteractiveMarkerServer("areaSelectionMarkerServer")
        # Create a menu handler to use menu options on an interactive marker
        self.menu_handler = MenuHandler()

        # Create a transform broadcaster, this will be used to create a TF between the global frame, and a frame for all the markers and stripes to ride on
        self.br = tf2_ros.StaticTransformBroadcaster()
        # Publish an array of poses, on a topic called "path_poses", these messages are destined for the robot to give goal positions
        self.posePublisher = rospy.Publisher("path_poses", geometry_msgs.msg.PoseArray, queue_size = 1)

        # Set some variables, these could be made into dyanmic variables later
        self.pathGap = 0.3 # Set the spacing between stripes
        self.robotRadius = 0.3 #  Set the safety inset distance for the stripes compared to the box they are in
        self.yawAngle = 0.0 # Yaw angle relative to stripes frame

        self.globalFrame = globalFrame.strip('/')  # Take arguement from user and remove any accidental "/"
        self.markerFrame = "markerFrame" # Name of the TF frame things will be attached to
        self.moveName = "selectionMarker" # Name of the main marker for moving coverage area around
        self.rotationName = "rotationMarker" # Name of the co-located rotation marker
        initialPose = Pose() # Blank pose structure
        initialPose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
        self.tfHandler(initialPose, self.markerFrame) # Create TF message between global and marker frame based on initial marker position

        # Initialise the various markers
        self.initMainMarker(initialPose, self.globalFrame, self.moveName) #  Main movement marker - translates marker TF frame
        self.addRotation(initialPose, self.globalFrame, self.rotationName) # Rotation marker - rotates marker TF frame

        # Generate corners of the boundary box, initially with corners (-1,-1) and (1,1) in marker TF frame
        firstCornerPose = Pose()
        firstCornerPose.position.x = -1
        firstCornerPose.position.y = -1
        self.firstCornerName = "firstCorner" # 1st = "bottom-right" in TF frame
        firstCorner = self.initCornerMarker(firstCornerPose, self.markerFrame, self.firstCornerName, startingCorner=True) # Initialise marker

        secondCornerPose = Pose()
        secondCornerPose.position.x = 1.0
        secondCornerPose.position.y = 1.0
        self.secondCornerName = "secondCorner" # 2nd = "top-left" in TF frame
        secondCorner = self.initCornerMarker(secondCornerPose, self.markerFrame, self.secondCornerName, startingCorner=False) # Initialise marker

        # All the interactive markers have been initialised, draw the boundary lines, and draw the robot path as line_strip
        self.drawLines(self.updateAreaBoundary(), self.markerFrame) # Draw boundary box
        self.drawPath(self.updateAreaBoundary(), self.markerFrame) #  Draw robot path

        #Setup menu entries and add the menu options to the main interactive marker
        self.coverageOption = self.menu_handler.insert( "Confirm Coverage" , callback=self.beginCoverage)
        self.cancelOption = self.menu_handler.insert( "Cancel" , callback=self.cancelCb)
        self.menu_handler.apply(self.server, "selectionMarker")

        # Create blank pose array to contain goal positions later, with goals in the global frame
        self.poseArray = geometry_msgs.msg.PoseArray()
        self.poseArray.header.frame_id = self.globalFrame
        self.tfHandler(initialPose, self.markerFrame) # Create TF message between global and marker frame based on initial marker position

        self.startIndicator()
        dynrecon = Server(CoverageSelectionConfig, self.dynReconfigCallback)

    def initMainMarker(self, pose = Pose(), frame = "map", name = "selectionMarker"):
        # Define a pose for the marker
        markerPose = Pose()
        markerPose.position.x = pose.position.x
        markerPose.position.y = pose.position.y
        markerPose.position.z = 0.1
        markerPose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))

        # create a purple box marker
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.25
        box_marker.scale.y = 0.25
        box_marker.scale.z = 0.25
        box_marker.color.r = 0.5
        box_marker.color.g = 0.0
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0

        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame
        int_marker.name = name
        int_marker.scale = 0.01 # No ring control is visible, only the cube itself can be use to move the marker
        int_marker.description = ""
        int_marker.pose = markerPose

        # Create the controls to move only in the x-y plane
        translate_control = InteractiveMarkerControl()
        translate_control.name = "move_inPlane"
        # MOVE_PLANE controls are by default in the y-z plane, by pitch pi/2 the controls are in the x-y plane
        translate_control.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, np.pi/2, 0))
        translate_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        # add the control to the interactive marker
        int_marker.controls.append( copy.deepcopy(translate_control ))


        # create a non-interactive control which contains the box
        translate_control.markers.append( box_marker )
        translate_control.always_visible = True
        # add the control to the interactive marker
        int_marker.controls.append(translate_control);

        # Update TF frame of marker based on the pose of this initial marker
        self.tfHandler(markerPose, self.markerFrame)


        # Tell the interactive marker server to call mainProcessFeedback() when feedback arrives for it
        # i.e. when the marker is moved, perform the mainProcessFeedback()
        self.server.insert(int_marker, self.mainProcessFeedback)
        # Update server with this marker
        self.server.applyChanges()
        return int_marker

    def addRotation(self, pose = Pose(), frame = "map", name = "rotationMarker"):
        # Define a pose for the marker
        markerPose = Pose()
        markerPose.position.x = pose.position.x
        markerPose.position.y = pose.position.y
        markerPose.position.z = 0.1
        markerPose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))

        # create a grey box marker, but too small to be visualised behind the main marker
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.01
        box_marker.scale.y = 0.01
        box_marker.scale.z = 0.01
        box_marker.color.r = 0.5
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 0.0
        box_marker.frame_locked = True # Markers will always redraw themselves everytime the TF frame they are attached to updates

        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame
        int_marker.name = name
        int_marker.description = "Define Coverage Region"
        int_marker.scale = 0.6 # Ring control will be visible, but not the marker itself
        int_marker.pose = markerPose


        rotate_control = InteractiveMarkerControl()
        rotate_control.name = "rotate_inPlane"
        # ROTATE_AXIS controls are by default in the y-z plane, by pitch pi/2 the controls are in the x-y plane
        rotate_control.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, np.pi/2, 0))
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        # add the control to the interactive marker
        int_marker.controls.append( copy.deepcopy(rotate_control ))


        # create a non-interactive control which contains the box
        rotate_control.markers.append( box_marker )
        rotate_control.always_visible = True
        # add the control to the interactive marker
        int_marker.controls.append(rotate_control);

        # add the interactive marker to our collection &
        # tell the server to call rotationProcessFeedback() when feedback arrives for it
        self.server.insert(int_marker, self.rotationProcessFeedback)
        self.server.applyChanges()

    def initCornerMarker(self, pose = Pose(), frame = "markerFrame", name = "marker", startingCorner=False):
        # Define a pose for the marker
        markerPose = Pose()
        markerPose.position.x = pose.position.x
        markerPose.position.y = pose.position.y
        markerPose.position.z = 0.1
        markerPose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))

        # create a green/blue box marker
        box_marker = Marker()
        box_marker.type = Marker.CUBE
        box_marker.scale.x = 0.20
        box_marker.scale.y = 0.20
        box_marker.scale.z = 0.20
        box_marker.color.r = 0.0
        box_marker.color.g = 0.5
        box_marker.color.b = 0.5
        box_marker.color.a = 1.0
        box_marker.frame_locked = True # Markers will always redraw themselves everytime the TF frame they are attached to updates

        # create an interactive marker for our server
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = frame
        int_marker.name = name
        #if startingCorner == True:
        #    description = "Robot starts here"
        #else:
        #    description = ""
        int_marker.description = ""
        int_marker.scale = 0.1
        int_marker.pose = markerPose

        translate_control = InteractiveMarkerControl()
        translate_control.name = "move_inPlane"
        # MOVE_PLANE controls are by default in the y-z plane, by pitch pi/2 the controls are in the x-y plane
        translate_control.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, np.pi/2, 0))
        translate_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        # add the control to the interactive marker
        int_marker.controls.append( copy.deepcopy(translate_control ))


        # create a non-interactive control which contains the box
        translate_control.markers.append( box_marker )
        translate_control.always_visible = True
        # add the control to the interactive marker
        int_marker.controls.append(translate_control);

        # add the interactive marker to our collection &
        # tell the server to call subProcessFeedback() when feedback arrives for it
        self.server.insert(int_marker, self.subProcessFeedback)
        self.server.applyChanges()
        return int_marker

    def tfHandler(self, markerPose, markerFrame = "markerFrame"):
        # Generate blank Transform message
        t = geometry_msgs.msg.TransformStamped()
        # Populate with necessary information
        t.header.stamp = rospy.Time.now() # time
        t.header.frame_id = self.globalFrame # Parent frame
        t.child_frame_id = markerFrame # Child frame
        t.transform.translation = markerPose.position # Use marker position in global frame as translation
        t.transform.rotation = markerPose.orientation # Use marker orientation in global frame as rotation

        # Use static transform broadcaster to update TF frame
        self.br.sendTransform(t)


    def mainProcessFeedback(self, feedback):
        p = feedback.pose.position
        #FEEDBACK USES NAME OR MARKER_NAME DEPENDING ON SOURCE (DIRECT OR serverGET)
        #print feedback.marker_name + " is now at " + str(p.x) + ", " + str(p.y) + ", " + str(p.z)

        # Set pose of rotation marker, so it is the same as main marker
        self.server.setPose(name=self.rotationName, pose=feedback.pose)
        # Update TF frame based on new position of main marker
        self.tfHandler(feedback.pose, self.markerFrame)
        # Apply any changes to the interactive marker server (e.g. rotation marker new pose)
        self.server.applyChanges()

    def rotationProcessFeedback(self, feedback):
        r = feedback.pose
        # Update the orientation of the main marker based on orientation of the rotation marker
        self.server.setPose(name=self.moveName, pose=r)
        # Invoke the main marker feedback to ensure the marker TF frame is updated correctly
        self.mainProcessFeedback(self.server.get(self.moveName))
        # Make updates on the marker server
        self.server.applyChanges()
        #boundary = self.updateAreaBoundary()


    def subProcessFeedback(self, feedback):
        p = feedback.pose.position
        # To ensure the corners move in equal and opposite directions to eachother, when one is updated, the other has it's pose updated
        # If firstCorner moves(update second corner position to be equal and opposite)
        if  feedback.marker_name == self.firstCornerName:
            anti_position = [-1*float(feedback.pose.position.x), -1*float(feedback.pose.position.y), -1*float(feedback.pose.position.z)]
            anti_pose = Pose()
            anti_pose.position = Point(*anti_position)
            self.server.setPose(name=self.secondCornerName, pose=anti_pose) #("OrientationLines", feedback.pose , Header())
            self.server.applyChanges()
        # If secondCorner moves(update first corner position to be equal and opposite)
        if  feedback.marker_name == self.secondCornerName:
            anti_position = [-1*float(feedback.pose.position.x), -1*float(feedback.pose.position.y), -1*float(feedback.pose.position.z)]
            anti_pose = Pose()
            anti_pose.position = Point(*anti_position)
            self.server.setPose(name=self.firstCornerName, pose=anti_pose)
            self.server.applyChanges()

        # Calculate boundary in marker TF frame based on new corner positions
        boundary = self.updateAreaBoundary()
        # Update the boundary box, and the robot path lines based on new boundary
        self.drawLines(boundary, self.markerFrame)
        self.drawPath(boundary, self.markerFrame)
        self.startIndicator()


    def updateAreaBoundary(self):
        # Gets poses of the corner markers, and returns bounds for x0, y0, x1, y1
        firstCorner = self.server.get(self.firstCornerName)
        secondCorner = self.server.get(self.secondCornerName)

        xValues = [firstCorner.pose.position.x, secondCorner.pose.position.x] # x positions
        yValues = [firstCorner.pose.position.y, secondCorner.pose.position.y] # y positions
        zValues = [firstCorner.pose.position.z, secondCorner.pose.position.z] # z not considered here

        boundary = [min(xValues), min(yValues), max(xValues), max(yValues)] # Order the values
        return boundary

    def drawLines(self, boundary, markerFrame):
        # Lines will be updateable (size, location), but will not be interactive themselves
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = markerFrame
        int_marker.name = "boundaryLines"
        int_marker.description = ""
        int_marker.pose = Pose()
        int_marker.pose.orientation.w = 1.0

        line_marker = Marker()
        line_marker.type = Marker.LINE_STRIP
        line_marker.scale.x = 0.1
        line_marker.scale.y = 0.1
        line_marker.scale.z = 0.1
        line_marker.color.r = 0.0
        line_marker.color.g = 0.5
        line_marker.color.b = 0.5
        line_marker.color.a = 1.0
        line_marker.frame_locked = True

        zvalue = 0.0
        line_marker.points.append(Point(*[boundary[0], boundary[1], zvalue]))
        line_marker.points.append(Point(*[boundary[0], boundary[3], zvalue]))
        line_marker.points.append(Point(*[boundary[2], boundary[3], zvalue]))
        line_marker.points.append(Point(*[boundary[2], boundary[1], zvalue]))
        line_marker.points.append(Point(*[boundary[0], boundary[1], zvalue]))

        line_control = InteractiveMarkerControl()
        line_control.always_visible = True
        line_control.markers.append( line_marker )

        # add the control to the interactive marker
        int_marker.controls.append( line_control )

        # As the marker is only for show, it has NONE interactive control
        null_control = InteractiveMarkerControl()
        null_control.name = "none"
        null_control.orientation_mode = InteractiveMarkerControl.FIXED
        null_control.interaction_mode = InteractiveMarkerControl.NONE

        # add the control to the interactive marker
        int_marker.controls.append(null_control);
        #self.server.insert(int_marker, lineFeedback)
        self.server.insert(int_marker)
        self.server.applyChanges()

    def drawPath(self, boundary, markerFrame):
        # Lines will be updateable (size, location), but will not be interactive themselves
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = markerFrame
        int_marker.name = "pathLines"
        int_marker.description = ""
        int_marker.pose = Pose()
        int_marker.pose.orientation.w = 1.0

        line_marker = Marker()
        line_marker.type = Marker.LINE_STRIP
        line_marker.scale.x = 0.10
        line_marker.scale.y = 0.10
        line_marker.scale.z = 0.10
        line_marker.color.r = 0.5
        line_marker.color.g = 0.5
        line_marker.color.b = 0.0
        line_marker.color.a = 0.5
        line_marker.frame_locked = True

        #Stripes algorithm takes arguements: orientation, line gap, x1, x2, y1, y2, yaw offset of robot)
        #To ensure the robot is "inside" the coverage area, the boundary is inset by the robot radius
        insetBoundary = [0,0,0,0]
        for cidx, corners in enumerate(boundary):
            if abs(corners) >= self.robotRadius:
                if round(cidx/2) == 1.0:
                    factor = -1
                else:
                    factor = 1

                insetBoundary[cidx] = boundary[cidx] + factor * self.robotRadius

        x1 = insetBoundary[0]
        x2 = insetBoundary[2]
        y1 = insetBoundary[1]
        y2 = insetBoundary[3]

        self.path = kgstripes.stripes(0, self.pathGap, x1, x2, y1, y2, 0)
        for position in self.path:
            line_marker.points.append(Point(*position))

        line_control = InteractiveMarkerControl()
        line_control.always_visible = True
        line_control.markers.append( line_marker )

        # add the control to the interactive marker
        int_marker.controls.append( line_control )

        # As the marker is only for show, it has NONE interactive control
        null_control = InteractiveMarkerControl()
        null_control.name = "none"
        null_control.interaction_mode = InteractiveMarkerControl.NONE
        # Keep the lines orientated in the orientation of the main marker (i.e. int_marker pose, there for in the x-y plane, despite the user moving the scene in rviz)
        null_control.orientation_mode = InteractiveMarkerControl.FIXED

        # add the control to the interactive marker
        int_marker.controls.append(null_control);
        #self.server.insert(int_marker, lineFeedback)
        self.server.insert(int_marker)
        self.server.applyChanges()

    def startIndicator(self):
        # Lines will be updateable (size, location), but will not be interactive themselves
        int_marker = InteractiveMarker()
        int_marker.header.frame_id = self.markerFrame
        int_marker.name = "startIndicator"
        int_marker.description = "Robot Will Start Here\nOriented in Direction of Arrow"
        int_marker.pose.position = self.server.get("pathLines").controls[0].markers[0].points[0]
        int_marker.pose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, self.yawAngle))

        line_marker = Marker()
        line_marker.type = Marker.ARROW
        line_marker.scale.x = 0.2
        line_marker.scale.y = 0.05
        line_marker.scale.z = 0.05
        line_marker.color.r = 0.5
        line_marker.color.g = 0.0
        line_marker.color.b = 0.5
        line_marker.color.a = 1.0
        #line_marker.frame_locked = True


        line_control = InteractiveMarkerControl()
        line_control.description = "Robot Will Start Here\nOriented in Direction of Arrow"
        int_marker.scale = 0.3
        line_control.always_visible = True
        line_control.markers.append( line_marker )

        # add the control to the interactive marker
        int_marker.controls.append( line_control )

        # As the marker is only for show, it has NONE interactive control
        null_control = InteractiveMarkerControl()
        null_control.name = "none"
        null_control.interaction_mode = InteractiveMarkerControl.NONE

        # add the control to the interactive marker
        int_marker.controls.append(null_control);
        #self.server.insert(int_marker, lineFeedback)
        self.server.insert(int_marker)
        self.server.applyChanges()


    def beginCoverage(self, feedback):
        print "Sending goal positions to path planner"
        # Append each goal position, based on the striping algorithm to a posearray message
        mainMarker = self.server.get(self.moveName)
        frameTransform = TransformStamped()
        frameTransform.header.stamp = rospy.Time.now()
        frameTransform.header.frame_id = self.globalFrame
        frameTransform.child_frame_id = self.markerFrame
        frameTransform.transform.translation = mainMarker.pose.position
        frameTransform.transform.rotation = mainMarker.pose.orientation

        #print frameTransform
        pathPose = PoseStamped()
        pathPose.header = frameTransform.header
        pathPose.pose.orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, self.yawAngle))

        for position in self.path:
            pathPose.pose.position.x = position[0]
            pathPose.pose.position.y = position[1]
            pathPose.pose.position.z = position[2]

            pose = tf2_geometry_msgs.do_transform_pose(pathPose, frameTransform)

            self.poseArray.poses.append(pose.pose)
        self.poseArray.header.stamp = rospy.Time.now() #  Set time stamp to NOW
        self.poseArray.header.seq = self.poseArray.header.seq + 1 # Increment the sequence counter
        self.posePublisher.publish(self.poseArray) # send pose array
        self.poseArray.poses = [] # clear pose array (avoids old data being send multiple times)

    def cancelCb(self, feedback):
        # Send blank pose array, which will halt the path planner (with not having any goals)
        self.poseArray.header.stamp = rospy.Time.now() #  Set time stamp to NOW
        self.poseArray.header.seq = self.poseArray.header.seq + 1 # Increment the sequence counter
        self.poseArray.poses = [] # clear pose array (avoids old data being send multiple times)
        self.posePublisher.publish(self.poseArray) # send pose array

    def dynReconfigCallback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {robot_radius}, {stripe_gap}, {yaw_angle}""".format(**config))
        self.pathGap = config.stripe_gap # Set the spacing between stripes
        self.robotRadius = config.robot_radius #  Set the safety inset distance for the stripes compared to the box they are in
        self.yawAngle = config.yaw_angle
        self.drawLines(self.updateAreaBoundary(), self.markerFrame)
        self.drawPath(self.updateAreaBoundary(), self.markerFrame)
        self.startIndicator()
        return config


if __name__=="__main__":
    # Initialise ROS node
    rospy.init_node("areaSelectionMarkers")

    #args = rospy.myargv(argv=sys.argv)

    # If user has passed an argument, pass that to the main class
    #if args >= 2:
    #    s = areaMarker(sys.argv[1])
    #else: #  Else use default values
    #    s = areaMarker()
    s = areaMarker()
    # Continue forever with rospy.spin()
    rospy.spin()
