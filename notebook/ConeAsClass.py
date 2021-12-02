
from interactive_markers.interactive_marker_server import InteractiveMarker
import numpy as np
from markers import createPose, cone, sphere
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose
from tf import transformations as tf

from visualization_msgs.msg import Marker, InteractiveMarker, InteractiveMarkerControl

def poseToTf(pose: Pose)->np.matrix:
    p = pose.position
    q = pose.orientation
    return tf.translation_matrix([p.x, p.y, p.z]).dot(tf.quaternion_matrix([q.x, q.y, q.z, q.w]))


class ConeMarker(InteractiveMarker):

    def __init__(self, server, pose, initAngle=.2, scale=.3, markers=[],*args, **kwds):
        super().__init__(*args, **kwds)

        self.im_server = server
        self.coneMarker = cone(initAngle, scale=scale,  color=ColorRGBA(1, .2, 1, .3))
        handle = sphere(radius=scale*.1, color=ColorRGBA(0, 1, 1, 1))

        self.im_cone = InteractiveMarker()
        self.im_cone.name = "cone"
        self.im_cone.header.frame_id = "world"
        self.im_cone.description = ""
        self.im_cone.scale = 0.2
        self.im_cone.pose = createPose(pose)

        self.imc_cone = InteractiveMarkerControl()
        self.imc_cone.always_visible=True
        self.imc_cone.interaction_mode = InteractiveMarkerControl.MOVE_ROTATE_3D
        self.imc_cone.markers = [self.coneMarker,*markers]
        self.im_cone.controls = [self.imc_cone]

        self.im_handle = InteractiveMarker()
        self.im_handle.name = "handle"
        self.im_handle.header.frame_id = "world"
        self.im_handle.scale = 0.2

        radius = scale * np.sin(initAngle)
        self.im_handle.pose = createPose(pose.dot(  tf.translation_matrix( [radius, 0, scale])  ) )


        self.imc_handle = InteractiveMarkerControl()
        self.imc_handle.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        self.imc_handle.markers = [handle]
        self.im_handle.controls = [self.imc_handle]

        self.im_server.insert(self.im_cone,self.callback)
        self.im_server.insert(self.im_handle,self.callback)
        self.im_server.applyChanges()

        self._pose_callback = []
        self._angle_callback=[]


    def addPoseCallback(self, cb):
        self._pose_callback.append(cb)

    def addAngleCallback(self, cb):
        self._angle_callback.append(cb)

    def callback(self, x):
        if x.marker_name == 'cone':
            T = poseToTf(x.pose)
            Th = poseToTf(self.im_handle.pose)
            Tc = poseToTf(self.im_cone.pose)
            deltaHandel = np.linalg.inv(Tc).dot(Th)
            Th_new = T.dot(deltaHandel)

            p = createPose(Th_new)
            self.im_server.setPose("handle", p)

            for cb in self._pose_callback:
                cb(T)
        else:
            T = poseToTf(x.pose)
            Tc = poseToTf(self.im_cone.pose)
            deltaHandel = np.linalg.inv(Tc).dot(T)
            radius =  tf.translation_from_matrix(deltaHandel)[0]
            scale =  tf.translation_from_matrix(deltaHandel)[2]

            newAngle = np.arcsin(radius)/scale
            self.imc_cone.markers.remove(self.coneMarker)
            self.coneMarker = cone(newAngle, scale=scale,  color=ColorRGBA(1, .2, 1, .3))

            self.imc_cone.markers.append(self.coneMarker)
            self.im_cone.description = f"{ np.rad2deg(newAngle):3.1f}"
            self.im_handle.controls = [self.imc_handle]
            self.im_server.insert(self.im_cone,self.callback)

            for cb in self._angle_callback:
                cb(newAngle)

        self.im_server.applyChanges()

