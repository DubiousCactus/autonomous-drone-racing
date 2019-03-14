#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2019 theomorales <theomorales@Theos-MacBook-Pro.local>
#
# Distributed under terms of the MIT license.

"""
SceneGenerator

Generates an image by projecting a 3D mesh over a 2D transparent background.
"""


import numpy as np
import quaternion
import moderngl
import random
import yaml

from math import cos, sin, atan2, fabs, copysign, asin, degrees, radians
from pyrr import Matrix44, Quaternion, Vector3, Vector4
from moderngl.ext.obj import Obj
from PIL import Image


class SceneGenerator:
    def __init__(self, mesh_path: str, width: int, height: int,
                 world_boundaries, gate_center: Vector3, camera_parameters,
                 drone_pose, render_perspective=False):
        random.seed(2)
        self.render_perspective = render_perspective
        self.mesh = Obj.open(mesh_path)
        self.width = width
        self.height = height
        self.gate_center = gate_center
        self.drone_pose = drone_pose
        self.boundaries = self.compute_boundaries(world_boundaries)
        with open(camera_parameters, 'r') as cam_file:
            try:
                self.camera_parameters = yaml.safe_load(cam_file)
            except yaml.YAMLError as exc:
                raise Exception(exc)
        self.setup_opengl()

    def add_bias(self, roll, pitch):
        self.pitch_bias = pitch
        self.roll_bias = roll

    # TODO: Compute from the origin
    def compute_boundaries(self, world_boundaries):
        # Set the orthographic coordinates for the boundaries, based on the size of the
        # mesh
        mesh_width = abs(
            min(self.mesh.vert, key = lambda v_pair: v_pair[0])[0]
            - max(self.mesh.vert, key = lambda v_pair: v_pair[0])[0]
        )
        mesh_height = abs(
            min(self.mesh.vert, key = lambda v_pair: v_pair[2])[2]
            - max(self.mesh.vert, key = lambda v_pair: v_pair[2])[2]
        )

        # print("Mesh width: {}\nMesh height: {}".format(mesh_width, mesh_height))

#         return  {
            # 'x': mesh_width * world_boundaries['x'],
            # 'y': mesh_width * world_boundaries['y'],
            # 'z': mesh_height * world_boundaries['z']
#         }

        return  {
            'x': world_boundaries['x'] / 2,
            'y': world_boundaries['y'] / 2,
            'z': world_boundaries['z'] / 2
        }

    def setup_opengl(self):
        vertex_shader_source = open('data/shader.vert').read()
        fragment_shader_source = open('data/shader.frag').read()
        # Context creation
        self.context = moderngl.create_standalone_context()
        # Shaders
        self.prog = self.context.program(vertex_shader=vertex_shader_source, fragment_shader=fragment_shader_source)
        self.grid_prog = self.context.program(vertex_shader=vertex_shader_source, fragment_shader=fragment_shader_source)

    def quaternion_to_euler_angle(self, q: Quaternion):
        roll = atan2( # X-axis rotation
            2.0 * (q.w * q.x + q.y * q.z),
            1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        # Y-axis rotation
        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        if fabs(sinp) >= 1:
            pitch = copysign(np.pi / 2, sinp)
        else:
            pitch = asin(sinp)
        yaw = atan2(# Z-axis rotation
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        return (roll, pitch, yaw)

    def generate(self):
        ''' Randomly move the gate around, while keeping it inside the boundaries '''
        translation = Vector3([
            random.uniform(-self.boundaries['x'], self.boundaries['x']),
            random.uniform(-self.boundaries['y'], self.boundaries['y']),
            0
        ])

        ''' Randomly rotate the gate horizontally, around the Z-axis '''
        rotation = Quaternion.from_z_rotation(random.random() * np.pi)

        model = Matrix44.from_translation(translation) * rotation
        gate_center = model * self.gate_center

        camera_intrinsics = [
            self.camera_parameters['camera_matrix']['data'][0:3],
            self.camera_parameters['camera_matrix']['data'][3:6],
            self.camera_parameters['camera_matrix']['data'][6::]
        ]
        fx, fy = camera_intrinsics[0][0], camera_intrinsics[1][1]
        cx, cy = camera_intrinsics[0][2], camera_intrinsics[1][2]
        x0, y0 = 0, 0 # Camera image origin
        zfar, znear = 100.0, 0.01 # distances to the clipping plane
        # Works by following: https://blog.noctua-software.com/opencv-opengl-projection-matrix.html
        # Doesn't work by following: http://kgeorge.github.io/2014/03/08/calculating-opengl-perspective-matrix-from-opencv-intrinsic-matrix
        projection = Matrix44([
            [fx/cx, 0, 0, 0],
            [0, fy/cy, 0, 0],
            [0, 0, (-zfar - znear)/(zfar - znear), -1],
            [0, 0, (-2.0*zfar*znear)/(zfar - znear), 0] # TODO: EXPLAIN WHY IT WORKS WHEN I FLIP [2][2] AND [3][2]
        ])

        # TODO: Find a way to add bias to the quaternion directly
        # euler_angles_deg = [degrees(x) for x in
                            # quaternion.as_euler_angles(np.quaternion(*self.drone_pose.orientation.xyzw))]
        # model_orientation = Quaternion(
            # quaternion.as_float_array(
                # quaternion.from_euler_angles([radians(x) for x in euler_angles_deg])
            # )
        # ) 

        '''
        It seems that heavy rotations on the model matrix make it appear flipped from
        the camera's perspective. Try building your own view matrix to see if you can
        apply the correct rotations to the camera only.
        '''
        # Camera view matrix
        view = Matrix44.look_at(
            # eye: position of the camera in world coordinates
            self.drone_pose.translation,
            # target: position in world coordinates that the camera is looking at
            self.drone_pose.translation + Vector3([100.0, 0.0, 0.0]),
            # up: up vector of the camera.
            (0.0, 0.0, 1.0), # We are using the Z-axis to match our base dataset
        )
        # Model View Projection matrix
        mvp = projection * view * self.drone_pose.orientation * model

        # Converting the gate center's world coordinates to image coordinates
        clip_space_gate_center = projection * (view * self.drone_pose.orientation *  Vector4.from_vector3(gate_center, w=1.0))

        if clip_space_gate_center.w != 0:
            normalized_device_coordinate_space_gate_center\
                = Vector3(clip_space_gate_center.xyz) / clip_space_gate_center.w
        else: # Clipped
            normalized_device_coordinate_space_gate_center = clip_space_gate_center.xyz

        viewOffset = 0 # TODO: verify that
        image_frame_gate_center =\
            ((np.array(normalized_device_coordinate_space_gate_center.xy) + 1.0) /
             2.0) * np.array([self.width, self.height]) + viewOffset

        # Translate from bottom-left to top-left
        image_frame_gate_center[1] = self.height - image_frame_gate_center[1]

        # Shader program
        self.prog['Light'].value = (0.0, 0.0, 6.0) # TODO
        self.prog['Color'].value = (1.0, 1.0, 1.0, 0.25) # TODO
        self.prog['Mvp'].write(mvp.astype('f4').tobytes())

        # Texturing
        texture_image = Image.open('data/shiny-white-metal-texture.jpg')
        texture = self.context.texture(texture_image.size, 3, texture_image.tobytes())
        texture.build_mipmaps()

        # Vertex Buffer and Vertex Array
        vbo = self.context.buffer(self.mesh.pack())
        vao = self.context.simple_vertex_array(self.prog, vbo, *['in_vert', 'in_text', 'in_norm'])

        # Project the perspective as a grid
        if self.render_perspective:
            grid = []
            for i in range(65):
                grid.append([i - 32, -32.0, 0.0, i - 32, 32.0, 0.0])
                grid.append([-32.0, i - 32, 0.0, 32.0, i - 32, 0.0])

            grid = np.array(grid)

            vp = projection * view * self.drone_pose.orientation
            self.grid_prog['Light'].value = (0.0, 0.0, 6.0) # TODO
            self.grid_prog['Color'].value = (1.0, 1.0, 1.0, 0.25) # TODO
            self.grid_prog['Mvp'].write(vp.astype('f4').tobytes())

            vbo_grid = self.context.buffer(grid.astype('f4').tobytes())
            vao_grid = self.context.simple_vertex_array(self.grid_prog, vbo_grid, 'in_vert')


        # Framebuffers
        # Use 4 samples for MSAA anti-aliasing
        fbo1 = self.context.framebuffer(
            self.context.renderbuffer((self.width, self.height), samples=8),
            depth_attachment=self.context.depth_renderbuffer(
                (self.width, self.height), samples=8
            )
        )

        # Downsample to the final framebuffer
        fbo2 = self.context.framebuffer(self.context.renderbuffer((self.width,
                                                                   self.height)))

        # Rendering
        fbo1.use()
        self.context.enable(moderngl.DEPTH_TEST)
        self.context.clear(0.9, 0.9, 0.9)
        texture.use()
        vao.render()

        if self.render_perspective:
            vao_grid.render(moderngl.LINES, 65 * 4)

        self.context.copy_framebuffer(fbo2, fbo1)

        # Loading the image using Pillow
        img = Image.frombytes('RGBA', fbo2.size, fbo2.read(components=4,
                                                         alignment=1), 'raw', 'RGBA', 0, -1)

        '''
            Apply distortion using the camera parameters
        '''
        # TODO

        annotations = {
            'gate_center_img_frame': image_frame_gate_center,
            'gate_rotation':
                [degrees(x) for x in
                 quaternion.as_euler_angles(np.quaternion(*rotation.xyzw))],
            'drone_pose': self.drone_pose.translation,
            'drone_orientation':
                [degrees(x) for x in
                 quaternion.as_euler_angles(np.quaternion(*self.drone_pose.orientation.xyzw))]
        }

        return (img, annotations)
