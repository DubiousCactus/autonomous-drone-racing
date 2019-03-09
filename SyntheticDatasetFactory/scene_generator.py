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
import moderngl
import random
import yaml

from pyrr import Matrix44, Quaternion, Vector3, Vector4
from math import cos, sin, atan2
from ModernGL.ext.obj import Obj
from PIL import Image


class SceneGenerator:
    def __init__(self, mesh_path: str, width: int, height: int,
                 world_boundaries, gate_center: Vector3, camera_parameters,
                 drone_pose):
        random.seed()
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
                print(exc)
        self.setup_opengl()

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

        return  {
            'x': mesh_width * world_boundaries['x'],
            'y': mesh_width * world_boundaries['y'],
            'z': mesh_height * world_boundaries['z']
        }

    def setup_opengl(self):
        vertex_shader_source = open('data/shader.vert').read()
        fragment_shader_source = open('data/shader.frag').read()
        # Context creation
        self.context = moderngl.create_standalone_context()
        # Shaders
        self.prog = self.context.program(vertex_shader=vertex_shader_source, fragment_shader=fragment_shader_source)
        self.grid_prog = self.context.program(vertex_shader=vertex_shader_source, fragment_shader=fragment_shader_source)

    def generate(self):
        ''' Randomly move the gate around, while keeping it inside the boundaries '''
        translation = Vector3([
            random.uniform(-self.boundaries['x'], self.boundaries['x']),
            random.uniform(-self.boundaries['y'], self.boundaries['y']),
            0
        ])

        # print("Randomized translation: {}".format(translation))

        ''' Randomly rotate the gate horizontally, around the Z-axis '''
        rotation = Quaternion.from_z_rotation(random.random() * np.pi)
        # print("Randomized rotation: {}".format(rotation))

        scale = Vector3([1., 1., 1.]) # Scale it by a factor of 1
        model = Matrix44.from_translation(translation) * rotation * Matrix44.from_scale(scale)
        gate_center = model * self.gate_center

        camera_intrinsics = [
            self.camera_parameters['camera_matrix']['data'][0:3],
            self.camera_parameters['camera_matrix']['data'][3:6],
            self.camera_parameters['camera_matrix']['data'][6::]
        ]
        fx, fy = camera_intrinsics[0][0], camera_intrinsics[1][1]
        cx, cy = camera_intrinsics[0][2], camera_intrinsics[1][2]
        x0, y0 = 0, 0 # Camera image origin
        zfar, znear = 100.0, 0.1 # distances to the clipping plane
        # Works by following: https://blog.noctua-software.com/opencv-opengl-projection-matrix.html
        # Doesn't work by following: http://kgeorge.github.io/2014/03/08/calculating-opengl-perspective-matrix-from-opencv-intrinsic-matrix
        projection = Matrix44([
            [fx/cx, 0, 0, 0],
            [0, fy/cy, 0, 0],
            [0, 0, (-zfar - znear)/(zfar - znear), -1],
            [0, 0, (-2.0*zfar*znear)/(zfar - znear), 0] # TODO: EXPLAIN WHY IT WORKS WHEN I FLIP [2][2] AND [3][2]
        ])

        # Camera view matrix
        '''
         x: horizontal axis
         y: depth axis
         z: vertical axis
        '''
        print("Drone translation: {}".format(self.drone_pose.translation))
        print("Drone orientation: {}".format(self.drone_pose.orientation))
        rollRad = atan2( # X-axis rotation
            2.0 * (self.drone_pose.orientation.w *
                   self.drone_pose.orientation.x +
                   self.drone_pose.orientation.y *
                   self.drone_pose.orientation.z),
            1.0 - 2.0 * (self.drone_pose.orientation.x *
                         self.drone_pose.orientation.x +
                         self.drone_pose.orientation.y *
                         self.drone_pose.orientation.y)
        )
        yawRad = atan2(# Z-axis rotation
            2.0 * (self.drone_pose.orientation.w *
                   self.drone_pose.orientation.z +
                   self.drone_pose.orientation.x *
                   self.drone_pose.orientation.y),
            1.0 - 2.0 * (self.drone_pose.orientation.y *
                         self.drone_pose.orientation.y +
                         self.drone_pose.orientation.z *
                         self.drone_pose.orientation.z)
        )
        ray = 1.0
        directionX = ray * (cos(rollRad) * sin(yawRad))
        directionY = ray * (cos(rollRad) * cos(yawRad))
        directionZ = ray * sin(rollRad)
        view = Matrix44.look_at(
            # eye: position of the camera in world coordinates
            self.drone_pose.translation,
            # target: position in world coordinates that the camera is looking at
#             (
                # self.drone_pose.orientation.z * self.drone_pose.translation.x,
                # self.drone_pose.orientation.y * self.drone_pose.translation.y,
                # self.drone_pose.orientation.x * self.drone_pose.translation.z
            # ),
            # self.drone_pose.orientation * self.drone_pose.translation,
            (
                self.drone_pose.translation.x + directionX,
                self.drone_pose.translation.y + directionY,
                self.drone_pose.translation.z + directionZ
            ),
            # up: up vector of the camera. ModernGL seems to invert the y- and z- axis compared to the OpenGL doc !
            (0.0, 0.0, 1.0),
        )
        # Model View Projection matrix
        mvp = projection * view * model
        # Don't transform the perspective grid
        no_translation_mvp = projection * view * Matrix44.identity()

        # Converting the gate center's world coordinates to image coordinates
        clip_space_gate_center = projection * (view * Vector4.from_vector3(gate_center, w=1.0))

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
        self.prog['Light'].value = (0.0, 10.0, 0.0) # TODO
        self.prog['Color'].value = (1.0, 1.0, 1.0, 0.25) # TODO
        self.prog['Mvp'].write(mvp.astype('f4').tobytes())

        self.grid_prog['Light'].value = (0.0, 10.0, 0.0)
        self.grid_prog['Color'].value = (1.0, 1.0, 1.0, 0.25)
        self.grid_prog['Mvp'].write(no_translation_mvp.astype('f4').tobytes())

        # Texturing
        texture_image = Image.open('data/shiny-white-metal-texture.jpg')
        texture = self.context.texture(texture_image.size, 3, texture_image.tobytes())
        texture.build_mipmaps()

        # Project the perspective as a grid
        grid = []
        for i in range(65):
            grid.append([i - 32, -32.0, 0.0, i - 32, 32.0, 0.0])
            grid.append([-32.0, i - 32, 0.0, 32.0, i - 32, 0.0])

        grid = np.array(grid)

        # Vertex Buffer and Vertex Array
        vbo = self.context.buffer(self.mesh.pack())
        vao = self.context.simple_vertex_array(self.prog, vbo, *['in_vert', 'in_text', 'in_norm'])
        vbo_grid = self.context.buffer(grid.astype('f4').tobytes())
        vao_grid = self.context.simple_vertex_array(self.grid_prog, vbo_grid, 'in_vert')

        # Framebuffers
        # Use 4 samples for MSAA anti-aliasing
        fbo1 = self.context.framebuffer(
            self.context.renderbuffer((self.width, self.height), samples=4),
            depth_attachment=self.context.depth_renderbuffer(
                (self.width, self.height), samples=4
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
        vao_grid.render(moderngl.LINES, 65 * 4)
        self.context.copy_framebuffer(fbo2, fbo1)

        # Loading the image using Pillow
        img = Image.frombytes('RGBA', fbo2.size, fbo2.read(components=4,
                                                         alignment=1), 'raw', 'RGBA', 0, -1)

        '''
            Apply distortion and rotation using the camera parameters computed above
        '''
        # TODO

        return (img, image_frame_gate_center, rotation)
