#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2019 Theo Morales <theo.morales.fr@gmail.com>
#
# Distributed under terms of the MIT license.

"""
SceneRenderer

Generates an image by projecting a 3D mesh over a 2D transparent background.
"""


import numpy as np
import quaternion
import moderngl
import random
import yaml

from pyrr import Matrix44, Quaternion, Vector3, Vector4, vector
from moderngl.ext.obj import Obj
from math import degrees, radians, cos, sin
from PIL import Image


class SceneRenderer:
    def __init__(self, mesh_path: str, width: int, height: int,
                 world_boundaries, gate_center: Vector3, camera_parameters,
                 render_perspective=False, seed=None):
        if seed:
            random.seed(seed)
        else:
            random.seed()
        self.render_perspective = render_perspective
        self.mesh = Obj.open(mesh_path)
        self.width = width
        self.height = height
        self.gate_center = gate_center
        self.boundaries = self.compute_boundaries(world_boundaries)
        with open(camera_parameters, 'r') as cam_file:
            try:
                self.camera_parameters = yaml.safe_load(cam_file)
            except yaml.YAMLError as exc:
                raise Exception(exc)
        self.setup_opengl()

    def compute_boundaries(self, world_boundaries):
        return  {
            'x': world_boundaries['x'] / 2,
            'y': world_boundaries['y'] / 2
        }

    def set_drone_pose(self, drone_pose):
        self.drone_pose = drone_pose
        self.gate_poses = []

    def setup_opengl(self):
        self.context = moderngl.create_standalone_context()
        camera_intrinsics = [
            self.camera_parameters['camera_matrix']['data'][0:3],
            self.camera_parameters['camera_matrix']['data'][3:6],
            self.camera_parameters['camera_matrix']['data'][6::]
        ]
        fx, fy = camera_intrinsics[0][0], camera_intrinsics[1][1]
        cx, cy = camera_intrinsics[0][2], camera_intrinsics[1][2]
        x0, y0 = 0, 0 # Camera image origin
        zfar, znear = 1000.0, 0.1 # distances to the clipping plane
        # Works by following: https://blog.noctua-software.com/opencv-opengl-projection-matrix.html
        # Doesn't work by following: http://kgeorge.github.io/2014/03/08/calculating-opengl-perspective-matrix-from-opencv-intrinsic-matrix
        self.projection = Matrix44([
            [fx/cx, 0, 0, 0],
            [0, fy/cy, 0, 0],
            [0, 0, (-zfar - znear)/(zfar - znear), -1],
            [0, 0, (-2.0*zfar*znear)/(zfar - znear), 0] # TODO: EXPLAIN WHY IT WORKS WHEN I FLIP [2][2] AND [3][2]
        ])

    def destroy(self):
        self.context.release()

    def render_gate(self, view):
        ''' Randomly move the gate around, while keeping it inside the boundaries '''
        gate_translation = None
        too_close = True
        # Prevent gates from spawning too close to each other
        while too_close:
            too_close = False
            gate_translation = Vector3([
                random.uniform(-self.boundaries['x'], self.boundaries['x']),
                random.uniform(-self.boundaries['y'], self.boundaries['y']),
                0
            ])
            for gate_pose in self.gate_poses:
                if np.linalg.norm(gate_pose - gate_translation) <= 2:
                    too_close = True
                    break

        self.gate_poses.append(gate_translation)

        ''' Randomly rotate the gate horizontally, around the Z-axis '''
        gate_rotation = Quaternion.from_z_rotation(random.random() * np.pi)
        model = Matrix44.from_translation(gate_translation) * gate_rotation

        # Model View Projection matrix
        mvp = self.projection * view * model

        # Shader program
        vertex_shader_source = open('data/shader.vert').read()
        fragment_shader_source = open('data/shader.frag').read()
        prog = self.context.program(vertex_shader=vertex_shader_source,
                                         fragment_shader=fragment_shader_source)
        prog['Light'].value = (0.0, 0.0, 4.0) # TODO
        prog['Color'].value = (1.0, 1.0, 1.0, 0.25) # TODO
        prog['Mvp'].write(mvp.astype('f4').tobytes())

        # Vertex Buffer and Vertex Array
        vbo = self.context.buffer(self.mesh.pack())
        vao = self.context.simple_vertex_array(prog, vbo, *['in_vert', 'in_text', 'in_norm'])

        return vao, model, gate_translation, gate_rotation

    '''
        Converting the gate center's world coordinates to image coordinates
    '''
    def compute_gate_center(self, view, model):
        # Return if the camera is within 80cm of the gate, because it's not
        # visible
        if np.linalg.norm(self.gate_center-self.drone_pose.translation) <= 1:
            return [-1, -1]

        # TODO: Move the gate center back to the image frame if it's slightly
        # outside of the image frame (the gate frame is still visible and we can
        # guess where to steer)

        '''
        If the gate is less than 2m ahead, and if the gate center in pixels is
        less than 1/5 of the image size outside, clip it to max x or min x
        '''

        gate_center = model * self.gate_center
        clip_space_gate_center = self.projection * (view *
                                                    Vector4.from_vector3(gate_center,
                                                                         w=1.0))

        if clip_space_gate_center.w != 0:
            normalized_device_coordinate_space_gate_center\
                = Vector3(clip_space_gate_center.xyz) / clip_space_gate_center.w
        else: # Clipped
            normalized_device_coordinate_space_gate_center = clip_space_gate_center.xyz

        viewOffset = 0
        image_frame_gate_center =\
            ((np.array(normalized_device_coordinate_space_gate_center.xy) + 1.0) /
             2.0) * np.array([self.width, self.height]) + viewOffset

        # Translate from bottom-left to top-left
        image_frame_gate_center[1] = self.height - image_frame_gate_center[1]

        return image_frame_gate_center

    '''
        Project the perspective as a grid (might need some tuning for
        non-square environments)
    '''
    def render_perspective_grid(self, view):
        vertex_shader_source = open('data/shader.vert').read()
        fragment_shader_source = open('data/shader.frag').read()
        grid_prog = self.context.program(vertex_shader=vertex_shader_source,
                             fragment_shader=fragment_shader_source)

        grid = []
        x_length = int(self.boundaries['x'])
        for i in range(x_length * 2 + 1):
            grid.append([i - x_length, -x_length, 0.0,
                         i - x_length, x_length, 0.0])
            grid.append([-x_length, i - x_length, 0.0,
                         x_length, i - x_length, 0.0])

        grid = np.array(grid)

        vp = self.projection * view
        grid_prog['Light'].value = (0.0, 0.0, 6.0) # TODO
        grid_prog['Color'].value = (1.0, 1.0, 1.0, 0.25) # TODO
        grid_prog['Mvp'].write(vp.astype('f4').tobytes())

        vbo = self.context.buffer(grid.astype('f4').tobytes())
        vao = self.context.simple_vertex_array(grid_prog, vbo, 'in_vert')

        return vao

    '''
        Returns the Euclidean distance of the gate to the camera
    '''
    def compute_camera_proximity(self, view, model):
        coords = self.compute_gate_center(view, model)
        if coords[0] < 0 or coords[0] > self.width or coords[1] < 0 or coords[1] > self.height:
            return gate_center, 1000
        else:
            return gate_center, np.linalg.norm((model * self.gate_center) - self.drone_pose.translation)

    def generate(self, background_gates=True, max_gates=6):
        # Camera view matrix
        view = Matrix44.look_at(
            # eye: position of the camera in world coordinates
            self.drone_pose.translation,
            # target: position in world coordinates that the camera is looking at
            self.drone_pose.translation + (self.drone_pose.orientation *
                                           Vector3([1.0, 0.0, 0.0])),
            # up: up vector of the camera.
            self.drone_pose.orientation * Vector3([0.0, 0.0, 1.0]),
        )

        # Texturing
        texture_image = Image.open('data/orange_texture.jpg')
        texture = self.context.texture(texture_image.size, 3, texture_image.tobytes())
        texture.build_mipmaps()

        # Framebuffers
        # Use 4 samples for MSAA anti-aliasing
        msaa_render_buffer = self.context.renderbuffer((self.width,
                                                        self.height),
                                                       samples=8)
        msaa_depth_render_buffer = self.context.depth_renderbuffer((self.width,
                                                                    self.height),
                                                                   samples=8)
        fbo1 = self.context.framebuffer(
            msaa_render_buffer,
            depth_attachment=msaa_depth_render_buffer)

        # Downsample to the final framebuffer
        render_buffer = self.context.renderbuffer((self.width, self.height))
        depth_render_buffer = self.context.depth_renderbuffer((self.width, self.height))
        fbo2 = self.context.framebuffer(render_buffer, depth_render_buffer)

        # Rendering
        fbo1.use()
        self.context.enable(moderngl.DEPTH_TEST)
        self.context.clear(0, 0, 0, 0)
        texture.use()

        gate_center = None
        gate_translation = None
        gate_rotation = None
        min_prox = None
        # Render at least one gate
        for i in range(random.randint(1, max_gates)):
            vao, model, translation, rotation = self.render_gate(view)
            center, proximity = self.compute_camera_proximity(view, model)
            # Pick the target gate: the closest to the camera
            if min_prox is None or proximity < min_prox:
                min_prox = proximity
                gate_center = center
                gate_translation = translation
                gate_rotation = rotation
            vao.render()
            vao.release()

        if self.render_perspective:
            vao_grid = self.render_perspective_grid(view)
            vao_grid.render(moderngl.LINES, 65 * 4)
            vao_grid.release()

        self.context.copy_framebuffer(fbo2, fbo1)

        # Loading the image using Pillow
        img = Image.frombytes('RGBA', fbo2.size, fbo2.read(components=4,
                                                         alignment=1), 'raw', 'RGBA', 0, -1)

        '''
            Apply distortion using the camera parameters
        '''
        # TODO

        annotations = {
            'gate_center_img_frame': gate_center,
            'gate_position': gate_translation,
            'gate_rotation':
                [degrees(x) for x in
                 quaternion.as_euler_angles(np.quaternion(*gate_rotation.xyzw))][1],
            'drone_pose': self.drone_pose.translation,
            'drone_orientation':
                [degrees(x) for x in
                 quaternion.as_euler_angles(np.quaternion(*self.drone_pose.orientation.xyzw))]
        }

        '''
        A soon-to-be-fixed bug in ModernGL forces me to release the render
        buffers manually
        '''
        msaa_render_buffer.release()
        msaa_depth_render_buffer.release()
        render_buffer.release()
        depth_render_buffer.release()
        fbo1.release()
        fbo2.release()
        texture.release()

        return (img, annotations)

