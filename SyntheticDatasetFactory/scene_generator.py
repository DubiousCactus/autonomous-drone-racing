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

from pyrr import Matrix44, Quaternion, Vector3
from moderngl.ext.obj import Obj
from PIL import Image


class SceneGenerator:
    def __init__(self, mesh_path: str, width: int, height: int,
                 world_boundaries, gate_center: Vector3):
        self.mesh = Obj.open(mesh_path)
        self.width = width
        self.height = height
        self.gate_center = gate_center
        self.boundaries = self.compute_boundaries(world_boundaries)
        self.setup_opengl()
        random.seed()

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

        print("Mesh width: {}\nMesh height: {}".format(mesh_width, mesh_height))

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



    def generate(self):
        ''' Randomly move the gate around, while keeping it inside the boundaries '''
        translation = Vector3([
            random.uniform(-self.boundaries['x'], self.boundaries['x']),
            0,
            random.uniform(-self.boundaries['z'], self.boundaries['z'])
        ])
        print("Randomized translation: {}".format(translation))

        ''' Randomly rotate the gate horizontally, around the Z-axis '''
        rotation = Quaternion.from_z_rotation(random.random() * np.pi)
        print("Randomized rotation: {}".format(rotation))

        scale = Vector3([1., 1., 1.]) # Scale it by a factor of 1
        # model = Matrix44.from_translation(translation) * rotation * Matrix44.from_scale(scale)
        model = Matrix44.identity()
        gate_center = model * self.gate_center

        print("Gate center: {}".format(gate_center))

        # Perspective projection
        '''
            TODO: Get intrinsics from camera calibration using OpenCV
            (convert_hz_intrinsic_to_opengl_projection)
        '''
        projection = Matrix44.perspective_projection(
            60.0, # field of view in y direction in degrees (vertical FoV)
            self.width/self.height, # aspect ratio of the view
            0.1, # distance from the viewer to the near clipping plane (only positive)
            1000.0 # distance from the viewer to the far clipping plane (only positive)
        )

        '''
            - Compute 2 vanishing points to find the horizon
            - Compute the height of the camera

            IF NOT POSSIBLE (because how do you automate this? Canny edge detection +
            Hough transform?):
                - Record the height of the drone at each frame and annotate that on the
                dataset
                - Record the roll, pitch, yaw to apply to the target of the camera
                (look_at)
        '''
        # Camera view matrix
        '''
         x: horizontal axis
         y: vertical axis
         z: depth axis
        '''
        view = Matrix44.look_at(
            (3, 3, 3), # eye: position of the camera in world coordinates
            (0.0, 0.0, 0.0), # target: position in world coordinates that the camera is looking at
            (0.0, 0.0, 1.0), # up: up vector of the camera
        )

        # Model View Projection matrix
        mvp = projection * view * model

        # Shader program
        self.prog['Light'].value = (0.0, 10.0, 0.0) # TODO
        self.prog['Color'].value = (1.0, 1.0, 1.0, 0.25) # TODO
        self.prog['Mvp'].write(mvp.astype('f4').tobytes())

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
        vao_grid = self.context.simple_vertex_array(self.prog, vbo_grid, 'in_vert')

        # Framebuffers
        fbo = self.context.framebuffer(
            self.context.renderbuffer((self.width, self.height)),
            self.context.depth_renderbuffer((self.width, self.height)),
        )
        # fbo = self.context.simple_framebuffer((self.width, self.height),
                                              # components=4, samples=4)

        # Rendering
        fbo.use()
        self.context.enable(moderngl.DEPTH_TEST)
        self.context.clear(1.0, 1.0, 1.0)
        # self.context.enable(moderngl.BLEND)
        # self.context.clear(0.0, 0.0, 0.0)
        texture.use()
        vao.render()
        vao_grid.render(moderngl.LINES, 65 * 4)

        # Loading the image using Pillow
        img = Image.frombytes('RGBA', fbo.size, fbo.read(components=4,
                                                         alignment=1), 'raw', 'RGBA', 0, -1)

        '''
            Apply distortion and rotation using the camera parameters computed above
        '''
        # TODO

        return img
