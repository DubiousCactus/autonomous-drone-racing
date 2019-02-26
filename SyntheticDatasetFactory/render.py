#!/usr/bin/env python

import numpy as np
import moderngl

from pyrr import Matrix44, Quaternion, Vector3
from moderngl.ext.obj import Obj
from PIL import Image


# Data files
vertex_data = Obj.open('data/gate_triangulated_faces.obj').pack()
texture_image = Image.open('data/wood.jpg')
vertex_shader_source = open('data/shader.vert').read()
fragment_shader_source = open('data/shader.frag').read()

# Context creation
ctx = moderngl.create_standalone_context()

# Shaders
prog = ctx.program(vertex_shader=vertex_shader_source, fragment_shader=fragment_shader_source)

# Matrices and Uniforms
translation = Vector3()
translation += [-45.0, 25.0, 0.0] # Translate the mesh

orientation = Quaternion()
rotation = Quaternion.from_z_rotation(np.pi / 4.0) # Rotate about Z by pi/2
orientation = rotation * orientation

scale = Vector3([50., 50., 50.]) # Scale it by a factor of 3

transRotScaleMatrix = Matrix44.identity()
transRotScaleMatrix = transRotScaleMatrix * Matrix44.from_translation(translation)
transRotScaleMatrix = transRotScaleMatrix * orientation # Multiply matrices and quaternions directly
transRotScaleMatrix = transRotScaleMatrix * Matrix44.from_scale(scale)

perspective = Matrix44.perspective_projection(
    45.0, # field of view in y direction in degrees
    1.0, # aspect ratio of the view (width / height)
    0.01, # distance from the viewer to the near clipping plane (only positive)
    1000.0 # distance from the viewer to the far clipping plane (only positive)
)
print("perspective projection: \n{}".format(perspective))
lookat = Matrix44.look_at(
    (0, -250, 400), # eye: position of the camera in world coordinates
    (0.0, 0.0, 0.0), # target: position in world coordinates that the camera is looking at
    (0.0, 0.0, 1.0), # up: up vector of the camera
)
print("lookat: \n{}".format(lookat))

mvp = perspective * lookat * transRotScaleMatrix
print("model view projection: \n{}".format(mvp))

prog['Light'].value = (-140.0, -300.0, 350.0)
prog['Color'].value = (1.0, 1.0, 1.0, 0.25)
prog['Mvp'].write(mvp.astype('f4').tobytes())

# Texture
texture = ctx.texture(texture_image.size, 3, texture_image.tobytes())
texture.build_mipmaps()

# Vertex Buffer and Vertex Array
vbo = ctx.buffer(vertex_data)
vao = ctx.simple_vertex_array(prog, vbo,* ['in_vert', 'in_text', 'in_norm'])

# Framebuffers
fbo = ctx.framebuffer(
    ctx.renderbuffer((512, 512)),
    ctx.depth_renderbuffer((512, 512)),
)

# Rendering
fbo.use()
ctx.enable(moderngl.DEPTH_TEST)
ctx.clear(0.9, 0.9, 0.9)
texture.use()
vao.render()

# Loading the image using Pillow
data = fbo.read(components=3, alignment=1)
img = Image.frombytes('RGB', fbo.size, data, 'raw', 'RGB', 0, -1)
img.save('output.png')
img.show()
