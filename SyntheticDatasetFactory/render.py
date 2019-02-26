#!/usr/bin/python3

import numpy as np
import moderngl
import random

from pyrr import Matrix44, Quaternion, Vector3
from moderngl.ext.obj import Obj
from PIL import Image


'''
    ----- TODO -----

[x] Random positioning of the gate
[x] Boundaries definition for the gate (relative to the mesh's size)
[ ] Compute the center of the gate
[ ] Compute the presence of the gate in the image frame
[?] Compute the distance to the gate
[ ] Camera calibration (use the correct parameters)
[x] Project on transparent background
[ ] Overlay with background image
[ ] Histogram equalization of both images (hue, saturation, luminence ?...)
[ ] Motion blur (shader ?)
[ ] Anti alisasing (shader ?)
[ ] Ship it!

'''

# Parameters
width = 640
height = 480
world_boundaries = {'x': 8, 'y': 10, 'z': 0} # Real world boundaries in meters (relative to the mesh's scale)

# Data files
mesh = Obj.open('data/gate_1mx1m_150cm.obj')
vertex_data = mesh.pack()
texture_image = Image.open('data/shiny-white-metal-texture.jpg')
vertex_shader_source = open('data/shader.vert').read()
fragment_shader_source = open('data/shader.frag').read()

# Set the orthographic coordinates for the boundaries, based on the size of the
# mesh
mesh_width = abs(
    min(mesh.vert, key = lambda v_pair: v_pair[0])[0]
    - max(mesh.vert, key = lambda v_pair: v_pair[0])[0]
)
mesh_height = abs(
    min(mesh.vert, key = lambda v_pair: v_pair[2])[2]
    - max(mesh.vert, key = lambda v_pair: v_pair[2])[2]
)
boundaries = {
    'x': mesh_width * world_boundaries['x'],
    'y': mesh_width * world_boundaries['y'],
    'z': mesh_height * world_boundaries['z']
}

print("Mesh width: {}\nMesh height: {}".format(mesh_width, mesh_height))

# Context creation
ctx = moderngl.create_standalone_context()

# Shaders
prog = ctx.program(vertex_shader=vertex_shader_source, fragment_shader=fragment_shader_source)

# Model translation, rotation, scaling
random.seed()
''' Randomly move the gate around, while keeping it inside the boundaries '''
translation = Vector3([
    random.uniform(-boundaries['x'], boundaries['x']),
    random.uniform(-boundaries['y'], boundaries['y']),
    random.uniform(-boundaries['z'], boundaries['z'])
])
print("Randomized translation: {}".format(translation))

''' Randomly rotate the gate horizontally, around the Y-axis '''
rotation = Quaternion.from_y_rotation(random.random() * np.pi)
print("Randomized rotation: {}".format(rotation))

scale = Vector3([1., 1., 1.]) # Scale it by a factor of 1
model = Matrix44.from_translation(translation) * rotation * Matrix44.from_scale(scale)

# Perspective projection
projection = Matrix44.perspective_projection(
    45.0, # field of view in y direction in degrees (vertical FoV)
    width/height, # aspect ratio of the view
    0.01, # distance from the viewer to the near clipping plane (only positive)
    1000.0 # distance from the viewer to the far clipping plane (only positive)
)

# Camera view matrix
'''
 x: horizontal axis
 y: vertical axis
 z: depth axis
'''
view = Matrix44.look_at(
    (0, 5, 20), # eye: position of the camera in world coordinates
    (0.0, 5.0, 0.0), # target: position in world coordinates that the camera is looking at
    (0.0, 1.0, 0.0), # up: up vector of the camera
)

# Model View Projection matrix
mvp = projection * view * model

# Shader program
prog['Light'].value = (-40.0, -30.0, 80.0) # TODO
prog['Color'].value = (1.0, 1.0, 1.0, 0.25) # TODO
prog['Mvp'].write(mvp.astype('f4').tobytes())

# Texturing
texture = ctx.texture(texture_image.size, 3, texture_image.tobytes())
texture.build_mipmaps()

# Vertex Buffer and Vertex Array
vbo = ctx.buffer(vertex_data)
vao = ctx.simple_vertex_array(prog, vbo, *['in_vert', 'in_text', 'in_norm'])

# Framebuffers
fbo = ctx.framebuffer(
    ctx.renderbuffer((width, height)),
    ctx.depth_renderbuffer((width, height)),
)

# Rendering
fbo.use()
ctx.enable(moderngl.BLEND)
ctx.clear(0.0, 0.0, 0.0)
texture.use()
vao.render()

# Loading the image using Pillow
data = fbo.read(components=4, alignment=1)
img = Image.frombytes('RGBA', fbo.size, data, 'raw', 'RGBA', 0, -1)

# TODO: blend the image over the background
# output = Image.blend(img, background, 0.0)

img.save('output.png')
img.show()
