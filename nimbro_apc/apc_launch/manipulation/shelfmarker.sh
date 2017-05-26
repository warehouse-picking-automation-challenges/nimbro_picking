#!/bin/bash

rostopic pub /shelf_marker visualization_msgs/Marker "header:
  seq: 0
  stamp: 0
  frame_id: 'shelf'
ns: ''
id: 0
type: 10
action: 0
pose:
  position: {x: 0.0, y: 0.0, z: 0.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
scale: {x: 1.0, y: 1.0, z: 1.0}
color: {r: 0.3, g: 0.3, b: 0.3, a: 0.7}
lifetime: {secs: 0, nsecs: 0}
frame_locked: true
points: []
colors: []
text: ''
mesh_resource: 'package://apc_model/meshes/shelf.stl'
mesh_use_embedded_materials: false"