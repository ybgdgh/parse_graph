# -*- coding:utf-8 -*-
import os.path as osp
import os

import cv2
import webcolors

from graphviz import Digraph

tomato_rgb = [236, 93, 87]
blue_rgb = [81, 167, 250]
tomato_hex = webcolors.rgb_to_hex(tomato_rgb)
blue_hex = webcolors.rgb_to_hex(blue_rgb)
pale_rgb = [112,191,64]
pale_hex = webcolors.rgb_to_hex(pale_rgb)
orange_rgb = [255, 160, 0]
orange_hex = webcolors.rgb_to_hex(orange_rgb)
contain_rgb = [0, 255, 255]
contain_hex = webcolors.rgb_to_hex(contain_rgb)

save_path = '/home/ybg/ROS_code/catkin_vision/src/parse_graph/vis_result/'

sg = Digraph('structs', format='png')  # initialize scene graph tool

# root
sg.node('struct_scene', shape='box', style='filled,rounded',
        label='scene', margin='0.11, 0.0001', width='0.11', height='0',
        fillcolor=tomato_hex, fontcolor='black')

def add_scene(scene):
    print("add scene for parse graph: ")
    print(scene)    
    sg.node('struct'+str(scene), shape='box', style='filled,rounded',
            label=scene, margin='0.11, 0.0001', width='0.11', height='0',
            fillcolor=contain_hex, fontcolor='black')
    sg.edge('struct_scene', 'struct'+str(scene))
    

def add_node(current_scene, object_, pose_x, pose_y, pose_z, color):
    print("add node for parse graph" )
    print(object_)
    
    sg.node('struct'+str(object_), shape='box', style='filled,rounded',
            label=object_, margin='0.11, 0.0001', width='0.11', height='0',
            fillcolor=tomato_hex, fontcolor='black')

    sg.node('regular'+str(object_), shape='box', style='filled,rounded',
            label='regular', margin='0.11, 0.0001', width='0.11', height='0',
            fillcolor=orange_hex, fontcolor='black')

    sg.node('address'+str(object_), shape='box', style='filled,rounded',
            label='address', margin='0.11, 0.0001', width='0.11', height='0',
            fillcolor=orange_hex, fontcolor='black')

    sg.node('relation'+str(object_), shape='box', style='filled,rounded',
            label='address', margin='0.11, 0.0001', width='0.11', height='0',
            fillcolor=orange_hex, fontcolor='black')

    sg.node('attribute_pose_'+str(object_), shape='box', style='filled, rounded',
            label="("+str(round(float(pose_x), 1))+"," +
            str(round(float(pose_y), 1))+"," +
            str(round(float(pose_z), 1)) + ")",
            margin='0.11, 0.0001', width='0.11', height='0',
            fillcolor=blue_hex, fontcolor='black')

    sg.node('attribute_color_'+str(object_), shape='box', style='filled, rounded',
            label=str(color),
            margin='0.11, 0.0001', width='0.11', height='0',
            fillcolor=blue_hex, fontcolor='black')

    sg.edge('struct'+str(current_scene), 'struct'+str(object_))
    sg.edge('struct'+str(object_),'regular'+str(object_))
    sg.edge('struct'+str(object_),'address'+str(object_))
    sg.edge('regular'+str(object_), 'attribute_pose_'+str(object_))
    sg.edge('regular'+str(object_), 'attribute_color_'+str(object_))
    sg.edge('address'+str(object_), 'relation'+str(object_))


def add_rela(subject, object_, relation):
    print("add relations for parse graph: ")
    print(subject, object_, relation)

    sg.node('rel'+str(subject)+str(object_), shape='box', style='filled, rounded', fillcolor=pale_hex, fontcolor='black',
            margin='0.11, 0.0001', width='0.11', height='0', label=str(relation))
    sg.edge('relation'+str(subject), 'rel'+str(subject)+str(object_))
    sg.edge('rel'+str(subject)+str(object_),'relation'+str(object_), constraint='false')


def viz_pg():
    print("update parse graph!")
    sg.render(osp.join(save_path, 'scene_graph'), view=False)
    img = cv2.imread(osp.join(save_path, 'scene_graph'+'.png'), cv2.IMREAD_COLOR)
    resize_x = 0.65
    resize_y = 0.9
    if img.shape[1] < int(1920*resize_x) and img.shape[0] < int(1080*resize_y):
        pad = cv2.resize(img.copy(), (int(1920*resize_x), int(1080*resize_y)))
        pad.fill(255)
        pad[:img.shape[0], :img.shape[1], :] = img
        resized = pad
    elif img.shape[1] < int(1920*resize_x):
        pad = cv2.resize(
            img.copy(), (int(1920 * resize_x), int(1080 * resize_y)))
        pad.fill(255)
        img = cv2.resize(img, (img.shape[1], int(1080 * resize_y)))
        pad[:img.shape[0], :img.shape[1], :] = img
        resized = pad
    elif img.shape[0] < int(1080*resize_y):
        pad = cv2.resize(
            img.copy(), (int(1920 * resize_x), int(1080 * resize_y)))
        pad.fill(255)
        img = cv2.resize(img, (int(1920 * resize_x), img.shape[0]))
        pad[:img.shape[0], :img.shape[1], :] = img
        resized = pad
    else:
        resized = cv2.resize(img, (int(1920*resize_x), int(1080*resize_y)))
    print("output image!")
    
    cv2.imshow('3D Scene Graph', resized)
    cv2.moveWindow('3D Scene Graph', 650, 0)
    cv2.waitKey(1)

    sg.clear()
    
