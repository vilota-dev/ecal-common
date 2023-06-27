#!/usr/bin/env python3

import sys
import time
from time import monotonic
from datetime import datetime
import threading

import capnp
import numpy as np
import cv2
import platform

import ecal.core.core as ecal_core
from byte_subscriber import ByteSubscriber
import copy

capnp.add_import_hook(['../src/capnp'])

import sys
sys.path.insert(0, "../python")

import odometry3d_capnp as eCALOdometry3d
import image_capnp as eCALImage
import cameracontrol_capnp as eCALCameraControl
import open3d as o3d
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering

from scipy.spatial.transform import Rotation as R
import sophus as sp

from PIL import Image

from subscriber_utils import TagDetectionsSubscriber, VioSubscriber, add_ui_on_ndarray, is_numeric
from utils import SyncedImageSubscriber

from o3d_utils import create_grid_mesh

isMacOS = (platform.system() == "Darwin")

image_topics = []
image_types = []
image_typeclasses = []
flag_dict = {}
flag_dict['vio_status'] = False
flag_dict['synced_status'] = False
flag_dict['thumbnail_status'] = False
flag_dict['csv_status'] = False

JUMP_THRESHOLD = 2 * 1e9

class ChooseWindow:

    def __init__(self):
        self.is_done = False

        # CONFIGURE WINDOW
        self.window = gui.Application.instance.create_window(
            "Camera confirm", 500, 350)
        self.window.set_on_layout(self._on_layout)
        self.window.set_on_close(self._on_close) 
        
        # CONFIGURE GUI
        em = self.window.theme.font_size
        margin = 0.5 * em
        
        self.panel_main = gui.Vert(0.5 * em, gui.Margins(margin))

        self.label_description = gui.Label("Please choose the avaliable camera.")
        self.label_description.text_color = gui.Color(1.0, 0.5, 0.0)
        self.panel_main.add_child(self.label_description)

        self.cb_cama = gui.Checkbox("cam_a")
        self.panel_main.add_child(self.cb_cama)

        self.cb_camb = gui.Checkbox("cam_b")        
        # self.cb_camb.checked = True
        self.panel_main.add_child(self.cb_camb)

        self.cb_camc = gui.Checkbox("cam_c")
        # self.cb_camc.checked = True
        self.panel_main.add_child(self.cb_camc)

        self.cb_camd = gui.Checkbox("cam_d")
        self.cb_camd.checked = True
        self.panel_main.add_child(self.cb_camd)

        self.label_description = gui.Label("Please choose the specific features.")
        self.label_description.text_color = gui.Color(1.0, 0.5, 0.0)
        self.panel_main.add_child(self.label_description)

        self.cb_vio = gui.Checkbox("vio is on")
        self.cb_vio.checked = True
        self.panel_main.add_child(self.cb_vio)

        self.cb_synced = gui.Checkbox("use synced image (default asynced)")
        self.panel_main.add_child(self.cb_synced)

        self.cb_thumbnail = gui.Checkbox("use thumbnail image")
        self.cb_thumbnail.checked = True
        self.panel_main.add_child(self.cb_thumbnail)

        self.cb_csv = gui.Checkbox("store odometry data in csv")
        self.panel_main.add_child(self.cb_csv)

        self.window.add_child(self.panel_main) 


        self.button_layout = gui.Horiz(0.5 * em, gui.Margins(margin,margin,margin,margin))
        self.button_layout.add_stretch()

        self.ok_button = gui.Button("Ok")
        self.ok_button.set_on_clicked(self._on_ok)
        self.button_layout.add_child(self.ok_button)
        
        self.window.add_child(self.button_layout) 

    def _on_layout(self, layout_context):
        contentRect = self.window.content_rect

        self.panel_main.frame = gui.Rect(contentRect.x, 
                                contentRect.y,
                                contentRect.width ,
                                contentRect.height - 50)
        self.button_layout.frame = gui.Rect(contentRect.x,
                                self.panel_main.frame.get_bottom(),
                                contentRect.width,
                                50)

    def _on_close(self):
        self.is_done = True
        return True  # False would cancel the close

    def _on_ok(self):
        global image_types, image_typeclasses
        num_subs = 0
        if self.cb_thumbnail.checked:
            flag_dict['thumbnail_status'] = True
            if self.cb_cama.checked:
                image_topics.append("S0/thumbnail/cama")
                num_subs += 1
            if self.cb_camb.checked:
                image_topics.append("S0/thumbnail/camb") 
                num_subs += 1
            if self.cb_camc.checked:
                image_topics.append("S0/thumbnail/camc")
                num_subs += 1
            if self.cb_camd.checked:
                image_topics.append("S0/thumbnail/camd")
                num_subs += 1
        else:
            if self.cb_cama.checked:
                image_topics.append("S0/cama")
                num_subs += 1
            if self.cb_camb.checked:
                image_topics.append("S0/camb") 
                num_subs += 1
            if self.cb_camc.checked:
                image_topics.append("S0/camc")
                num_subs += 1
            if self.cb_camd.checked:
                image_topics.append("S0/camd")
                num_subs += 1

        image_types = ["Image" for _ in range(num_subs)]
        image_typeclasses = [eCALImage.Image for _ in range(num_subs)]
        
        if self.cb_vio.checked:
            flag_dict['vio_status'] = True
        
        if self.cb_synced.checked:
            flag_dict['synced_status'] = True

        if self.cb_csv.checked:
            flag_dict['csv_status'] = True

        print(f"Subscribing to {image_topics}")

        gui.Application.instance.quit()


class VideoWindow:
    MENU_QUIT = 1
   

    def __init__(self):
        self.is_done = False

        self.expTime_display_flag = False
        self.sensIso_display_flag = False
        self.latencyDevice_display_flag = False
        self.latencyHost_display_flag = False
        self.tagDebug_display_flag = False

        self.expMax = 12000
        self.sensIsoMax = 800

        # CONFIGURE FONTS
        self.font_large = gui.FontDescription(typeface="monospace",
                            style=gui.FontStyle.NORMAL,
                            point_size=25)
        self.font_id_large = gui.Application.instance.add_font(self.font_large)

        # CONFIGURE WINDOW
        self.window = gui.Application.instance.create_window(
            "Drone Monitor", 1300, 800)
        self.window.set_on_layout(self._on_layout_odom)
        self.window.set_on_close(self._on_close)


        # CONFIGURE MENU
        if gui.Application.instance.menubar is None:
            if isMacOS:
                app_menu = gui.Menu()
                app_menu.add_item("Quit", VideoWindow.MENU_QUIT)
            
            # first sub menu
            debug_menu = gui.Menu()
            if not isMacOS:
                # debug_menu.add_separator()
                debug_menu.add_item("Quit", VideoWindow.MENU_QUIT)
            
            # all the submenu collection
            menu = gui.Menu() 
            if isMacOS:
                # macOS will name the first menu item for the running application
                # (in our case, probably "Python"), regardless of what we call
                # it. This is the application menu, and it is where the
                # About..., Preferences..., and Quit menu items typically go.
                menu.add_menu("Example", app_menu)
                menu.add_menu("Menu", debug_menu)
            else:
                menu.add_menu("Menu", debug_menu)
            gui.Application.instance.menubar = menu

        # connect the menu items to the window
        self.window.set_on_menu_item_activated(VideoWindow.MENU_QUIT,
                                               self._on_menu_quit)



        # CONFIGURE GUI
        em = self.window.theme.font_size
        margin = 0.5 * em
        
        # side 
        self.collapse = gui.CollapsableVert("Control panel", 0.33 * em,
                                       gui.Margins(em, 0, 0, 0))
        
        # change display radio buttoh
        self.rb_display = gui.RadioButton(gui.RadioButton.HORIZ)
        self.rb_display.set_items(["Odometry", "Video"])
        self.rb_display.set_on_selection_changed(self._select_rb_display)
        self.collapse.add_child(self.rb_display)


        # tab 
        tabs = gui.TabControl()

        # odom tab
        odom_tab = gui.Vert(0.5 * em, gui.Margins(8,8,16,8))

        label_display_control = gui.Label("Edit odometry")
        label_display_control.text_color = gui.Color(1.0, 0.5, 0.0)
        odom_tab.add_child(label_display_control)

        self.cb_grid = gui.Checkbox("Floor grid")
        self.cb_grid.checked = True
        self.cb_grid.set_on_checked(self._on_cb_grid)
        odom_tab.add_child(self.cb_grid)

        self.cb_land = gui.Checkbox("Land model")
        self.cb_land.checked = False
        self.cb_land.set_on_checked(self._on_cb_land)
        odom_tab.add_child(self.cb_land)

        self.land_current_degree = 0
        self.land_rotate_interval = 2
        btn_land_clk = gui.Button(f"Clockwise {self.land_rotate_interval}\u00B0")
        btn_land_clk.set_on_clicked(self._btn_land_clk)
        odom_tab.add_child(btn_land_clk)        
        
        btn_land_anti = gui.Button(f"Anticlockwise {self.land_rotate_interval}\u00B0")
        btn_land_anti.set_on_clicked(self._btn_land_anti)
        odom_tab.add_child(btn_land_anti)

        btn_clear = gui.Button("Clear path")
        btn_clear.set_on_clicked(self._btn_clear)
        odom_tab.add_child(btn_clear)

        label_display_control = gui.Label("Points")
        label_display_control.text_color = gui.Color(1.0, 0.5, 0.0)
        odom_tab.add_child(label_display_control)

        btn_st_pt = gui.Button("Mark starting point")
        btn_st_pt.vertical_padding_em = 0
        btn_st_pt.horizontal_padding_em = 0
        btn_st_pt.set_on_clicked(self._btn_st_pt)
        odom_tab.add_child(btn_st_pt)

        btn_st_clr = gui.Button("Clear starting point")
        btn_st_clr.vertical_padding_em = 0
        btn_st_clr.horizontal_padding_em = 0
        btn_st_clr.set_on_clicked(self._btn_st_clr)
        odom_tab.add_child(btn_st_clr)

        btn_ed_pt = gui.Button("Mark end point")
        btn_ed_pt.vertical_padding_em = 0
        btn_ed_pt.horizontal_padding_em = 0
        btn_ed_pt.set_on_clicked(self._btn_ed_pt)
        odom_tab.add_child(btn_ed_pt)

        btn_ed_clr = gui.Button("Clear end point")
        btn_ed_clr.vertical_padding_em = 0
        btn_ed_clr.horizontal_padding_em = 0
        btn_ed_clr.set_on_clicked(self._btn_ed_clr)
        odom_tab.add_child(btn_ed_clr)

        label_display_control = gui.Label("Camera view")
        label_display_control.text_color = gui.Color(1.0, 0.5, 0.0)
        odom_tab.add_child(label_display_control)

        btn_start_view = gui.Button("Set to starting position")
        btn_start_view.vertical_padding_em = 0
        btn_start_view.horizontal_padding_em = 0
        btn_start_view.set_on_clicked(self.set_start_view) 
        odom_tab.add_child(btn_start_view)

        btn_top_view = gui.Button("Set to top position")
        btn_top_view.vertical_padding_em = 0
        btn_top_view.horizontal_padding_em = 0
        btn_top_view.set_on_clicked(self.set_top_view) 
        odom_tab.add_child(btn_top_view)
        
        self.cb_trace = gui.Checkbox("Enable tracing view")
        self.cb_trace.set_on_checked(self._on_cb_tracing)
        odom_tab.add_child(self.cb_trace)

        tabs.add_tab("Odometry", odom_tab)
        self.collapse.add_child(tabs)
        
        # video tab
        video_tab = gui.Vert(0.5 * em, gui.Margins(8,8,8,8))

        label_camera_control = gui.Label("Streaming control")
        label_camera_control.text_color = gui.Color(1.0, 0.5, 0.0)
        video_tab.add_child(label_camera_control)
        
        self.cb_cama = gui.Checkbox("Stream cama")
        # self.cb_cama.checked = True
        video_tab.add_child(self.cb_cama)

        self.cb_camb = gui.Checkbox("Stream camb")
        # self.cb_camb.checked = True
        video_tab.add_child(self.cb_camb)

        self.cb_camc = gui.Checkbox("Stream camc")
        # self.cb_camc.checked = True
        video_tab.add_child(self.cb_camc)

        self.cb_camd = gui.Checkbox("Stream camd")
        self.cb_camd.checked = True
        video_tab.add_child(self.cb_camd)
        
        label_display_control = gui.Label("Display control")
        label_display_control.text_color = gui.Color(1.0, 0.5, 0.0)
        video_tab.add_child(label_display_control)
        
        tedit_expMax = gui.TextEdit()
        tedit_expMax.placeholder_text = "Enter the max exposure"
        tedit_expMax.set_on_value_changed(self._tedit_expMax)
        video_tab.add_child(tedit_expMax)

        tedit_sensIsoMax = gui.TextEdit()
        tedit_sensIsoMax.placeholder_text = "Enter the max exposure"
        tedit_sensIsoMax.set_on_value_changed(self._tedit_sensIsoMax)
        video_tab.add_child(tedit_sensIsoMax)
        
        switch_expTime = gui.ToggleSwitch("Display expTime")
        switch_expTime.set_on_clicked(self._on_switch_expTime)
        video_tab.add_child(switch_expTime)
        
        switch_sensIso = gui.ToggleSwitch("Display sensIso")
        switch_sensIso.set_on_clicked(self._on_switch_sensIso)
        video_tab.add_child(switch_sensIso)

        switch_latencyDevice = gui.ToggleSwitch("Display latencyDevice")
        switch_latencyDevice.set_on_clicked(self._on_switch_latencyDevice)
        video_tab.add_child(switch_latencyDevice)

        switch_latencyHost = gui.ToggleSwitch("Display latencyHost")
        switch_latencyHost.set_on_clicked(self._on_switch_latencyHost)
        video_tab.add_child(switch_latencyHost)

        switch_tagDebug = gui.ToggleSwitch("Display tag debug image")
        switch_tagDebug.set_on_clicked(self._on_switch_tagDebug)
        video_tab.add_child(switch_tagDebug)
            
        label_info = gui.Label("Streaming mode")
        label_info.text_color = gui.Color(1.0, 0.5, 0.0)
        video_tab.add_child(label_info)

        self.synced_label = gui.Label("")
        video_tab.add_child(self.synced_label)

        tabs.add_tab("Video", video_tab)

        # Stats
        self.odom_stats_collapse = gui.CollapsableVert("Odometry stats", em, 
                                gui.Margins(em, 0, 0, 0))

        stats_tab = gui.Vert(0.5 * em, gui.Margins(8,8,8,8))
        self.odom_stats_collapse.add_child(stats_tab)

        stats_label = gui.Label("Aggregate statistics")
        stats_label.text_color = gui.Color(1.0, 0.5, 0.0)
        stats_tab.add_child(stats_label)
        
        self.first_odom = True
        self.init_odom_ts = 0
        self.last_odom_ts = 0
        self.total_dist = 0
        self.total_angle_rotation = 0
        self.elapsed_time = 0
        self.prev_drone_pose = None
        self.avg_exp = {}
        self.avg_gain = {}
        
        self.total_dist_label = gui.Label("Total distance")
        self.total_dist_label.font_id = self.font_id_large
        stats_tab.add_child(self.total_dist_label)

        self.average_speed_label = gui.Label("Avg speed")
        self.average_speed_label.font_id = self.font_id_large
        stats_tab.add_child(self.average_speed_label)

        self.average_ang_speed_label = gui.Label("Avg ang speed")
        self.average_ang_speed_label.font_id = self.font_id_large
        stats_tab.add_child(self.average_ang_speed_label)

        self.average_exp = gui.Label("Avg exp")
        self.average_exp.font_id = self.font_id_large
        stats_tab.add_child(self.average_exp)

        self.average_gain = gui.Label("Avg gain")
        self.average_gain.font_id = self.font_id_large
        stats_tab.add_child(self.average_gain)

        # message show
        label_info = gui.Label("Vio Information")
        label_info.text_color = gui.Color(1.0, 0.5, 0.0)
        self.collapse.add_child(label_info)

        self.proxy_vio = gui.WidgetProxy()
        self.collapse.add_child(self.proxy_vio)

        label_info = gui.Label("Camera Information")
        label_info.text_color = gui.Color(1.0, 0.5, 0.0)
        self.collapse.add_child(label_info)

        self.proxy_1 = gui.WidgetProxy()
        self.collapse.add_child(self.proxy_1)
        
        self.proxy_2 = gui.WidgetProxy()
        self.collapse.add_child(self.proxy_2)

        self.proxy_3 = gui.WidgetProxy()
        self.collapse.add_child(self.proxy_3)

        self.proxy_4 = gui.WidgetProxy()
        self.collapse.add_child(self.proxy_4)
        
        self.proxy_tag = gui.WidgetProxy()
        self.collapse.add_child(self.proxy_tag)

        # odom widget
        self.widget3d = gui.SceneWidget()
        self.widget3d.scene = rendering.Open3DScene(self.window.renderer)
        self.widget3d.scene.set_lighting(o3d.visualization.rendering.Open3DScene.LightingProfile.NO_SHADOWS , np.array([0, 0, 1]).astype(np.float32))        
        self.widget3d.scene.show_axes(True)
        self.widget3d.scene.set_background([2, 1, 2, 1])

        lit = rendering.MaterialRecord()
        lit.shader = "defaultLit"

        line_mat = rendering.MaterialRecord()
        line_mat.shader = "unlitLine"
        line_mat.line_width = 2

        # add land survey model
        self.land_survey = o3d.io.read_triangle_mesh("./model_data/10239-Tag-Survey-report v7.obj", True, True)
        self.land_survey.compute_vertex_normals()
        self.land_survey.rotate(R =[[0, 1, 0],
                                    [-1,  0, 0],
                                    [0,  0, 1]], center = [0, 0, 0])
        self.land_survey.paint_uniform_color([0.0, 0.0, 0.0])
        self.widget3d.scene.add_geometry("land_survey", self.land_survey, lit)
        self.widget3d.scene.show_geometry("land_survey", False)

        # add floor
        self.floor_width = 10
        self.floor_height = 10
        floor = o3d.geometry.TriangleMesh.create_box(width=self.floor_width, height=self.floor_height, depth=0.01)
        floor.compute_vertex_normals()
        floor.translate([0, 0, self.land_survey.get_min_bound()[2]], relative=False)  
        floor.paint_uniform_color([0.5, 0.5, 0.5])
        self.widget3d.scene.add_geometry("floor", floor, lit)

        floor_grid = create_grid_mesh(self.floor_width, self.floor_height, 5)
        floor_grid.translate([floor.get_center()[0], floor.get_center()[1], floor.get_max_bound()[2]], relative=False)  
        floor_grid.paint_uniform_color([0.1, 0.1, 0.1])        
        self.widget3d.scene.add_geometry("floor_grid", floor_grid, line_mat)

        # add drone
        self.drone = o3d.geometry.TriangleMesh.create_coordinate_frame()
        self.drone.compute_vertex_normals()
        self.drone.translate([0, 0, 0], relative=False)
        self.widget3d.scene.add_geometry("drone", self.drone, lit)

        self.path_line_list = []
        self.cleaning_now = False

        # add camera
        self.bounds = self.widget3d.scene.bounding_box
        self.drone_bound = self.drone. get_axis_aligned_bounding_box()
        
        self.set_start_view()

        # main panel
        self.panel_main = gui.Vert(0.5 * em, gui.Margins(margin))

        # top panel
        self.panel_top = gui.Horiz(0.5 * em, gui.Margins(margin))
        self.rgb_widget_1 = gui.ImageWidget(o3d.geometry.Image(np.zeros((320,520,1), dtype=np.uint8)))
        self.rgb_widget_2 = gui.ImageWidget(o3d.geometry.Image(np.zeros((320,520,1), dtype=np.uint8)))
        self.panel_top.add_child(self.rgb_widget_1)
        self.panel_top.add_child(self.rgb_widget_2)
        self.panel_main.add_child(self.panel_top) 

        # bottom panel
        self.panel_bottom = gui.Horiz(0.5 * em, gui.Margins(margin))
        self.rgb_widget_3 = gui.ImageWidget(o3d.geometry.Image(np.zeros((320,520,1), dtype=np.uint8)))
        self.rgb_widget_4 = gui.ImageWidget(o3d.geometry.Image(np.zeros((320,520,1), dtype=np.uint8)))
        self.panel_bottom.add_child(self.rgb_widget_3)
        self.panel_bottom.add_child(self.rgb_widget_4)
        self.panel_main.add_child(self.panel_bottom) 

        
        self.window.add_child(self.collapse)
        self.window.add_child(self.odom_stats_collapse)
        self.window.add_child(self.panel_main)
        self.window.add_child(self.widget3d)

        self.init_video = 0 
        self.init_odom = 0

        # tags
        # to replace with programmatic mesh generation
        cube_mesh_model = o3d.io.read_triangle_model("assets/april.obj")                   
        self.cube_mesh = cube_mesh_model.meshes[1].mesh
        self.cube_material = cube_mesh_model.materials[1]

        bbox = self.cube_mesh.get_axis_aligned_bounding_box()
        model_mesh_scale = bbox.get_max_bound()[1] - bbox.get_min_bound()[1]
        self.cube_mesh.scale(1 / model_mesh_scale, center=[0,0,0])
        
        self.lit = lit


    def destroy_tag_labels(self, sub):
        for label in sub.tag_labels:
            self.widget3d.remove_3d_label(label)
        sub.tag_labels = set()

    def add_tags(self, stream, tags, sub):
        if (not tags is None):
            camera_frame_msg = tags.cameraExtrinsic.bodyFrame
            body_T_camera = self._capnp_se3_to_sophus_se3(camera_frame_msg)

            tag_names = set([f"tag_{tag.id} ({stream})" for tag in tags.tags])
            old_not_new = sub.tag_geometries - tag_names
            new_and_old = sub.tag_geometries & tag_names

            for tag in tags.tags:
                tag_name = f"tag_{tag.id} ({stream})"
                tag_axis_name = f"tagaxis_{tag.id} ({stream})"
                tag_camera_pose_msg = tag.poseInCameraFrame
                
                camera_T_tag = self._capnp_se3_to_sophus_se3(tag_camera_pose_msg)

                if sub.uses_vio and sub.vio is not None:
                    quat = [sub.vio.pose.orientation.x,
                            sub.vio.pose.orientation.y,
                            sub.vio.pose.orientation.z, 
                            sub.vio.pose.orientation.w]
                    quat_norm = np.linalg.norm(quat) # will be 0 if no vio msg
                else:
                    quat_norm = 0

                if quat_norm > 0:
                    vio_t = np.array([sub.vio.pose.position.x,
                                  sub.vio.pose.position.y,
                                  sub.vio.pose.position.z], dtype=np.float64)
                    vio_r = R.from_quat(quat).as_matrix()
                    origin_T_body = sp.SE3(vio_r, vio_t)
                    tag_pose = origin_T_body * body_T_camera * camera_T_tag
                else:
                    tag_pose = body_T_camera * camera_T_tag                

                # place axis at bottom right corner of tag to match debug image
                axis_translate = self.make_se3(0,
                                               -tag.tagSize / 2,
                                               -tag.tagSize / 2,
                                               1, 0, 0, 0)
                axis_pose = tag_pose * axis_translate

                if tag_name in new_and_old:
                    # tag already visible in scene
                    # update pose
                    self.widget3d.scene.set_geometry_transform(tag_name, tag_pose.matrix())
                    self.widget3d.scene.set_geometry_transform(tag_axis_name, axis_pose.matrix())
                    sub.tag_labels.add(self.widget3d.add_3d_label(tag_pose.translation(), tag_name))
                elif tag_name in old_not_new:
                    # can't happen
                    continue
                elif self.widget3d.scene.has_geometry(tag_name):
                    # tag has been added, but not visible
                    # let's make tag visible
                    self.widget3d.scene.show_geometry(tag_name, True)
                    self.widget3d.scene.show_geometry(tag_axis_name, True)
                    
                    self.widget3d.scene.set_geometry_transform(tag_name, tag_pose.matrix())
                    self.widget3d.scene.set_geometry_transform(tag_axis_name, axis_pose.matrix())
                    sub.tag_labels.add(self.widget3d.add_3d_label(tag_pose.translation(), tag_name))
                    
                    sub.tag_geometries.add(tag_name)
                else:
                    # tag has not been added
                    # let's add it
                    axis_geometry = o3d.geometry.TriangleMesh.create_coordinate_frame(tag.tagSize)
                    axis_geometry.compute_vertex_normals()
                    
                    cube_mesh = copy.deepcopy(self.cube_mesh)
                    cube_mesh.scale(tag.tagSize, center=[0,0,0])
                    cube_mesh.compute_vertex_normals()

                    # add to scene
                    self.widget3d.scene.add_geometry(tag_name, cube_mesh, self.cube_material)
                    self.widget3d.scene.add_geometry(tag_axis_name, axis_geometry, self.lit)
                    
                    self.widget3d.scene.set_geometry_transform(tag_name, tag_pose.matrix())
                    self.widget3d.scene.set_geometry_transform(tag_axis_name, axis_pose.matrix())
                    sub.tag_labels.add(self.widget3d.add_3d_label(tag_pose.translation(), tag_name))
                    
                    sub.tag_geometries.add(tag_name)

            for tag_name in old_not_new:
                # tags are no longer detected
                # make tag invisible
                tag_axis_name = f"tagaxis_{tag_name.split('_')[1]}"
                self.widget3d.scene.show_geometry(tag_name, False)
                self.widget3d.scene.show_geometry(tag_axis_name, False)
                sub.tag_geometries.remove(tag_name)

    def _capnp_se3_to_sophus_se3(self, capnp_se3):
        w = capnp_se3.orientation.w
        x = capnp_se3.orientation.x
        y = capnp_se3.orientation.y
        z = capnp_se3.orientation.z

        return self.make_se3(capnp_se3.position.x,
                             capnp_se3.position.y,
                             capnp_se3.position.z,
                             w, x, y, z)
    
    def make_se3(self, x, y, z, qw, qx, qy, qz):
        t = np.array([x, y, z], dtype=np.float64)
        r = R.from_quat([qx, qy, qz, qw]).as_matrix()
        return sp.SE3(r, t)

    def _on_layout_video(self, layout_context):

        contentRect = self.window.content_rect
        panel_main_width = 1040
        widget3d_width = 1040
        self.collapse.frame = gui.Rect(contentRect.x, 
                                contentRect.y,
                                260,
                                contentRect.height / 2)
        
        self.odom_stats_collapse.frame = gui.Rect(contentRect.x,
                                self.collapse.frame.get_bottom(),
                                260,
                                contentRect.height / 2)      

        self.panel_main.frame = gui.Rect(self.collapse.frame.get_right(), 
                                contentRect.y,
                                panel_main_width,
                                contentRect.height)

        self.widget3d.frame = gui.Rect(self.panel_main.frame.get_right(),
                        contentRect.y,
                        widget3d_width,
                        contentRect.height)
    
    def _on_layout_odom(self, layout_context):

        contentRect = self.window.content_rect
        panel_main_width = 1040
        widget3d_width = 1040
        self.collapse.frame = gui.Rect(contentRect.x, 
                                contentRect.y,
                                260,
                                contentRect.height / 2)
        
        self.odom_stats_collapse.frame = gui.Rect(contentRect.x,
                        self.collapse.frame.get_bottom(),
                        260,
                        contentRect.height / 2)   

        self.widget3d.frame = gui.Rect(self.collapse.frame.get_right(),
                                contentRect.y,
                                widget3d_width,
                                contentRect.height)

        self.panel_main.frame = gui.Rect(self.widget3d.frame.get_right(), 
                                contentRect.y,
                                panel_main_width,
                                contentRect.height)

    def _on_close(self):
        self.is_done = True
        return True  # False would cancel the close
    
    def _on_menu_quit(self):
        gui.Application.instance.quit()

    def _select_rb_display(self, idx):
        if self.rb_display.selected_value == "Odometry":
            self.window.set_on_layout(self._on_layout_odom)
        elif self.rb_display.selected_value == "Video":
            self.window.set_on_layout(self._on_layout_video)
        else:
            print("error in switching display")

    def _on_cb_grid(self, is_checked):
        if is_checked:
            self.widget3d.scene.show_geometry("floor_grid", True)
        else:
            self.widget3d.scene.show_geometry("floor_grid", False)

    def _on_cb_land(self, is_checked):
        if is_checked:
            self.widget3d.scene.show_geometry("land_survey", True)
        else:
            self.widget3d.scene.show_geometry("land_survey", False)
      
    def set_start_view(self):
        self.widget3d.setup_camera(60.0, self.bounds, self.bounds.get_center())   
        camera_pos = np.array([0, 0, 50], dtype=np.float32)
        target = np.array([0, 0, 0], dtype=np.float32)
        up = np.array([1, 0, 0], dtype=np.float32)
        self.widget3d.look_at(target, camera_pos, up)

    def set_top_view(self):
        self.widget3d.setup_camera(60.0, self.bounds, self.bounds.get_center())   
        camera_pos = np.array([self.land_survey.get_center()[0], self.land_survey.get_center()[1], 70], dtype=np.float32)
        target = np.array([self.land_survey.get_center()[0], self.land_survey.get_center()[1], 0], dtype=np.float32)
        up = np.array([1, 0, 0], dtype=np.float32)
        self.widget3d.look_at(target, camera_pos, up)
    

    def _on_cb_tracing(self, is_checked):
        if is_checked:
            pass
        else:
            self.set_start_view()

    def _btn_land_clk(self):

        self.land_current_degree += self.land_rotate_interval

        theta = np.deg2rad(self.land_current_degree)
        # Construct the rotation matrix
        rotation = np.array([[np.cos(theta), np.sin(theta), 0, 0],
                    [-np.sin(theta), np.cos(theta), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]], dtype=np.float64)

        self.widget3d.scene.set_geometry_transform("land_survey", rotation)


    def _btn_land_anti(self):
        
        self.land_current_degree -= self.land_rotate_interval

        theta = np.deg2rad(self.land_current_degree)
        # Construct the rotation matrix
        rotation = np.array([[np.cos(theta), np.sin(theta), 0, 0],
                    [-np.sin(theta), np.cos(theta), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]], dtype=np.float64)
        self.widget3d.scene.set_geometry_transform("land_survey", rotation)


    def _btn_clear(self):
        self.cleaning_now = True

        for line_name in self.path_line_list:
            self.widget3d.scene.remove_geometry(line_name)
        
        self.cleaning_now = False
    
    def _btn_st_pt(self):
        if not self.widget3d.scene.has_geometry("starting point"):
            x_coor = self.widget3d.scene.get_geometry_transform("drone")[0][3]
            y_coor = self.widget3d.scene.get_geometry_transform("drone")[1][3]
            z_coor = self.widget3d.scene.get_geometry_transform("drone")[2][3]
            
            lit = rendering.MaterialRecord()
            lit.shader = "defaultLit"

            radius = 0.2
            center = np.array([x_coor, y_coor, z_coor])
            ball_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
            ball_mesh.compute_vertex_normals()
            ball_mesh.translate(center)
            ball_mesh.paint_uniform_color([0.0, 0.0, 0.9])
            self.widget3d.scene.add_geometry("starting point",ball_mesh,lit)

    def _btn_st_clr(self):
        if self.widget3d.scene.has_geometry("starting point"):
            self.widget3d.scene.remove_geometry("starting point")

    def _btn_ed_pt(self):
        if not self.widget3d.scene.has_geometry("end point"):
            x_coor = self.widget3d.scene.get_geometry_transform("drone")[0][3]
            y_coor = self.widget3d.scene.get_geometry_transform("drone")[1][3]
            z_coor = self.widget3d.scene.get_geometry_transform("drone")[2][3]
            
            lit = rendering.MaterialRecord()
            lit.shader = "defaultLit"

            radius = 0.2
            center = np.array([x_coor, y_coor, z_coor])
            ball_mesh = o3d.geometry.TriangleMesh.create_sphere(radius=radius)
            ball_mesh.compute_vertex_normals()
            ball_mesh.translate(center)
            ball_mesh.paint_uniform_color([0.0, 0.9, 0.0])
            self.widget3d.scene.add_geometry("end point",ball_mesh,lit)

    def _btn_ed_clr(self):
        if self.widget3d.scene.has_geometry("end point"):
            self.widget3d.scene.remove_geometry("end point")
    
    def _tedit_expMax(self, new_text):
        if is_numeric(new_text):            
            self.expMax = int(new_text)
        else:
            self.expMax = 12000

    def _tedit_sensIsoMax(self, new_text):
        if is_numeric(new_text):
            self.sensIsoMax = int(new_text)
        else:
            self.sensIsoMax = 800
            
    def _on_switch_expTime(self, is_on):
        if is_on:
            self.expTime_display_flag = True
        else:
            self.expTime_display_flag = False
    
    def _on_switch_sensIso(self, is_on):
        if is_on:
            self.sensIso_display_flag = True
        else:
            self.sensIso_display_flag = False
    
    def _on_switch_latencyDevice(self, is_on):
        if is_on:
            self.latencyDevice_display_flag = True
        else:
            self.latencyDevice_display_flag = False
    
    def _on_switch_latencyHost(self, is_on):
        if is_on:
            self.latencyHost_display_flag = True
        else:
            self.latencyHost_display_flag = False
    
    def _on_switch_tagDebug(self, is_on):
        if is_on:
            self.tagDebug_display_flag = True
        else:
            self.tagDebug_display_flag = False


def read_img(window):


    # PRINT ECAL VERSION AND DATE
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
    
    # INITIALIZE eCAL API
    ecal_core.initialize(sys.argv, "calibration_dataset_recorder")
    
    # SET PROCESS STATE
    ecal_core.set_process_state(1, 1, "I feel good")

    # set up tag detection subscribers

    vio_topic = "S0/vio_odom" if flag_dict['vio_status'] else None
    tag_sub = { "cama": TagDetectionsSubscriber("S0/tags/cama"),
                "camb": TagDetectionsSubscriber("S0/tags/camb"),
                "camc": TagDetectionsSubscriber("S0/tags/camc"),
                "camd": TagDetectionsSubscriber("S0/tags/camd"), }
    vio_sub = None

    # set up vio subscriber
    if (flag_dict['vio_status']):
        vio_sub = VioSubscriber("S0/vio_odom")
        tag_sub["cama"].add_vio_sub(vio_sub, "S0/vio_odom")
        tag_sub["camb"].add_vio_sub(vio_sub, "S0/vio_odom")
        tag_sub["camc"].add_vio_sub(vio_sub, "S0/vio_odom")
        tag_sub["camd"].add_vio_sub(vio_sub, "S0/vio_odom")
    else:
        window.proxy_vio.set_widget(gui.Label("vio is not on"))
    
    # set up image subscriber
    if (flag_dict['synced_status']):
        window.synced_label.text = "Synced images"
        synced_recorder = SyncedImageSubscriber(image_types, 
                                                image_topics, 
                                                image_typeclasses)
        synced_recorder.rolling = True
    else:
        window.synced_label.text = "Asynced images"
        asynced_recorder = SyncedImageSubscriber(image_types, 
                                                 image_topics, 
                                                 image_typeclasses, 
                                                 False)
        asynced_recorder.rolling = True

    def update_frame(imageName,img_ndarray):
        if(('cama' in imageName) and window.cb_cama.checked):
            window.rgb_widget_1.update_image(o3d.geometry.Image(img_ndarray))

        if(('camb' in imageName) and window.cb_camb.checked):
            window.rgb_widget_2.update_image(o3d.geometry.Image(img_ndarray))
        
        if(('camc' in imageName) and window.cb_camc.checked):
            window.rgb_widget_3.update_image(o3d.geometry.Image(img_ndarray))
        
        if(('camd' in imageName) and window.cb_camd.checked):
            window.rgb_widget_4.update_image(o3d.geometry.Image(img_ndarray))


    def update_proxy(proxy,display_msg):
        
        widget = proxy.get_widget()
       
        if widget is None:
            label = gui.Label(display_msg)
            proxy.set_widget(label)
        else:
            widget.text = display_msg

        window.window.set_needs_layout() 
        
    # odom path material    
    mat = rendering.MaterialRecord()
    mat.shader = "unlitLine"
    mat.line_width = 5
    # mat.sRGB_color = [0.0, 1.0, 0.0]
    
    line_index = 0

    file_last_time = time.monotonic()

    if flag_dict['csv_status']:
        # clear the csv file at init    
        current_time = datetime.now()

        with open(f"./odom_data/position_{current_time}.csv", "w") as csvfile:
            csvfile.truncate()

        with open(f"./odom_data/orientation_{current_time}.csv", "w") as csvfile:
            csvfile.truncate()

    while ecal_core.ok():
        # synced mode
        if (flag_dict['synced_status']):
            ecal_images = synced_recorder.pop_sync_queue()
        else:
            ecal_images = asynced_recorder.pop_sync_queue()

        for imageName in ecal_images:

            imageMsg = ecal_images[imageName]
            if imageName not in window.avg_exp and imageName not in window.avg_gain:
                window.avg_exp[imageName] = (0, 0)
                window.avg_gain[imageName] = (0, 0)
            
            window.avg_exp[imageName] = (window.avg_exp[imageName][0] + (imageMsg.exposureUSec - window.avg_exp[imageName][0]) / (window.avg_exp[imageName][1] + 1),
                                         window.avg_exp[imageName][1] + 1)
            window.avg_gain[imageName] = (window.avg_gain[imageName][0] + (imageMsg.gain - window.avg_gain[imageName][0]) / (window.avg_gain[imageName][1] + 1),
                                          window.avg_gain[imageName][1] + 1)

            expTime_display = f"expTime = {imageMsg.exposureUSec}" 
            sensIso_display = f"sensIso = {imageMsg.gain}" 
            latencyDevice_display = f"device latency = {imageMsg.header.latencyDevice / 1e6 :.2f} ms" 
            host_latency = time.monotonic() *1e9 - imageMsg.header.stamp 
            latencyHost_display = f"host latency = {host_latency / 1e6 :.2f} ms" 

            all_display = imageName + '\n' + expTime_display + '\n' + sensIso_display + '\n' + latencyDevice_display + '\n' + latencyHost_display 

            dim = (512, 320) #width height

            camName = imageName.split("/")[-1]
            if window.tagDebug_display_flag and not tag_sub[camName].tags is None:
                # hijack image stream
                imageMsg = tag_sub[camName].tags.image

            if imageMsg.encoding == "mono8":
                img_ndarray = np.frombuffer(imageMsg.data, dtype=np.uint8)
                img_ndarray = img_ndarray.reshape((imageMsg.height, imageMsg.width, 1))             
                img_ndarray = cv2.resize(img_ndarray, dim, interpolation = cv2.INTER_NEAREST)

                #convert numpy array to 3 channel
                img_ndarray = np.repeat(img_ndarray.reshape(dim[1], dim[0], 1), 3, axis=2)
            elif imageMsg.encoding == "yuv420":
                img_ndarray = np.frombuffer(imageMsg.data, dtype=np.uint8)
                img_ndarray = img_ndarray.reshape((imageMsg.height * 3 // 2, imageMsg.width, 1))

                img_ndarray = cv2.cvtColor(img_ndarray, cv2.COLOR_YUV2RGB_IYUV)
                img_ndarray = cv2.resize(img_ndarray, dim, interpolation = cv2.INTER_NEAREST)
            elif imageMsg.encoding == "bgr8":
                img_ndarray = np.frombuffer(imageMsg.data, dtype=np.uint8)
                img_ndarray = img_ndarray.reshape((imageMsg.height, imageMsg.width, 3))
                
                # bgr to rgb
                img_ndarray = img_ndarray[:,:,::-1]
                img_ndarray = cv2.resize(img_ndarray, dim, interpolation = cv2.INTER_NEAREST)
            else:
                raise RuntimeError("unknown encoding: " + imageMsg.encoding)

            img_ndarray = add_ui_on_ndarray(img_ndarray, imageMsg.exposureUSec, imageMsg.gain, latencyDevice_display, latencyHost_display,
                                            window.expMax, window.sensIsoMax,
                                            window.expTime_display_flag,
                                            window.sensIso_display_flag,
                                            window.latencyDevice_display_flag,
                                            window.latencyHost_display_flag)
            

            if not window.is_done:

                update_frame(imageName, img_ndarray)
                if(('cama' in imageName) and window.cb_cama.checked):
                    update_proxy(window.proxy_1,all_display)
                if(('camb' in imageName) and window.cb_camb.checked):
                    update_proxy(window.proxy_2,all_display)
                if(('camc' in imageName) and window.cb_camc.checked):
                    update_proxy(window.proxy_3,all_display)
                if(('camd' in imageName) and window.cb_camd.checked):
                    update_proxy(window.proxy_4,all_display)
                
        if not window.is_done:
            for stream, sub in tag_sub.items():
                if sub.new_data:
                    window.destroy_tag_labels(sub)
                    window.add_tags(stream, sub.tags, sub)
                    sub.new_data = False

            if flag_dict['vio_status']:
                update_proxy(window.proxy_vio, vio_sub.vio_msg)

                prev_x_coor = window.widget3d.scene.get_geometry_transform("drone")[0][3]
                prev_y_coor = window.widget3d.scene.get_geometry_transform("drone")[1][3]
                prev_z_coor = window.widget3d.scene.get_geometry_transform("drone")[2][3]
                
                x = vio_sub.orientation_x
                y = vio_sub.orientation_y
                z = vio_sub.orientation_z
                w = vio_sub.orientation_w  
                quat = [x, y, z, w]

                if np.linalg.norm(quat) > 0: # check valid quaternion
                    t = np.array([vio_sub.position_x,
                                  vio_sub.position_y,
                                  vio_sub.position_z], dtype=np.float64)
                    r = R.from_quat(quat).as_matrix()
                    new_drone_pose = sp.SE3(r, t)

                    if window.first_odom:
                        window.first_odom = False
                        window.init_odom_ts = vio_sub.header.stamp

                    if window.prev_drone_pose is not None:
                        # compute axis angle of transfomation
                        drone_transform = new_drone_pose.matrix() @ window.prev_drone_pose.inverse().matrix()
                        transform_rotation = drone_transform[:3, :3]
                        quat = R.from_matrix(transform_rotation).as_quat()
                        axis_angle = 2 * np.arccos(quat[3])
                        window.total_angle_rotation += axis_angle

                    window.prev_drone_pose = new_drone_pose
                    
                    # set axis pose
                    window.widget3d.scene.set_geometry_transform("drone", new_drone_pose.matrix())

                if (vio_sub.position_x > window.floor_width / 2 or vio_sub.position_y > window.floor_height / 2):
                    line_mat = rendering.MaterialRecord()
                    line_mat.shader = "unlitLine"
                    line_mat.line_width = 2

                    window.widget3d.scene.remove_geometry("floor_grid")
                    window.widget3d.scene.remove_geometry("floor")

                    width = 2 * max(vio_sub.position_x, window.floor_width)
                    height = 2 * max(vio_sub.position_y, window.floor_height)

                    window.floor_width = max(width, height)
                    window.floor_height = max(width, height)

                    floor = o3d.geometry.TriangleMesh.create_box(width=window.floor_width, height=window.floor_height, depth=0.01)
                    floor.compute_vertex_normals()
                    floor.translate([0, 0, window.land_survey.get_min_bound()[2]], relative=False)  
                    floor.paint_uniform_color([0.5, 0.5, 0.5])
                    window.widget3d.scene.add_geometry("floor", floor, window.lit)

                    floor_grid = create_grid_mesh(window.floor_width, window.floor_height, 5)
                    floor_grid.translate([floor.get_center()[0], floor.get_center()[1], floor.get_max_bound()[2]], relative=False)  
                    floor_grid.paint_uniform_color([0.1, 0.1, 0.1])        
                    window.widget3d.scene.add_geometry("floor_grid", floor_grid, line_mat)

                # store the imu to the csv file
                if((time.monotonic() - file_last_time) > 1) and flag_dict['csv_status']:
                    file_last_time = time.monotonic()
                    f_position = open(f"./odom_data/position_{current_time}.csv", "a")
                    f_orientation = open(f"./odom_data/orientation_{current_time}.csv", "a")
                    f_position.write(f"{file_last_time}, {vio_sub.position_x}, {vio_sub.position_y}, {vio_sub.position_z} \n")
                    f_orientation.write(f"{file_last_time}, {x}, {y}, {z}, {w}\n")
                    f_position.close()
                    f_orientation.close()


                # print("trans matrix", drone_transform)
                current_x_coor = window.widget3d.scene.get_geometry_transform("drone")[0][3]
                current_y_coor = window.widget3d.scene.get_geometry_transform("drone")[1][3]
                current_z_coor = window.widget3d.scene.get_geometry_transform("drone")[2][3]

                # path sub line
                vertices = np.array([
                    [prev_x_coor, prev_y_coor, prev_z_coor],  # previous coordinate
                    [current_x_coor, current_y_coor, current_z_coor], # updated coordinate
                ], dtype = np.float64)
                norm = np.linalg.norm(vertices[1] - vertices[0])
                window.total_dist += norm
                edge = np.array([[0, 1],], dtype = np.int32)

                odom_jump_detected = abs(vio_sub.ts - window.last_odom_ts) > JUMP_THRESHOLD
                window.last_odom_ts = vio_sub.ts
                window.elapsed_time = vio_sub.ts - window.init_odom_ts
                if window.elapsed_time > 0:
                    print()
                    window.total_dist_label.text = f"total dist: {window.total_dist:.2f} m"
                    window.average_speed_label.text = f"avg speed = {window.total_dist / window.elapsed_time * 1e9} m/s"
                    window.average_ang_speed_label.text = f"avg angspeed = {window.total_angle_rotation / window.elapsed_time * 180 / np.pi * 1e9} deg/s"
                    window.average_exp.text = f"avg exp = {window.avg_exp}"
                    window.average_gain.text = f"avg gain = {window.avg_gain}"

                # draw the sub line
                if not np.array_equal(vertices[0], vertices[1]):
                    line_path = o3d.geometry.LineSet()
                    line_path.points = o3d.utility.Vector3dVector(vertices)
                    line_path.lines = o3d.utility.Vector2iVector(edge)
                    if odom_jump_detected:
                        line_path.colors = o3d.utility.Vector3dVector([(0, 0, 1)])
                    else:
                        line_path.colors = o3d.utility.Vector3dVector([(1, 0, 0)])
                    
                    if not window.cleaning_now:
                        line_name = "line_" + str(line_index)
                        window.path_line_list.append(line_name)
                        window.widget3d.scene.add_geometry(line_name, line_path, mat, False)
                        line_index +=1
            else:
                pass
            
            if window.cb_trace.checked:

                window.widget3d.setup_camera(60.0, window.bounds, window.bounds.get_center())   
                camera_pos = np.array([current_x_coor, current_y_coor, current_z_coor+15], dtype=np.float32)
                target = np.array([current_x_coor, current_y_coor, current_z_coor], dtype=np.float32)
                up = np.array([1, 0, 0], dtype=np.float32)
                window.widget3d.look_at(target, camera_pos, up)

        window.window.post_redraw()

    # finalize eCAL API
    ecal_core.finalize()



def main(): 
    
    # choose camera app
    choose_app = gui.Application.instance
    choose_app.initialize()

    choose_win = ChooseWindow()
    
    choose_app.run()    

    # initialised main app
    app = gui.Application.instance
    app.initialize()
    
    win = VideoWindow()
    
    # NEW THREAD FOR IMAGE READING    
    img_reading_thread= threading.Thread(target=read_img,args=(win,))
    img_reading_thread.start()
    
    time.sleep(3)
    
    # run main app
    app.run()
    
        

if __name__ == "__main__":
    main() 