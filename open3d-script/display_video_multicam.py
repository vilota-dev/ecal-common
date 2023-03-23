#!/usr/bin/env python3

import sys
import time
from time import monotonic
import threading

import capnp
import numpy as np
import cv2
import platform

import ecal.core.core as ecal_core
from byte_subscriber import ByteSubscriber


capnp.add_import_hook(['../src/capnp'])

import odometry3d_capnp as eCALOdometry3d
import image_capnp as eCALImage
import cameracontrol_capnp as eCALCameraControl
import open3d as o3d
import open3d.visualization.gui as gui

from PIL import Image

from utils import SyncedImageSubscriber, AsyncedImageSubscriber, VioSubscriber, add_ui_on_ndarray

isMacOS = (platform.system() == "Darwin")

image_topics = []
flag_dict = {}
flag_dict['vio_status'] = False
flag_dict['synced_status'] = False
flag_dict['thumbnail_status'] = False


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
        self.panel_main.add_child(self.cb_camb)

        self.cb_camc = gui.Checkbox("cam_c")
        self.panel_main.add_child(self.cb_camc)

        self.cb_camd = gui.Checkbox("cam_d")
        self.panel_main.add_child(self.cb_camd)

        self.label_description = gui.Label("Please choose the specific features.")
        self.label_description.text_color = gui.Color(1.0, 0.5, 0.0)
        self.panel_main.add_child(self.label_description)

        self.cb_vio = gui.Checkbox("vio is on")
        self.panel_main.add_child(self.cb_vio)

        self.cb_synced = gui.Checkbox("use synced image (default asynced)")
        self.panel_main.add_child(self.cb_synced)

        self.cb_thumbnail = gui.Checkbox("use thumbnail image (default normal)")
        self.panel_main.add_child(self.cb_thumbnail)
        
        self.window.add_child(self.panel_main) 


        self.button_layout = gui.Horiz(0.5 * em, gui.Margins(margin))
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
        if self.cb_thumbnail.checked:
            flag_dict['thumbnail_status'] = True
            if self.cb_cama.checked:
                image_topics.append("S0/thumbnail/cama")
            if self.cb_camb.checked:
                image_topics.append("S0/thumbnail/camb") 
            if self.cb_camc.checked:
                image_topics.append("S0/thumbnail/camc")
            if self.cb_camd.checked:
                image_topics.append("S0/thumbnail/camd")
        else:
            if self.cb_cama.checked:
                image_topics.append("S0/cama")
            if self.cb_camb.checked:
                image_topics.append("S0/camb") 
            if self.cb_camc.checked:
                image_topics.append("S0/camc")
            if self.cb_camd.checked:
                image_topics.append("S0/camd")
        
        if self.cb_vio.checked:
            flag_dict['vio_status'] = True
        
        if self.cb_synced.checked:
            flag_dict['synced_status'] = True

        
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

        # CONFIGURE WINDOW
        self.window = gui.Application.instance.create_window(
            "Video", 1300, 800)
        self.window.set_on_layout(self._on_layout)
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
        self.collapse = gui.CollapsableVert("Widgets", 0.33 * em,
                                       gui.Margins(em, 0, 0, 0))
        

        self.label_camera_control = gui.Label("Streaming control")
        self.label_camera_control.text_color = gui.Color(1.0, 0.5, 0.0)
        self.collapse.add_child(self.label_camera_control)
        
        self.cb_cama = gui.Checkbox("Start streaming cama")
        self.cb_cama.checked = True
        self.collapse.add_child(self.cb_cama)

        self.cb_camb = gui.Checkbox("Start streaming camb")
        self.cb_camb.checked = True
        self.collapse.add_child(self.cb_camb)

        self.cb_camc = gui.Checkbox("Start streaming camc")
        self.cb_camc.checked = True
        self.collapse.add_child(self.cb_camc)

        self.cb_camd = gui.Checkbox("Start streaming camd")
        self.cb_camd.checked = True
        self.collapse.add_child(self.cb_camd)
        
        self.label_display_control = gui.Label("Display control")
        self.label_display_control.text_color = gui.Color(1.0, 0.5, 0.0)
        self.collapse.add_child(self.label_display_control)

        switch_expTime = gui.ToggleSwitch("Display expTime")
        switch_expTime.set_on_clicked(self._on_switch_expTime)
        self.collapse.add_child(switch_expTime)
        
        switch_sensIso = gui.ToggleSwitch("Display sensIso")
        switch_sensIso.set_on_clicked(self._on_switch_sensIso)
        self.collapse.add_child(switch_sensIso)

        switch_latencyDevice = gui.ToggleSwitch("Display latencyDevice")
        switch_latencyDevice.set_on_clicked(self._on_switch_latencyDevice)
        self.collapse.add_child(switch_latencyDevice)

        switch_latencyHost = gui.ToggleSwitch("Display latencyHost")
        switch_latencyHost.set_on_clicked(self._on_switch_latencyHost)
        self.collapse.add_child(switch_latencyHost)
            
        self.label_info = gui.Label("Streaming mode")
        self.label_info.text_color = gui.Color(1.0, 0.5, 0.0)
        self.collapse.add_child(self.label_info)

        self.synced_label = gui.Label("")
        self.collapse.add_child(self.synced_label)
        
        self.label_info = gui.Label("Vio Information")
        self.label_info.text_color = gui.Color(1.0, 0.5, 0.0)
        self.collapse.add_child(self.label_info)

        self.proxy_vio = gui.WidgetProxy()
        self.collapse.add_child(self.proxy_vio)

        self.label_info = gui.Label("Camera Information")
        self.label_info.text_color = gui.Color(1.0, 0.5, 0.0)
        self.collapse.add_child(self.label_info)

        self.proxy_1 = gui.WidgetProxy()
        self.collapse.add_child(self.proxy_1)
        
        self.proxy_2 = gui.WidgetProxy()
        self.collapse.add_child(self.proxy_2)

        self.proxy_3 = gui.WidgetProxy()
        self.collapse.add_child(self.proxy_3)

        self.proxy_4 = gui.WidgetProxy()
        self.collapse.add_child(self.proxy_4)
        
        self.window.add_child(self.collapse)
        

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

        self.window.add_child(self.panel_main) 



    def _on_layout(self, layout_context):
        contentRect = self.window.content_rect
        panel_main_width = 1040
        self.collapse.frame = gui.Rect(contentRect.x, 
                                contentRect.y,
                                contentRect.width - panel_main_width,
                                contentRect.height)

        self.panel_main.frame = gui.Rect(self.collapse.frame.get_right(), 
                                contentRect.y,
                                panel_main_width,
                                contentRect.height)
        
    def _on_close(self):
        self.is_done = True
        return True  # False would cancel the close
    
    def _on_menu_quit(self):
        gui.Application.instance.quit()

    
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
    




def read_img(window):


    # PRINT ECAL VERSION AND DATE
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
    
    # INITIALIZE eCAL API
    ecal_core.initialize(sys.argv, "calibration_dataset_recorder")
    
    # SET PROCESS STATE
    ecal_core.set_process_state(1, 1, "I feel good")

    # set up vio subscriber
    if (flag_dict['vio_status']):
        vio_sub = VioSubscriber("S0/vio_odom")
    else:
        window.proxy_vio.set_widget(gui.Label("vio is not on"))
    
    # set up image subscriber
    if (flag_dict['synced_status']):
        window.synced_label.text = "Synced images"
        synced_recorder = SyncedImageSubscriber(image_topics)
        synced_recorder.rolling = True
    else:
        window.synced_label.text = "Asynced images"
        asynced_recorder = AsyncedImageSubscriber(image_topics)
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
        


    while ecal_core.ok():

        # synced mode
        if (flag_dict['synced_status']):
            ecal_images = synced_recorder.pop_sync_queue()
        else:
            ecal_images = asynced_recorder.pop_async_queue()

        for imageName in ecal_images:

            imageMsg = ecal_images[imageName]

            img_ndarray = np.frombuffer(imageMsg.data, dtype=np.uint8)
            img_ndarray = img_ndarray.reshape((imageMsg.height, imageMsg.width, 1))
            
            expTime_display = f"expTime = {imageMsg.exposureUSec}" 
            sensIso_display = f"sensIso = {imageMsg.gain}" 
            latencyDevice_display = f"device latency = {imageMsg.header.latencyDevice / 1e6 :.2f} ms" 
            host_latency = time.monotonic() *1e9 - imageMsg.header.stamp 
            latencyHost_display = f"host latency = {host_latency / 1e6 :.2f} ms" 

            all_display = imageName + '\n' + expTime_display + '\n' + sensIso_display + '\n' + latencyDevice_display + '\n' + latencyHost_display 

            # resize to smaller resolution
            # scale_percent = 40 # percent of original size
            # width = int(img_ndarray.shape[1] * scale_percent / 100)
            # height = int(img_ndarray.shape[0] * scale_percent / 100)
            
            dim = (512, 320) #width height
            img_ndarray = cv2.resize(img_ndarray, dim, interpolation = cv2.INTER_NEAREST)

            #convert numpy array to 3 channel
            img_ndarray = np.repeat(img_ndarray.reshape(dim[1], dim[0], 1), 3, axis=2)

            img_ndarray = add_ui_on_ndarray(img_ndarray, imageMsg.exposureUSec, imageMsg.gain, latencyDevice_display, latencyHost_display,
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
            if flag_dict['vio_status']:
                update_proxy(window.proxy_vio, vio_sub.vio_msg)


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