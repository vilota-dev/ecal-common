#!/usr/bin/env python3

import sys
import time
import threading

import capnp
import numpy as np
import cv2
import platform

import ecal.core.core as ecal_core

capnp.add_import_hook(['../src/capnp'])

import image_capnp as eCALImage
import cameracontrol_capnp as eCALCameraControl
import open3d as o3d
import open3d.visualization.gui as gui

from PIL import Image

from utils import SyncedImageSubscriber

isMacOS = (platform.system() == "Darwin")

image_topics = []

class Recorder:

    def __init__(self, image_topics):

        self.image_sub = SyncedImageSubscriber(image_topics)

class ChooseWindow:

    def __init__(self):
        
        self.is_done = False
        
        self.status_cama = False
        self.status_camb = False
        self.status_camc = False
        self.status_camd = False
        
        # CONFIGURE WINDOW
        self.window = gui.Application.instance.create_window(
            "Camera confirm", 500, 300)
        self.window.set_on_layout(self._on_layout)
        self.window.set_on_close(self._on_close) 
        
        
        # CONFIGURE GUI
        em = self.window.theme.font_size
        margin = 0.5 * em
        
        self.panel_main = gui.Vert(0.5 * em, gui.Margins(margin))

        self.label_description = gui.Label("Please choose the avaliable camera and click 'Ok'.")
        self.label_description.text_color = gui.Color(1.0, 0.5, 0.0)
        self.panel_main.add_child(self.label_description)

        cb_cama = gui.Checkbox("cam_a")
        cb_cama.set_on_checked(self._on_cb_cama)  # set the callback function
        self.panel_main.add_child(cb_cama)

        cb_camb = gui.Checkbox("cam_b")
        cb_camb.set_on_checked(self._on_cb_camb)  
        self.panel_main.add_child(cb_camb)

        cb_camc = gui.Checkbox("cam_c")
        cb_camc.set_on_checked(self._on_cb_camc)  
        self.panel_main.add_child(cb_camc)

        cb_camd = gui.Checkbox("cam_d")
        cb_camd.set_on_checked(self._on_cb_camd)  
        self.panel_main.add_child(cb_camd)
        
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


    def _on_cb_cama(self, is_checked):
        if is_checked:
            self.status_cama = True
        else:
            self.status_cama = False
    
    def _on_cb_camb(self, is_checked):
        if is_checked:
            self.status_camb = True
        else:
            self.status_camb = False    
    
    def _on_cb_camc(self, is_checked):
        if is_checked:
            self.status_camc = True
        else:
            self.status_camc = False    
    
    def _on_cb_camd(self, is_checked):
        if is_checked:
            self.status_camd = True
        else:
            self.status_camd = False
    
    def _on_ok(self):

        if self.status_cama:
            image_topics.append("S0/cama")
        if self.status_camb:
            image_topics.append("S0/camb")        
        if self.status_camc:
            image_topics.append("S0/camc")        
        if self.status_camd:
            image_topics.append("S0/camd")
        
        print(f"Subscribing to {image_topics}")

        gui.Application.instance.quit()




class VideoWindow:
    MENU_QUIT = 1

    def __init__(self):
                
        self.is_done = False

        self.streaming_status_cama = False
        self.streaming_status_camb = False
        self.streaming_status_camc = False
        self.streaming_status_camd = False

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
        
        cb_cama = gui.Checkbox("Start streaming cama")
        cb_cama.set_on_checked(self._on_cb_cama)  # set the callback function
        self.collapse.add_child(cb_cama)

        cb_camb = gui.Checkbox("Start streaming camb")
        cb_camb.set_on_checked(self._on_cb_camb)  # set the callback function
        self.collapse.add_child(cb_camb)

        cb_camc = gui.Checkbox("Start streaming camc")
        cb_camc.set_on_checked(self._on_cb_camc)  # set the callback function
        self.collapse.add_child(cb_camc)

        cb_camd = gui.Checkbox("Start streaming camd")
        cb_camd.set_on_checked(self._on_cb_camd)  # set the callback function
        self.collapse.add_child(cb_camd)
        
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

        self.label_info = gui.Label("Information")
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

    def _on_cb_cama(self, is_checked):
        if is_checked:
            self.streaming_status_cama = True
        else:
            self.streaming_status_cama = False
    
    def _on_cb_camb(self, is_checked):
        if is_checked:
            self.streaming_status_camb = True
        else:
            self.streaming_status_camb = False    
    
    def _on_cb_camc(self, is_checked):
        if is_checked:
            self.streaming_status_camc = True
        else:
            self.streaming_status_camc = False    
    
    def _on_cb_camd(self, is_checked):
        if is_checked:
            self.streaming_status_camd = True
        else:
            self.streaming_status_camd = False
    
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

    # print(type(ecal_core.mon_monitoring()[1]))

    recorder = Recorder(image_topics)
    recorder.image_sub.rolling = True   # ensure self.image_sub.pop_sync_queue() works

    # print(ecal_core.mon_monitoring())

    def update_frame(imageName,img_ndarray):
        if(imageName == 'S0/cama' and window.streaming_status_cama):
            window.rgb_widget_1.update_image(o3d.geometry.Image(img_ndarray))

        if(imageName == 'S0/camb' and window.streaming_status_camb):
            window.rgb_widget_2.update_image(o3d.geometry.Image(img_ndarray))
        
        if(imageName == 'S0/camc' and window.streaming_status_camc):
            window.rgb_widget_3.update_image(o3d.geometry.Image(img_ndarray))
        
        if(imageName == 'S0/camd' and window.streaming_status_camd):
            window.rgb_widget_4.update_image(o3d.geometry.Image(img_ndarray))


    def update_proxy(proxy,display_msg):
        
        widget = proxy.get_widget()
       
        if widget is None:
            label = gui.Label(display_msg)
            proxy.set_widget(label)
        else:
            widget.text = display_msg

        window.window.set_needs_layout() 
        

    expMin = 500
    expMax = 12000
    sensMin = 110
    sensMax = 800
    
    progress_bar_length = 200
    progress_bar_height = 15
    
    left_x = 10
    
    left_y_pb1 = 30
    spacing_2pb = 30
    left_y_pb2 = left_y_pb1 + progress_bar_height + spacing_2pb

    expTime_frame_start = (left_x, left_y_pb1)
    expTime_frame_end = (left_x + progress_bar_length, left_y_pb1 + progress_bar_height)
    
    sensIso_frame_start = (left_x,left_y_pb2)
    sensIso_frame_end = (left_x + progress_bar_length, left_y_pb2 + progress_bar_height)

    spacing_pb_text = 40
    spacing_2text = 20
    latencyDevice_coor = (left_x, left_y_pb2 + progress_bar_height + spacing_pb_text)
    latencyHost_coor = (left_x, left_y_pb2 + progress_bar_height + spacing_pb_text + spacing_2text)

    while ecal_core.ok():

        # READ IN DATA
        ecal_images = recorder.image_sub.pop_sync_queue()
        
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


            if window.expTime_display_flag:
                # add progress bar
                expTime_length = int((imageMsg.exposureUSec - expMin) / (expMax - expMin) * progress_bar_length)
                img_ndarray = cv2.rectangle(img_ndarray, expTime_frame_start, (left_x + expTime_length, expTime_frame_end[1]), (255, 0, 0),-1)
                # add progress bar frame
                img_ndarray = cv2.rectangle(img_ndarray, expTime_frame_start, expTime_frame_end, (255, 255, 255),2)
                # add description
                img_ndarray = cv2.putText(img_ndarray, 'expTime', (expTime_frame_end[0]+5, expTime_frame_start[1]), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (255, 0, 0), 1)
                img_ndarray = cv2.putText(img_ndarray, str(expMin), (expTime_frame_start[0],expTime_frame_end[1]+15), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255, 255, 255), 1)    
                img_ndarray = cv2.putText(img_ndarray, str(expMax), (expTime_frame_end[0],expTime_frame_end[1]+15), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255, 255, 255), 1)    

            if window.sensIso_display_flag:
                # add progress bar
                sensIso_length = int((imageMsg.gain - sensMin) / (sensMax - sensMin) * progress_bar_length)
                img_ndarray = cv2.rectangle(img_ndarray, sensIso_frame_start, (left_x +sensIso_length, sensIso_frame_end[1]), (0, 0, 255),-1)
                # add progress bar frame
                img_ndarray = cv2.rectangle(img_ndarray, sensIso_frame_start, sensIso_frame_end, (255, 255, 255),2)
                # add description
                img_ndarray = cv2.putText(img_ndarray, 'sensIso', (sensIso_frame_end[0]+5, sensIso_frame_start[1]), cv2.FONT_HERSHEY_TRIPLEX, 0.5, (0, 0, 255), 1)    
                img_ndarray = cv2.putText(img_ndarray, str(sensMin), (sensIso_frame_start[0],sensIso_frame_end[1]+15), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255, 255, 255), 1)    
                img_ndarray = cv2.putText(img_ndarray, str(sensMax), (sensIso_frame_end[0],sensIso_frame_end[1]+15), cv2.FONT_HERSHEY_TRIPLEX, 0.4, (255, 255, 255), 1)    


            # add latency text
            if window.latencyDevice_display_flag:
                img_ndarray = cv2.putText(img_ndarray, latencyDevice_display, latencyDevice_coor, cv2.FONT_HERSHEY_TRIPLEX, 0.5, (127, 0, 255), 1)  
            
            if window.latencyHost_display_flag:
                img_ndarray = cv2.putText(img_ndarray, latencyHost_display, latencyHost_coor, cv2.FONT_HERSHEY_TRIPLEX, 0.5, (127, 0, 255), 1)    


            if not window.is_done:

                update_frame(imageName, img_ndarray)
                if(imageName == 'S0/cama' and window.streaming_status_cama):
                    update_proxy(window.proxy_1,all_display)
                if(imageName == 'S0/camb' and window.streaming_status_camb):
                    update_proxy(window.proxy_2,all_display)
                if(imageName == 'S0/camc' and window.streaming_status_camc):
                    update_proxy(window.proxy_3,all_display)
                if(imageName == 'S0/camd' and window.streaming_status_camd):
                    update_proxy(window.proxy_4,all_display)
        
        window.window.post_redraw()


    # finalize eCAL API
    ecal_core.finalize()



def main(): 
    
    # choose camera app
    choose_app = gui.Application.instance
    choose_app.initialize()

    choose_win = ChooseWindow()
    
    choose_app.run()    

    # main app
    app = gui.Application.instance
    app.initialize()
    
    win = VideoWindow()
    
    # NEW THREAD FOR IMAGE READING    
    img_reading_thread= threading.Thread(target=read_img,args=(win,))
    img_reading_thread.start()
    
    time.sleep(3)
    
    app.run()
    
        

if __name__ == "__main__":
    
    main() 