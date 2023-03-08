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


class Recorder:

    def __init__(self, image_topics):

        self.image_sub = SyncedImageSubscriber(image_topics)


class VideoWindow:
    MENU_QUIT = 1

    def __init__(self):

        self.streaming_status = False

        # CONFIGURE WINDOW
        self.window = gui.Application.instance.create_window(
            "Open3D - Video", 1300, 800)
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
        
        self.proxy_1 = gui.WidgetProxy()
        self.collapse.add_child(self.proxy_1)
        
        self.proxy_2 = gui.WidgetProxy()
        self.collapse.add_child(self.proxy_2)

        self.proxy_3 = gui.WidgetProxy()
        self.collapse.add_child(self.proxy_3)

        self.proxy_4 = gui.WidgetProxy()
        self.collapse.add_child(self.proxy_4)

        self.label = gui.Label("control")
        self.label.text_color = gui.Color(1.0, 0.5, 0.0)

        self.collapse.add_child(self.label)
        
        cb = gui.Checkbox("Start streaming")
        cb.set_on_checked(self._on_cb)  # set the callback function
        self.collapse.add_child(cb)
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

        # self.proxy_5 = gui.WidgetProxy()
        # self.rgb_widget_3.add_child(self.proxy_5)
        self.panel_bottom.add_child(self.rgb_widget_3)
        self.panel_bottom.add_child(self.rgb_widget_4)
        self.panel_main.add_child(self.panel_bottom) 


        self.window.add_child(self.panel_main) 


        # start image updating thread
        self.is_done = False

    def _on_layout(self, layout_context):
        contentRect = self.window.content_rect
        panel_main_width = 1040
        self.collapse.frame = gui.Rect(contentRect.x, contentRect.y,
                                contentRect.width - panel_main_width,
                                contentRect.height)

        self.panel_main.frame = gui.Rect(self.collapse.frame.get_right(), contentRect.y,
                                panel_main_width,
                                contentRect.height)

    def _on_close(self):
        self.is_done = True
        return True  # False would cancel the close
    
    def _on_menu_quit(self):
        gui.Application.instance.quit()

    def _on_cb(self, is_checked):
        if is_checked:
            self.streaming_status = True
        else:
            self.streaming_status = False



def read_img(window):


    # PRINT ECAL VERSION AND DATE
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
    
    # INITIALIZE eCAL API
    ecal_core.initialize(sys.argv, "calibration_dataset_recorder")
    
    # SET PROCESS STATE
    ecal_core.set_process_state(1, 1, "I feel good")

    # SET UP SUBSCRIBER
    image_topics = ["S0/camb","S0/camc","S0/camd"]
    
    recorder = Recorder(image_topics)
    recorder.image_sub.rolling = True   # ensure self.image_sub.pop_sync_queue() works


    def update_frame(imageName,img_ndarray):
        if(imageName == 'S0/cama'):
            window.rgb_widget_1.update_image(o3d.geometry.Image(img_ndarray))

        if(imageName == 'S0/camb'):
            window.rgb_widget_2.update_image(o3d.geometry.Image(img_ndarray))
        
        if(imageName == 'S0/camc'):
            window.rgb_widget_3.update_image(o3d.geometry.Image(img_ndarray))
        
        if(imageName == 'S0/camd'):
            window.rgb_widget_4.update_image(o3d.geometry.Image(img_ndarray))

    def update_proxy(proxy,display_msg):
        
        label = gui.Label(display_msg + '\n')
        proxy.set_widget(label)

        window.window.set_needs_layout() 
        




    while ecal_core.ok():

        # READ IN DATA
        ecal_images = recorder.image_sub.pop_sync_queue()
        
        for imageName in ecal_images:

            imageMsg = ecal_images[imageName]

            img_ndarray = np.frombuffer(imageMsg.data, dtype=np.uint8)
            img_ndarray = img_ndarray.reshape((imageMsg.height, imageMsg.width, 1))
            
            expTime_display = f"expTime = {imageMsg.exposureUSec}"
            sensIso_display = f"sensIso = {imageMsg.gain}"
            latencyDevice_display = f"latency device = {imageMsg.header.latencyDevice / 1e6} ms"
            latencyHost_display = f"latency host = {imageMsg.header.latencyHost / 1e6} ms"

            all_display = imageName + '\n' + expTime_display + '\n' + sensIso_display + '\n' + latencyDevice_display + '\n' + latencyHost_display 

            # img_ndarray = cv2.putText(img_ndarray, expTime_display, (100,100), cv2.FONT_HERSHEY_TRIPLEX, 1, (255,0,0), 2)
            # img_ndarray = cv2.putText(img_ndarray, sensIso_display, (100,140), cv2.FONT_HERSHEY_TRIPLEX, 1, (255,0,0), 2)
            # img_ndarray = cv2.putText(img_ndarray, latencyDevice_display, (100,180), cv2.FONT_HERSHEY_TRIPLEX, 1, (255,0,0), 2)    
            # img_ndarray = cv2.putText(img_ndarray, latencyHost_display, (100,220), cv2.FONT_HERSHEY_TRIPLEX, 1, (255,0,0), 2)    
            
            # resize to smaller resolution
            # scale_percent = 40 # percent of original size
            # width = int(img_ndarray.shape[1] * scale_percent / 100)
            # height = int(img_ndarray.shape[0] * scale_percent / 100)
            
            dim = (512, 320) #width height
            img_ndarray = cv2.resize(img_ndarray, dim, interpolation = cv2.INTER_NEAREST)

            #convert numpy array to 3 channel
            img_ndarray = np.repeat(img_ndarray.reshape(dim[1], dim[0], 1), 3, axis=2)


            if not window.is_done and window.streaming_status:

                update_frame(imageName, img_ndarray)

                if(imageName == 'S0/cama'):
                    update_proxy(window.proxy_1,all_display)
                if(imageName == 'S0/camb'):
                    update_proxy(window.proxy_2,all_display)
                if(imageName == 'S0/camc'):
                    update_proxy(window.proxy_3,all_display)
                if(imageName == 'S0/camd'):
                    update_proxy(window.proxy_4,all_display)
                    # update_proxy(window.proxy_5, "hahahah")
        
        window.window.post_redraw()


    # finalize eCAL API
    ecal_core.finalize()



def main(): 

    app = gui.Application.instance
    app.initialize()

    win = VideoWindow()

    # NEW THREAD FOR IMAGE READING
    img_reading_thread= threading.Thread(target=read_img,args=(win,))
    img_reading_thread.start()

    time.sleep(3)

    app.run()
    print("hahah")
    
        

if __name__ == "__main__":
    
    main() 