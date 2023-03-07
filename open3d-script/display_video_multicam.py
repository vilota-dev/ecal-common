#!/usr/bin/env python3

import sys
import time
import threading

import capnp
import numpy as np
import cv2
import platform

import ecal.core.core as ecal_core
# from byte_subscriber import ByteSubscriber

capnp.add_import_hook(['../src/capnp'])

import image_capnp as eCALImage
import cameracontrol_capnp as eCALCameraControl
import open3d as o3d
import open3d.visualization.gui as gui

from PIL import Image

from utils import SyncedImageSubscriber, image_resize

isMacOS = (platform.system() == "Darwin")


class Recorder:

    def __init__(self, image_topics):

        self.image_sub = SyncedImageSubscriber(image_topics)


class VideoWindow:
    MENU_QUIT = 1

    def __init__(self):

        self.streaming_status = False

        # configure window
        self.window = gui.Application.instance.create_window(
            "Open3D - Video", 1400, 850)
        self.window.set_on_close(self._on_close)

        # configure menu
        if gui.Application.instance.menubar is None:
            if isMacOS:
                app_menu = gui.Menu()
                app_menu.add_item("Quit", VideoWindow.MENU_QUIT)
            debug_menu = gui.Menu()
            if not isMacOS:
                debug_menu.add_separator()
                debug_menu.add_item("Quit", VideoWindow.MENU_QUIT)

            menu = gui.Menu()
            if isMacOS:
                # macOS will name the first menu item for the running application
                # (in our case, probably "Python"), regardless of what we call
                # it. This is the application menu, and it is where the
                # About..., Preferences..., and Quit menu items typically go.
                menu.add_menu("Example", app_menu)
                menu.add_menu("Debug", debug_menu)
            else:
                menu.add_menu("Debug", debug_menu)
            gui.Application.instance.menubar = menu

        # The menubar is global, but we need to connect the menu items to the
        # window, so that the window can call the appropriate function when the
        # menu item is activated.

        self.window.set_on_menu_item_activated(VideoWindow.MENU_QUIT,
                                               self._on_menu_quit)



        # configure gui
        em = self.window.theme.font_size
        margin = 0.5 * em
        self.panel_main = gui.Vert(0.5 * em, gui.Margins(margin))

        self.panel_top = gui.Horiz(0.5 * em, gui.Margins(margin))

        self.rgb_widget_1 = gui.ImageWidget(o3d.geometry.Image(np.zeros((400,640,1), dtype=np.uint8)))
        self.rgb_widget_2 = gui.ImageWidget(o3d.geometry.Image(np.zeros((400,640,1), dtype=np.uint8)))
        self.panel_top.add_child(self.rgb_widget_1)
        self.panel_top.add_child(self.rgb_widget_2)
        
        self.panel_main.add_child(self.panel_top) 

        self.panel_bottom = gui.Horiz(0.5 * em, gui.Margins(margin))

        collapse = gui.CollapsableVert("Widgets", 0.33 * em,
                                       gui.Margins(em, 0, 0, 0))
        self._label = gui.Label("control")
        self._label.text_color = gui.Color(1.0, 0.5, 0.0)

        collapse.add_child(self._label)
        cb = gui.Checkbox("Start streaming")
        cb.set_on_checked(self._on_cb)  # set the callback function
        collapse.add_child(cb)
        self.panel_bottom.add_child(collapse)
        
        self.rgb_widget_3 = gui.ImageWidget(o3d.geometry.Image(np.zeros((400,640,1), dtype=np.uint8)))
        self.panel_bottom.add_child(self.rgb_widget_3)

        self.panel_main.add_child(self.panel_bottom) 


        self.window.add_child(self.panel_main) 


        # start image updating thread
        self.is_done = False

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
        if(imageName == 'S0/camb'):
            window.rgb_widget_1.update_image(o3d.geometry.Image(img_ndarray))
        
        if(imageName == 'S0/camc'):
            window.rgb_widget_2.update_image(o3d.geometry.Image(img_ndarray))
        
        if(imageName == 'S0/camd'):
            window.rgb_widget_3.update_image(o3d.geometry.Image(img_ndarray))

    while ecal_core.ok():
        
        # READ IN DATA
        ecal_images = recorder.image_sub.pop_sync_queue()
        timestamp1 = time.monotonic()
        for imageName in ecal_images:

            imageMsg = ecal_images[imageName]

            img_ndarray = np.frombuffer(imageMsg.data, dtype=np.uint8)
            img_ndarray = img_ndarray.reshape((imageMsg.height, imageMsg.width, 1))
            
            expTime_display = f"expTIme = {imageMsg.exposureUSec}"
            sensIso_display = f"sensIso = {imageMsg.gain}"
            latencyDevice_display = f"latency device = {imageMsg.header.latencyDevice / 1e6} ms"
            latencyHost_display = f"latency host = {imageMsg.header.latencyHost / 1e6} ms"

            img_ndarray = cv2.putText(img_ndarray, expTime_display, (100,100), cv2.FONT_HERSHEY_TRIPLEX, 1, (255,0,0), 2)
            img_ndarray = cv2.putText(img_ndarray, sensIso_display, (100,140), cv2.FONT_HERSHEY_TRIPLEX, 1, (255,0,0), 2)
            img_ndarray = cv2.putText(img_ndarray, latencyDevice_display, (100,180), cv2.FONT_HERSHEY_TRIPLEX, 1, (255,0,0), 2)    
            img_ndarray = cv2.putText(img_ndarray, latencyHost_display, (100,220), cv2.FONT_HERSHEY_TRIPLEX, 1, (255,0,0), 2)    
    
            
            # resize to smaller resolution
            scale_percent = 40 # percent of original size
            width = int(img_ndarray.shape[1] * scale_percent / 100)
            height = int(img_ndarray.shape[0] * scale_percent / 100)
            dim = (width, height)
            
            img_ndarray = cv2.resize(img_ndarray, dim, interpolation = cv2.INTER_NEAREST)

            #convert numpy array to 3 channel (800,1280,3)
            img_ndarray = np.repeat(img_ndarray.reshape(height, width, 1), 3, axis=2)
            # print(f"size after repeat = {img_ndarray.shape}")


            if not window.is_done and window.streaming_status:
                update_frame(imageName, img_ndarray)
        
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
    
        


if __name__ == "__main__":
    
    main() 