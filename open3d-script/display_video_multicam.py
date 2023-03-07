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

# store the meta data from ecal
image_dict = {}
# store the image used in gui display
rgb_frame_dict = {}


class Recorder:

    def __init__(self, image_topics):

        self.image_sub = SyncedImageSubscriber(image_topics)


class VideoWindow:
    MENU_QUIT = 1

    def __init__(self):


        # configure window
        self.window = gui.Application.instance.create_window(
            "Open3D - Video", 4000, 800)
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
        self.panel = gui.Horiz(0.5 * em, gui.Margins(margin))

        self.rgb_widget_1 = gui.ImageWidget(o3d.geometry.Image(np.zeros((800,1280,1), dtype=np.uint8)))
        self.rgb_widget_2 = gui.ImageWidget(o3d.geometry.Image(np.zeros((800,1280,1), dtype=np.uint8)))
        self.rgb_widget_3 = gui.ImageWidget(o3d.geometry.Image(np.zeros((800,1280,1), dtype=np.uint8)))

        self.panel.add_child(self.rgb_widget_1)
        self.panel.add_child(self.rgb_widget_2)
        self.panel.add_child(self.rgb_widget_3)

        self.window.add_child(self.panel)        
        

        # start image updating thread
        self.is_done = False
        threading.Thread(target=self._update_thread).start()

    def _update_thread(self):
            # This is NOT the UI thread, need to call post_to_main_thread() to update
            # the scene or any part of the UI.
            global image_dict
            global rgb_frame_dict
            while not self.is_done:

                time.sleep(0.100)

            # Get the next frame, for instance, reading a frame from the camera.
                # if (len(rgb_frame_dict) == 0):

                #     print(len(image_dict))

                #     for imageName in image_dict:
                #         imageMsg = image_dict[imageName]
                #         img_ndarray = np.frombuffer(imageMsg.data, dtype=np.uint8)
                #         img_ndarray = img_ndarray.reshape((imageMsg.height, imageMsg.width, 1))
                #         # print("raw shape = ",img_ndarray.shape)

                #         # img_ndarray = image_resize(img_ndarray, width=640)
                        
                #         # dim = (640, 400,1)
                #         # # resize image
                #         # img_ndarray = cv2.resize(img_ndarray, dim, interpolation = cv2.INTER_NEAREST)
                #         # print("after resize = ",img_ndarray.shape)

                #         #convert numpy array to 3 channel (800,1280,3)
                #         img_ndarray = np.repeat(img_ndarray, 3, axis=2)
                #         # print("after extend to 3 channels = ",img_ndarray.shape)

                #         # make sure img_ndarray (800,1280,3)
                #         #convert numpy array to open3d image
                #         rgb_frame_dict[imageName] = o3d.geometry.Image(img_ndarray)
                    



                # Update the images. This must be done on the UI thread.
                def update():

                    global image_dict
                    global rgb_frame_dict

                    if(len(rgb_frame_dict) == 0):
                        print(len(image_dict))

                        for imageName in image_dict:
                            imageMsg = image_dict[imageName]
                            img_ndarray = np.frombuffer(imageMsg.data, dtype=np.uint8)
                            img_ndarray = img_ndarray.reshape((imageMsg.height, imageMsg.width, 1))

                            img_ndarray = np.repeat(img_ndarray, 3, axis=2)

                            # make sure img_ndarray (800,1280,3)
                            #convert numpy array to open3d image
                            rgb_frame_dict[imageName] = o3d.geometry.Image(img_ndarray)

                        # print(type(rgb_frame_dict["S0/camd"]))
                        self.rgb_widget_1.update_image(rgb_frame_dict["S0/camb"])
                        self.rgb_widget_2.update_image(rgb_frame_dict["S0/camc"])
                        self.rgb_widget_3.update_image(rgb_frame_dict["S0/camd"])

                        # suspecting race condition here
                        rgb_frame_dict = {}

                if not self.is_done:
                    gui.Application.instance.post_to_main_thread(
                        self.window, update)
    
    
    def _on_close(self):
        self.is_done = True
        return True  # False would cancel the close
    
    def _on_menu_quit(self):
        gui.Application.instance.quit()





def read_img():

    global image_dict

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


    while ecal_core.ok():
        # READ IN DATA
        image_dict = recorder.image_sub.pop_sync_queue()

    # finalize eCAL API
    ecal_core.finalize()








def main(): 
    

    # NEW THREAD FOR IMAGE READING
    img_reading_thread= threading.Thread(target=read_img)
    img_reading_thread.start()

    time.sleep(3)

    app = gui.Application.instance
    app.initialize()

    win = VideoWindow()

    app.run()
    
        





if __name__ == "__main__":
    
    main() 