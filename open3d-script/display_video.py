import sys
import time

import capnp
import numpy as np
import cv2
import threading
import platform


import ecal.core.core as ecal_core
from byte_subscriber import ByteSubscriber

capnp.add_import_hook(['../src/capnp'])

import image_capnp as eCALImage

import open3d as o3d
import open3d.visualization.gui as gui

from PIL import Image

isMacOS = (platform.system() == "Darwin")

imshow_map = {}

exposure_string = ""


class VideoWindow:
    MENU_QUIT = 1

    def __init__(self):

        self.rgb_images = []


        self.window = gui.Application.instance.create_window(
            "Open3D - Video", 1280, 1000)
        self.window.set_on_close(self._on_close)

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




        em = self.window.theme.font_size
        margin = 0.5 * em
        self.panel = gui.Vert(0.5 * em, gui.Margins(margin))
        self.panel.add_child(gui.Label("Color image"))
        exposure_label = gui.Label("exposure : " + exposure_string)
        exposure_label.text = exposure_string+"hello"
        self.panel.add_child(exposure_label)
        
        self.rgb_widget = gui.ImageWidget()
        self.panel.add_child(self.rgb_widget)
        self.window.add_child(self.panel)        

        collapse = gui.CollapsableVert("Widgets", 0.33 * em,
                                       gui.Margins(em, 0, 0, 0))
        self._label = gui.Label("control")
        self._label.text_color = gui.Color(1.0, 0.5, 0.0)

        collapse.add_child(self._label)
        cb = gui.Checkbox("Start streaming")
        cb.set_on_checked(self._on_cb)  # set the callback function
        collapse.add_child(cb)



        self.panel.add_child(collapse)









        self.streaming_status = False

        self.is_done = False
        threading.Thread(target=self._update_thread).start()

    def _update_thread(self):
            # This is NOT the UI thread, need to call post_to_main_thread() to update
            # the scene or any part of the UI.
            img_ndarray = np.zeros((800,1280,3), dtype=np.uint8)
            rgb_frame = o3d.geometry.Image(img_ndarray)
            while not self.is_done:
                time.sleep(0.100)

                # Get the next frame, for instance, reading a frame from the camera.
                for im in imshow_map:

                    #get the numpy array (800,1280,1)
                    img_ndarray = imshow_map[im] 

                    #convert numpy array to 3 channel (800,1280,3)
                    img_ndarray=np.repeat(img_ndarray, 3, axis=2)

                    #convert numpy array to open3d image
                    rgb_frame = o3d.geometry.Image(img_ndarray)


                # Update the images. This must be done on the UI thread.
                def update():
                    if self.streaming_status:
                        self.rgb_widget.update_image(rgb_frame)
                    

                if not self.is_done:
                    gui.Application.instance.post_to_main_thread(
                        self.window, update)
    
    
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




def callback(topic_name, msg, ts):

    # need to remove the .decode() function within the Python API of ecal.core.subscriber StringSubscriber
    with eCALImage.Image.from_bytes(msg) as imageMsg:
        # print(f"seq = {imageMsg.header.seq}, stamp = {imageMsg.header.stamp}, with {len(msg)} bytes, encoding = {imageMsg.encoding}")
        # print(f"width = {imageMsg.width}, height = {imageMsg.height}")
        # print(f"exposure = {imageMsg.exposureUSec}, gain = {imageMsg.gain}")
        # print(f"intrinsic = {imageMsg.intrinsic}")
        # print(f"extrinsic = {imageMsg.extrinsic}")

        if (imageMsg.encoding == "mono8"):
            exposure_string = str(imageMsg.exposureUSec)

            mat = np.frombuffer(imageMsg.data, dtype=np.uint8)
            mat = mat.reshape((imageMsg.height, imageMsg.width, 1))
            
            expTime_display = str(imageMsg.exposureUSec)
            sensIso_display = str(imageMsg.gain)

            mat = cv2.putText(mat, "expTIme = " + expTime_display, (100,100), cv2.FONT_HERSHEY_TRIPLEX, 2, (255,0,0), 2)
            mat = cv2.putText(mat, "sensIso = " + sensIso_display, (100,200), cv2.FONT_HERSHEY_TRIPLEX, 2, (255,0,0), 2)            
            
            
            imshow_map[topic_name + " mono8"] = mat

            # cv2.imshow("mono8", mat)
            # cv2.waitKey(3)
        elif (imageMsg.encoding == "yuv420"):
            mat = np.frombuffer(imageMsg.data, dtype=np.uint8)
            mat = mat.reshape((imageMsg.height * 3 // 2, imageMsg.width, 1))

            mat = cv2.cvtColor(mat, cv2.COLOR_YUV2BGR_IYUV)

            imshow_map[topic_name + " yuv420"] = mat
            # cv2.imshow("yuv420", mat)
            # cv2.waitKey(3)
        elif (imageMsg.encoding == "bgr8"):
            mat = np.frombuffer(imageMsg.data, dtype=np.uint8)
            mat = mat.reshape((imageMsg.height, imageMsg.width, 3))
            imshow_map[topic_name + " bgr8"] = mat
        elif (imageMsg.encoding == "jpeg"):
            mat_jpeg = np.frombuffer(imageMsg.data, dtype=np.uint8)
            mat = cv2.imdecode(mat_jpeg, cv2.IMREAD_COLOR)
            imshow_map[topic_name + " jpeg"] = mat
        else:
            raise RuntimeError("unknown encoding: " + imageMsg.encoding)


def main():
     # print eCAL version and date
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
    
    # initialize eCAL API
    ecal_core.initialize(sys.argv, "test_image_sub")
    
    # set process state
    ecal_core.set_process_state(1, 1, "I feel good")

    # create subscriber and connect callback

    n = len(sys.argv)
    if n == 1:
        topic = "S0/camd"
    elif n == 2:
        topic = sys.argv[1]
    else:
        raise RuntimeError("Need to pass in exactly one parameter for topic")

    print(f"Streaming topic {topic}")
    sub = ByteSubscriber(topic)
    sub.set_callback(callback)
    

    app = gui.Application.instance
    app.initialize()

    win = VideoWindow()

    app.run()


if __name__ == "__main__":
    main()