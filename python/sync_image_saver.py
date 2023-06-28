#!/usr/bin/env python3


import numpy as np
import cv2
import sys
import json

import ecal.core.core as ecal_core
import capnp

capnp.add_import_hook(['../src/capnp'])

import image_capnp as eCALImage
import disparity_capnp as eCALDisparity

from utils import SyncedImageSubscriber, image_msg_to_cv_mat, disparity_to_cv_mat, image_resize

class SyncImageSaver:
    def __init__(self, types, topics, typeclasses) -> None:
        self.sub = SyncedImageSubscriber(types, topics, typeclasses, False)

    # def take_snapshot(self):
    #     image_dict = self.sub.pop_latest()

def main(types, topics, typeclasses):

    # print eCAL version and date
    print("eCAL {} ({})\n".format(ecal_core.getversion(), ecal_core.getdate()))
    
    # initialize eCAL API
    ecal_core.initialize(sys.argv, "sync_image_saver")
    
    # set process state
    ecal_core.set_process_state(1, 1, "I feel good")

    saver = SyncImageSaver(types, topics, typeclasses)

    count = 1

    while ecal_core.ok():

        image_dict = saver.sub.pop_latest()

        image_list = []
        image_vis_list = []
        image_name = ""
        text_dicts = {}

        if (len(image_dict) != 0):

            for i in range(len(topics)):
                topic = topics[i]
                type = types[i]

                image_name += topic + " "

                text_dict = {}

                text_dict["type"] = type
                # text_dict["topic"] = topic

                if type == "Image":
                    imageMsg = image_dict[topic]
                    mat, mat_vis = image_msg_to_cv_mat(imageMsg)

                    mat_resized = image_resize(mat_vis, width=360)
                    image_vis_list.append(mat_resized)
                    image_list.append(mat)


                    text_dict["width"] = imageMsg.width
                    text_dict["height"] = imageMsg.height
                    
                    text_dict["exposure_usec"] = imageMsg.exposureUSec
                    text_dict["gain"] = imageMsg.gain
                    text_dict["intrinsic"] = f"{imageMsg.intrinsic}"

                elif type == "Disparity":
                    imageMsg = image_dict[topic]
                    mat, disp_vis = disparity_to_cv_mat(imageMsg)

                    disp_vis_resized = image_resize(disp_vis, width=360)
                    image_vis_list.append(disp_vis_resized)
                    image_list.append(mat)

                    text_dict["width"] = imageMsg.width
                    text_dict["height"] = imageMsg.height

                    text_dict["fx"] = imageMsg.fx
                    text_dict["fy"] = imageMsg.fy
                    text_dict["cx"] = imageMsg.cx
                    text_dict["cy"] = imageMsg.cy
                    text_dict["baseline"] = imageMsg.baseline
                    text_dict["encoding"] = f"{imageMsg.encoding}"
                else:
                    raise RuntimeError("should not be here")
                
                text_dicts[topic] = text_dict

            cv2.imshow(image_name, cv2.hconcat(image_vis_list))
        
        key = cv2.waitKey(10)
        if key == ord('q'):
            break
        elif key == ord(' '):
            print(f"take snapshot {count}")

            for i in range(len(topics)):
                topic = topics[i]
                type = types[i]

                topic_noslash = topic.replace("/", "-")

                filename = f"output/{count:03d}_{topic_noslash}_{type}.tiff"

                cv2.imwrite(filename, image_list[i])
                print(f"writen {filename}")

            filename = f"output/{count:03d}.json"

            with open(filename, "w") as outfile:
                json.dump(text_dicts, outfile, indent=4)
                print(f"written {filename}")


            count += 1





if __name__ == "__main__":

    # types can be Image or Disparity
    types = ["Disparity", "Disparity", "Disparity", "Disparity"]
    typeclasses = [eCALDisparity.Disparity, 
                   eCALDisparity.Disparity, 
                   eCALDisparity.Disparity, 
                   eCALDisparity.Disparity]
    topics = ["S1/disparity/stereo1", 
              "S0/disparity/stereo1", 
              "S0/disparity/stereo2", 
              "S1/disparity/stereo2"]

    main(types, topics, typeclasses)
