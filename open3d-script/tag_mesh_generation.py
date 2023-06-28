import numpy as np
import trimesh
from PIL import Image
from moms_apriltag import TagGenerator2
import cv2

def make_board_from_image(image: Image, board_length: int, board_width: int) -> trimesh.Trimesh:
      # sizing 
      ratio = board_width / board_length
      texture_width = 2 * (board_length + board_width)
      x1 = board_length / texture_width
      x2 = x1 + board_width / texture_width
      x3 = x2 + x1
      y1 = 1/3
      y2 = 2/3

      assert image.width == image.height
      image_length = image.width
      im = Image.new('RGB', (int(2 * image_length * (1 + ratio)), 3 * image_length), (255, 255, 255))
      im.paste(image, (0, image_length))

      uv = [[0, y2], [x1, y2], [0, y1], [x1, y1], [x2, y2], [x2, y1], 
            [x3, y2], [x3, y1], [1, y2], [1, y1], [x1, 1], [x2, 1], 
            [x1, 0], [x2, 0]]
      material = trimesh.visual.texture.SimpleMaterial(image=im)
      color_visuals = trimesh.visual.TextureVisuals(uv=uv, image=im, material=material)

      # 14 vertices and 12 face triangles
      board = trimesh.Trimesh(vertices=[[0, board_length, 0],
                                        [0, board_length, board_length],
                                        [0, 0, 0],
                                        [0, 0, board_length],
                                        [board_width, board_length, board_length],
                                        [board_width, 0, board_length],
                                        [board_width, board_length, 0],
                                        [board_width, 0, 0],
                                        [0, board_length, 0],
                                        [0, 0, 0],
                                        [0, board_length, 0],
                                        [board_width, board_length, 0],
                                        [0, 0, 0],
                                        [board_width, 0, 0]],
                              # right-hand convention
                              faces=[[2,1,0],
                                     [3,1,2],
                                     [3,4,1],
                                     [5,4,3],
                                     [5,6,4],
                                     [7,6,5],
                                     [7,8,6],
                                     [9,8,7],
                                     [11,10,1],
                                     [4,11,1],
                                     [5,3,12],
                                     [13,5,12]],
                              visual=color_visuals,
                              validate=True,
                              process=True)

      # center mesh
      tf = np.eye(4)
      tf[:3, 3] = -board.centroid
      board.apply_transform(tf)
      return board

def get_tag_image(family, id):
      tag_family = TagGenerator2(family)
      tag = cv2.resize(tag_family.generate(id),
                       dsize=(300, 300),
                       interpolation=cv2.INTER_NEAREST)
      tag = Image.fromarray(tag)
      return tag

if __name__ == "__main__":
      trimesh.util.attach_to_log()

      auto_apriltag = get_tag_image("tag16h5", 2)
      raw_apriltag = Image.open('assets/apriltag.png')

      board = make_board_from_image(auto_apriltag, 0.15, 0.5)
      board.show()
      board.export('assets/board.obj')
