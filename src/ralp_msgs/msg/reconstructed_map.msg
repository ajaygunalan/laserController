Header header                                  # header with timestamp corresponding to relative image
ralp_msgs/reconstructed_map_pixel[] map        # map with information for each pixel (row major matrix), x increase towards right, y increase toward bottom
uint16 height                                  # image height, that is, number of rows
uint16 width                                   # image width, that is, number of columns
float32 heightMm                               # scene height, in mm
float32 widthMm                                # scene width, in mm
float32 maxZ                                   # maximum height
float32 zeroLevel                              # z-distance from left camera at which there is the 0 level
float32 left                                   # minimum x
float32 right                                  # maximum x
float32 bottom                                 # minimum y
float32 top                                    # maximum y
