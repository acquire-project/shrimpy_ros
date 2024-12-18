import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Image
#import numpy as np
from torch import Tensor
from waveorder.models import phase_thick_3d

class PhaseReconstructionNode(Node):

    def __init__(self):
        super().__init__('phase_reconstruction_node')
        self.sub = self.create_subscription(Image, 'image_raw', self.image_callback, 11)
        
        self.get_logger().info("Generating phase calibration")
                # calculate the phase calibration
        self.simulation_arguments = {
            "zyx_shape": (11, 2448, 2048), # update to match data shape, probably (11, 2448, 2048)
            "yx_pixel_size": 0.345, # in um, camera has 3.45 um pixels and we use 10x magnification
            "z_pixel_size": 3, # in um, 1V piezo steps correspond to 3 um steps, update as needed
            "index_of_refraction_media": 1, # this is an air immersion objective
        }

        self.transfer_function_arguments = {
            "z_padding": 5, # it help to add padding to avoid wrapping artifacts at the edges
            "wavelength_illumination": 0.532, # in um, green light
            "numerical_aperture_illumination": 0.4, # this is a rough estimate
            "numerical_aperture_detection": 0.45, # the objective has 0.45 NA detection
        }
        zyx=self.simulation_arguments["zyx_shape"]
        self.image_z_stack = Tensor.new_empty(zyx)
        self.generate_phase_calibration()
        self.get_logger().info("Phase reconstruction node started")
        
    def image_callback(self, msg: Image):

        self.image_z_stack[self.image_count] = msg.data
        self.get_logger().info(f"Received image, I now have {self.image_count} images")
        self.get_logger().info(f"{type(msg.data)}")

        if len(self.image_queue) == self.simulation_arguments["zyx_shape"][0]:
            self.get_logger().info("Performing phase reconstruction")
            self.perform_phase_reconstruction()
            self.image_count = 0
        else:
            self.image_count += 1

    def generate_phase_calibration(self):
        

        # Calculate transfer function
        (
            self.real_potential_transfer_function,
            self.imag_potential_transfer_function,
        ) = phase_thick_3d.calculate_transfer_function(
            **self.simulation_arguments, **self.transfer_function_arguments
        )

        self.get_logger().info(f"Phase calibration generated with shape {self.real_potential_transfer_function.shape}")

    
    def perform_phase_reconstruction(self):
        # concatenate images as a 3d array
        self.get_logger().info(f"Image volume generated with shape {self.real_potential_transfer_function.shape}")
        
        # Reconstruct
        zyx_recon = phase_thick_3d.apply_inverse_transfer_function(
            self.image_z_stack,
            self.real_potential_transfer_function,
            self.imag_potential_transfer_function,
            self.transfer_function_arguments["z_padding"],
            self.simulation_arguments["z_pixel_size"],
            self.transfer_function_arguments["wavelength_illumination"]
        )
        
        
def main(args=None):
    rclpy.init(args=args)

    node = PhaseReconstructionNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("Handling interrupt: Shutting down")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
