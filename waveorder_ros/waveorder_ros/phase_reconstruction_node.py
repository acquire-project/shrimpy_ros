import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Image
import numpy as np
import waveorder

class PhaseReconstructionNode(Node):

    def __init__(self):
        super().__init__('phase_reconstruction_node')
        self.sub = self.create_subscription(Image, 'image_raw', self.image_callback, 10)
        self.image_queue = []
        
        self.get_logger().info("Generating phase calibration")
        self.generate_phase_calibration()
        self.get_logger().info("Phase reconstruction node started")
        
    def image_callback(self, msg: Image):
        self.image_queue.append(msg)
        self.get_logger().info(f"Received image {msg.header.seq}, I now have {len(self.image_queue)} images")

        if len(self.image_queue) == self.simulation_arguments["zyx_shape"][0]:
            self.get_logger().info("Performing phase reconstruction")
            self.perform_phase_reconstruction()
            self.image_queue = []

    def generate_phase_calibration(self):
        
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
        
        # Calculate transfer function
        (
            self.real_potential_transfer_function,
            self.imag_potential_transfer_function,
        ) = waveorder.phase_thick_3d.calculate_transfer_function(
            **self.simulation_arguments, **self.transfer_function_arguments
        )

        self.get_logger().info(f"Phase calibration generated with shape {self.real_potential_transfer_function.shape}")

    
    def perform_phase_reconstruction(self):
        # concatenate images as a 3d array
        image_volume = np.array([np.array(image.data) for image in self.image_queue])
        
        self.get_logger().info(f"Image volume generated with shape {self.real_potential_transfer_function.shape}")
        
        zyx_recon = waveorder.phase_thick_3d.apply_inverse_transfer_function(
            image_volume,
            self.real_potential_transfer_function,
            self.imag_potential_transfer_function,
            self.transfer_function_arguments["z_padding"],
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
