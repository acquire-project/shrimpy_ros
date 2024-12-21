import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Image
import numpy as np
import torch
from waveorder.models import phase_thick_3d

class PhaseReconstructionNode(Node):

    def __init__(self):
        super().__init__('phase_reconstruction_node')
        
        self.setup_parameters()
        
        self.get_logger().info("Generating phase calibration")
        # calculate the phase calibration
        
        self.image_z_stack = np.zeros( (self.get_parameter("zyx_shape").value[0],
                                        self.get_parameter("zyx_shape").value[1] * self.get_parameter("zyx_shape").value[2]),
                                        dtype=np.uint8)
        self.generate_phase_calibration()

        self.image_count = 0
        self.sub = self.create_subscription(Image, 'flir_camera/image_raw', self.image_callback, 11)
        self.get_logger().info("Phase reconstruction node started")

    def setup_parameters(self):
        # Define the simulation and transfer function arguments as parameters
        # These should be updated to match the microscope and camera setup
        self.declare_parameter("zyx_shape", (11, 2448, 2048))
        self.declare_parameter("yx_pixel_size", 0.345)
        self.declare_parameter("z_pixel_size", 3)
        self.declare_parameter("index_of_refraction_media", 1)
        
        self.declare_parameter("z_padding", 5)
        self.declare_parameter("wavelength_illumination", 0.532)
        self.declare_parameter("numerical_aperture_illumination", 0.4)
        self.declare_parameter("numerical_aperture_detection", 0.45)
    
    
    def image_callback(self, msg: Image):

        # Copy image data into tensor
        self.image_z_stack[self.image_count,:] = msg.data
        
        #self.image_z_stack[self.image_count] = msg.data
        self.get_logger().info(f"Received image, I now have {self.image_count} images")
        self.get_logger().info(f"{type(msg.data)}")

        if self.image_count == self.get_parameter("zyx_shape").value[0]-1:
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
            zyx_shape=self.get_parameter("zyx_shape").value,
            yx_pixel_size=self.get_parameter("yx_pixel_size").value,
            z_pixel_size=self.get_parameter("z_pixel_size").value,
            wavelength_illumination=self.get_parameter("wavelength_illumination").value,
            z_padding=self.get_parameter("z_padding").value,
            index_of_refraction_media=self.get_parameter("index_of_refraction_media").value,
            numerical_aperture_illumination=self.get_parameter("numerical_aperture_illumination").value,
            numerical_aperture_detection=self.get_parameter("numerical_aperture_detection").value,
        )

        self.get_logger().info(f"Phase calibration generated with shape {self.real_potential_transfer_function.shape}")

    
    def perform_phase_reconstruction(self):
        self.get_logger().info("Performing phase reconstruction")

        tensor_z_stack = torch.tensor(self.image_z_stack.reshape(self.get_parameter("zyx_shape").value), dtype=torch.float32)
        
        # Reconstruct
        zyx_recon = phase_thick_3d.apply_inverse_transfer_function(
            tensor_z_stack,
            self.real_potential_transfer_function,
            self.imag_potential_transfer_function,
            self.get_parameter("z_padding").value,
            self.get_parameter("z_pixel_size").value,
            self.get_parameter("wavelength_illumination").value
        )
        
        self.get_logger().info(f"Reconstructed image w/ with shape {zyx_recon.shape}")

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
