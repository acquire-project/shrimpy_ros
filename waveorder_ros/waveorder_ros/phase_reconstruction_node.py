import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
import numpy as np
import torch
from waveorder.models import phase_thick_3d

class PhaseReconstructionNode(Node):

    def __init__(self):
        super().__init__('phase_reconstruction_node')
        
        self.setup_parameters()

        try:
            from image_transport_py import ImageTransport
            image_transport = ImageTransport('imagetransport_sub', image_transport='compressed')
            image_transport.subscribe('image_raw', 10, self.image_callback)
        except:
            self.get_logger().warning("python image transport not found, falling back on publishing sensor_msgs:msg:Image topic")
            self.sub = self.create_subscription(Image, 'image_raw', self.image_callback, 10)

        
        self.get_logger().info("Generating phase calibration")
        # calculate the phase calibration
        
        # cache the zyx shape as a member, since its used in multiple places
        self.zyx_shape = self.get_parameter("zyx_shape").value
        (z, y, x) = self.zyx_shape  # 
        
        self.image_data_type = None  # This is the data type of the images
        self.image_z_stack = None # This will become the z-stack buffer, but it can't be allocated until the first image is received
        
        self.generate_phase_calibration()

        self.image_count = 0
        self.pub = self.create_publisher(Float32MultiArray, 'phase_reconstruction', 10)
        
        # Initialize the phase reconstruction volume message
        self.phase_recon_msg = Float32MultiArray()
        self.phase_recon_msg.layout.dim = [MultiArrayDimension(), MultiArrayDimension(), MultiArrayDimension()]
        self.phase_recon_msg.layout.dim[0].label = "z"
        self.phase_recon_msg.layout.dim[0].size = z
        self.phase_recon_msg.layout.dim[0].stride = x
        self.phase_recon_msg.layout.dim[1].label = "y"
        self.phase_recon_msg.layout.dim[1].size = y
        self.phase_recon_msg.layout.dim[1].stride = x 
        self.phase_recon_msg.layout.dim[2].label = "x"
        self.phase_recon_msg.layout.dim[2].size = x
        self.phase_recon_msg.layout.dim[2].stride = 1
        
        self.get_logger().info("Phase reconstruction node started")

    def setup_parameters(self):
        # Define the simulation and transfer function arguments as parameters
        # These should be updated to match the microscope and camera setup
        self.declare_parameter("zyx_shape", (11, 2048, 2448))
        self.declare_parameter("yx_pixel_size", 0.345)
        self.declare_parameter("z_pixel_size", 3)
        self.declare_parameter("index_of_refraction_media", 1)
        
        self.declare_parameter("z_padding", 5)
        self.declare_parameter("wavelength_illumination", 0.532)
        self.declare_parameter("numerical_aperture_illumination", 0.4)
        self.declare_parameter("numerical_aperture_detection", 0.45)
    
    
    def image_callback(self, msg: Image):

        if msg.width != self.zyx_shape[2] or msg.height != self.zyx_shape[1]:
            self.get_logger().error(f"Image size mismatch: Expected {self.zyx_shape[1]}x{self.zyx_shape[2]}, got {msg.height}x{msg.width}")
            raise ValueError("Image size mismatch")
        
        # on first image, initialize the image stack
        if self.image_z_stack is None:
            match msg.encoding:
                case 'mono8':
                    self.image_data_type = np.uint8
                case 'mono16':
                    self.image_data_type = np.uint16
                case _:
                    raise ValueError(f"Unsupported encoding: {msg.encoding}")

            self.image_z_stack = np.zeros((self.zyx_shape[0], self.zyx_shape[1]*self.zyx_shape[2]), dtype=self.image_data_type)
            
        # Copy image data into the buffer
        self.image_z_stack[self.image_count,:] = np.frombuffer(msg.data, dtype=self.image_data_type)
        
        self.get_logger().info(f"Received image, I now have {self.image_count+1} images")

        if self.image_count == self.zyx_shape[0]-1:
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
            zyx_shape=self.zyx_shape,
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

        tensor_z_stack = torch.tensor(self.image_z_stack.reshape(self.zyx_shape), dtype=torch.float32)
        
        # Reconstruct
        zyx_recon = phase_thick_3d.apply_inverse_transfer_function(
            tensor_z_stack,
            self.real_potential_transfer_function,
            self.imag_potential_transfer_function,
            self.get_parameter("z_padding").value,
            self.get_parameter("z_pixel_size").value,
            self.get_parameter("wavelength_illumination").value
        )
        
        self.get_logger().info(f"Reconstructed volume w/ with shape {zyx_recon.shape}")

        # Publish the reconstructed phase
        self.phase_recon_msg.data = zyx_recon.numpy().flatten()
        self.pub.publish(self.phase_recon_msg)
        
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
