import os, cv2, time, numpy as np, torch, torchvision.transforms as T
from PIL import Image
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg, CameraInfo
from cv_bridge import CvBridge
from zoedepth.models.builder import build_model
from zoedepth.utils.config import get_config

def make_cam_info(w, h, fx, fy, cx, cy, frame_id):
    msg = CameraInfo()
    msg.width  = w
    msg.height = h
    msg.k = [fx, 0.0, cx,
             0.0, fy, cy,
             0.0, 0.0, 1.0]
    msg.p = [fx, 0.0, cx, 0.0,
             0.0, fy, cy, 0.0,
             0.0, 0.0, 1.0, 0.0]
    msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
    msg.distortion_model = "plumb_bob"
    msg.header.frame_id = frame_id
    return msg

class ZoeRGBDNode(Node):
    def __init__(self):
        super().__init__("zoe_rgbd_node")

        # ------- params -------
        self.declare_parameter("video_source", 0)            # 0=webcam, or path to file
        self.declare_parameter("img_size_h", 192)            # ZoeDepth input (H)
        self.declare_parameter("img_size_w", 320)            # ZoeDepth input (W)
        self.declare_parameter("frame_id", "camera_color_optical_frame")
        self.declare_parameter("depth_scale", 1.0)           # metric scale factor 's' (calibrate!)
        self.declare_parameter("fx", 525.0)
        self.declare_parameter("fy", 525.0)
        self.declare_parameter("cx", 320.0)
        self.declare_parameter("cy", 240.0)
        self.declare_parameter("print_every", 30)

        p = self.get_parameter
        self.img_size = (int(p("img_size_h").value), int(p("img_size_w").value))  # (H,W)
        self.frame_id = p("frame_id").value
        self.depth_scale = float(p("depth_scale").value)
        self.fx, self.fy, self.cx, self.cy = map(float, (p("fx").value, p("fy").value, p("cx").value, p("cy").value))
        self.print_every = int(p("print_every").value)

        src = p("video_source").value
        self.source = int(src) if isinstance(src, int) or (isinstance(src, str) and src.isdigit()) else src

        # Torch threading (leave a core free)
        torch.set_num_threads(max(1, (os.cpu_count() or 4) - 1))
        torch.set_num_interop_threads(1)
        self.get_logger().info(f"torch {torch.__version__} threads={torch.get_num_threads()} interop={torch.get_num_interop_threads()}")

        # ZoeDepth build (NYU v1 weights; DPT_BEiT backbone; 64 bins)
        conf = get_config("zoedepth", "infer", "nyu", version_name="v1")
        conf["midas_model_type"]    = "DPT_BEiT_L_384"
        conf["n_bins"]              = 64
        conf["pretrained_resource"] = "url::https://github.com/isl-org/ZoeDepth/releases/download/v1.0/ZoeD_M12_N.pt"
        t0 = time.perf_counter()
        self.model = build_model(conf).eval()    # CPU
        self.get_logger().info(f"ZoeDepth ready in {(time.perf_counter()-t0)*1e3:.1f} ms")

        # Video
        self.cap = cv2.VideoCapture(self.source)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open video source: {self.source}")
        self.W  = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))  or 640
        self.H  = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 480
        self.fps= float(self.cap.get(cv2.CAP_PROP_FPS) or 30.0)
        self.get_logger().info(f"Video {self.source} {self.W}x{self.H} @ {self.fps:.2f} fps")

        # ROS pubs
        self.bridge = CvBridge()
        self.pub_rgb   = self.create_publisher(ImageMsg,  "/camera/color/image_rect",        10)
        self.pub_depth = self.create_publisher(ImageMsg,  "/camera/aligned_depth_to_color",  10)
        self.pub_info  = self.create_publisher(CameraInfo,"/camera/color/camera_info",       10)

        # Preprocess helpers
        self.to_tensor = T.ToTensor()
        self.frame_i = 0

        # Use a timer to tick at the capture frame rate
        period = 1.0 / max(self.fps, 1.0)
        self.timer = self.create_timer(period, self.tick)

    def preprocess(self, frame):
        # BGR uint8 -> padded to Zoe img size
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(rgb)
        th, tw = self.img_size
        h, w = img.height, img.width
        s = min(tw / w, th / h)
        nw, nh = max(1, int(w*s)), max(1, int(h*s))
        img = img.resize((nw, nh), Image.BILINEAR)
        pad_w, pad_h = tw - nw, th - nh
        canvas = np.zeros((th, tw, 3), dtype=np.uint8)
        t, l = pad_h // 2, pad_w // 2
        canvas[t:t+nh, l:l+nw] = np.asarray(img)
        X = self.to_tensor(canvas)[None]  # [1,3,H,W]
        return X, (t, l, nh, nw)

    def post(self, depth_tensor, pad):
        # remove pad, resize back to original (H,W)
        t, l, nh, nw = pad
        d = depth_tensor.squeeze().detach().cpu().numpy()
        d = d[t:t+nh, l:l+nw]
        return cv2.resize(d, (self.W, self.H), interpolation=cv2.INTER_CUBIC)

    @torch.inference_mode()
    def tick(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().info("End of stream")
            rclpy.shutdown()
            return

        self.frame_i += 1
        stamp = self.get_clock().now().to_msg()

        # RGB publish
        rgb_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        rgb_msg.header.stamp = stamp
        rgb_msg.header.frame_id = self.frame_id
        self.pub_rgb.publish(rgb_msg)

        # Depth inference -> meters + NaNs
        X, pad = self.preprocess(frame)
        depth_pred = self.model.infer(X, pad_input=False)   # ZoeDepth tensor
        d = self.post(depth_pred, pad).astype(np.float32)
        d *= float(self.depth_scale)                         # calibrate metric scale
        d[d <= 0.0] = np.nan                                 # 32FC1 invalids as NaN

        depth_msg = self.bridge.cv2_to_imgmsg(d, encoding="32FC1")
        depth_msg.header.stamp = stamp                       # EXACT SAME stamp as RGB
        depth_msg.header.frame_id = self.frame_id
        self.pub_depth.publish(depth_msg)

        # CameraInfo (intrinsics for the RGB image)
        info = make_cam_info(self.W, self.H, self.fx, self.fy, self.cx, self.cy, self.frame_id)
        info.header.stamp = stamp
        self.pub_info.publish(info)

        if self.frame_i % max(self.print_every,1) == 0:
            self.get_logger().info(f"pub frame {self.frame_i}")

def main():
    rclpy.init()
    node = ZoeRGBDNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
